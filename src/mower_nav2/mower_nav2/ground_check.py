"""One reviewed short ground goal; no retries, CAN transmission or unlock."""
import argparse
import json
import math
from pathlib import Path
import platform
import signal
import socket
import struct
import time


def corridor_error(start, goal, pose, width=0.25):
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    length = math.hypot(dx, dy)
    if length <= 0 or not all(math.isfinite(v) for v in (*start, *goal, *pose)):
        return 'invalid corridor'
    along = ((pose[0]-start[0])*dx+(pose[1]-start[1])*dy)/length
    lateral = abs((pose[0]-start[0])*dy-(pose[1]-start[1])*dx)/length
    if along < -0.10 or along > length+0.15 or lateral > width:
        return 'outside reviewed short corridor'
    return None


def heading_error(start_yaw, goal_yaw, current_yaw, margin=0.35):
    if not all(math.isfinite(v) for v in (start_yaw, goal_yaw, current_yaw, margin)) or margin < 0:
        return 'invalid heading'
    delta = math.atan2(math.sin(goal_yaw-start_yaw), math.cos(goal_yaw-start_yaw))
    actual = math.atan2(math.sin(current_yaw-start_yaw), math.cos(current_yaw-start_yaw))
    if not min(0.0, delta)-margin <= actual <= max(0.0, delta)+margin:
        return 'Heading leaves reviewed turn range'
    return None


def needs_initial_alignment(start_yaw, goal_yaw):
    if not all(math.isfinite(v) for v in (start_yaw, goal_yaw)):
        return True
    return abs(math.atan2(math.sin(goal_yaw-start_yaw), math.cos(goal_yaw-start_yaw))) > 0.10


def sampled_path_point(path, distance=0.3):
    """Use the same first-point distance convention as Humble RotationShim."""
    if len(path) < 2 or not math.isfinite(distance) or distance <= 0:
        raise ValueError('Invalid initial alignment path')
    if any(len(point) != 2 or not all(math.isfinite(v) for v in point) for point in path):
        raise ValueError('Invalid initial alignment path')
    for point in path[1:]:
        if math.hypot(point[0]-path[0][0], point[1]-path[0][1]) >= distance:
            return point
    raise ValueError('Path too short for initial alignment sample')


def initial_path_alignment_error(pose, target):
    if pose is None or target is None or not all(math.isfinite(v) for v in (*pose, *target)):
        return 'Invalid initial alignment geometry'
    dx, dy = target[0]-pose[0], target[1]-pose[1]
    if math.hypot(dx, dy) < 0.01:
        return 'Invalid initial alignment geometry'
    delta = math.atan2(dy, dx)-pose[2]
    if abs(math.atan2(math.sin(delta), math.cos(delta))) > 0.10:
        return 'Initial path heading not aligned'
    return None


def navigation_limits(proposal):
    """Keep the historical ceiling unless the scoped 0.4 trial is explicit."""
    profile = proposal.get('navigation_profile', 'first_ground')
    if profile == 'first_ground':
        return .1, 1, 250
    if (profile == 'short_04' and proposal.get('max_linear_speed_mps') == .4
            and proposal.get('max_angular_speed_radps') == .2
            and proposal.get('corridor_half_width_m') == .2):
        return .4, 4, 550
    raise ValueError('Unsupported short navigation profile')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64'):
        parser.error('Run hardware checks on RK3588 only')
    if args.output.exists():
        parser.error('Use a new evidence file')
    proposal = json.loads(args.proposal.read_text())
    try:
        max_linear, raw_linear, wheel_ceiling = navigation_limits(proposal)
    except ValueError as exc:
        parser.error(str(exc))
    if not proposal.get('motion_approved') or not 0.5 <= proposal.get('distance_m', 0) <= 1.0:
        parser.error('Require explicit approval for a short segment between 0.5 and 1 metre')
    start = proposal['pose']['xy_m']
    goal_xy = proposal['proposed_target_xy_m']
    start_yaw = math.radians(proposal['pose']['yaw_deg'])
    yaw = math.radians(proposal.get('goal_yaw_deg', proposal['pose']['yaw_deg']))
    heading_margin = proposal.get('heading_margin_rad', 0.35)
    corridor_width = proposal.get('corridor_half_width_m', 0.25)
    yaw_tolerance = proposal.get('goal_yaw_tolerance_rad', 0.25)
    require_initial_alignment = needs_initial_alignment(start_yaw, yaw) or bool(
        proposal.get('require_initial_heading_alignment', False))
    initial_heading_aligned = not require_initial_alignment
    initial_alignment_target = None
    values = (*start, *goal_xy, start_yaw, yaw, heading_margin, corridor_width, yaw_tolerance)
    if not all(math.isfinite(value) for value in values):
        parser.error('Short-test geometry must be finite')
    actual_distance = math.hypot(goal_xy[0]-start[0], goal_xy[1]-start[1])
    if abs(actual_distance-proposal['distance_m']) > 0.1 or not 0 < yaw_tolerance <= 0.25:
        parser.error('Target geometry or final heading tolerance exceeds reviewed limits')
    if not 0 < corridor_width <= 0.25 or not 0 < heading_margin <= 0.35:
        parser.error('Reviewed corridor or heading margin exceeds short-test limits')
    delta = math.atan2(math.sin(yaw-start_yaw), math.cos(yaw-start_yaw))
    if abs(delta) > math.pi/3:
        parser.error('First turning test supports at most 60 degrees')
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.action import ActionClient
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav_msgs.msg import Odometry
    from mcu_can_bridge.protocol import parse_wheel_feedback
    rclpy.init()
    node = rclpy.create_node('ground_short_acceptance')
    begin = time.monotonic()
    events, poses, faults = [], {}, []
    accepted = 0
    last_accept = last_wheel = last_status = 0.0
    mode = None
    wheels = None
    api_running = False
    handle = None
    result = {'navigation_goal_sent': False, 'automatic_unlock': False,
              'goal_xy_m': goal_xy, 'goal_yaw_deg': math.degrees(yaw), 'approved_start_xy_m': start,
              'max_linear_speed_mps': max_linear, 'completed': False,
              'initial_heading_alignment_required': require_initial_alignment}
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind(('can0',))
    bus.setblocking(False)

    def pose_cb(key, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses[key] = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def loc_cb(msg):
        nonlocal accepted, last_accept
        for status in msg.status:
            events.append({'t_s': time.monotonic()-begin, 'localization': status.message})
            if status.message == 'accepted':
                accepted += 1
                last_accept = time.monotonic()

    def cmd_cb(msg):
        nonlocal initial_heading_aligned
        current = robot()
        if current and initial_path_alignment_error(current, initial_alignment_target) is None:
            initial_heading_aligned = True
        if not initial_heading_aligned and msg.linear.x > 0.001:
            faults.append('Forward command before initial heading alignment')
        events.append({'t_s': time.monotonic()-begin, 'cmd_vel_gated': [msg.linear.x, msg.angular.z]})
        if not (-0.001 <= msg.linear.x <= max_linear+.001 and abs(msg.angular.z) <= 0.201):
            faults.append('Velocity outside first ground ceiling')

    node.create_subscription(Odometry, '/fixed_map_localization/odometry', lambda m: pose_cb('map', m), 10)
    node.create_subscription(Odometry, '/odometry/filtered', lambda m: pose_cb('odom', m), 10)
    node.create_subscription(DiagnosticArray, '/fixed_map_localization/diagnostics', loc_cb, 10)
    node.create_subscription(Twist, '/cmd_vel_gated', cmd_cb, 10)

    def robot():
        if len(poses) != 2:
            return None
        tx, ty, a = poses['map']; x, y, b = poses['odom']
        return [tx+math.cos(a)*x-math.sin(a)*y, ty+math.sin(a)*x+math.cos(a)*y, a+b]

    def step(guard=False):
        nonlocal mode, wheels, last_status, last_wheel, api_running
        rclpy.spin_once(node, timeout_sec=0.02)
        for _ in range(256):
            try:
                raw = bus.recv(16)
            except BlockingIOError:
                break
            cid, dlc, data = struct.unpack('=IB3x8s', raw)
            if cid not in (0x005, 0x507, 0x50A) or dlc != 8:
                continue
            now = time.monotonic()
            events.append({'t_s': now-begin, 'can_id': hex(cid), 'payload': data.hex()})
            if cid == 0x507:
                mode, last_status = data[4], now
                if api_running and mode != 2:
                    faults.append('API exited; no automatic reacquisition')
                api_running = api_running or mode == 2
            elif cid == 0x50A:
                wheels, last_wheel = parse_wheel_feedback(data), now
                if any(abs(v) > wheel_ceiling for v in wheels):
                    faults.append('Wheel speed exceeds ground test ceiling')
            else:
                vx = int.from_bytes(data[1:2], 'big', signed=True)
                wz = int.from_bytes(data[3:4], 'big', signed=True)
                if not (0 <= vx <= raw_linear and abs(wz) <= 2 and data[2] == data[4] == data[7] == 0):
                    faults.append('CAN payload violates speed/blade/unlock boundary')
        if guard:
            now = time.monotonic()
            if now-last_accept > 2.5 or now-last_wheel > 0.25 or now-last_status > 0.25:
                faults.append('Localization or CAN telemetry stale')
            p = robot()
            if p is None or corridor_error(start, goal_xy, p[:2], corridor_width):
                faults.append('Pose outside reviewed corridor')
            elif heading_error(start_yaw, yaw, p[2], heading_margin):
                faults.append('Heading leaves reviewed short segment')
            if p:
                events.append({'t_s': now-begin, 'robot_in_map': p})
            if faults:
                raise RuntimeError(faults[0])

    def await_future(future, seconds, guard=False):
        deadline = time.monotonic()+seconds
        while not future.done() and time.monotonic() < deadline:
            step(guard)
        if not future.done():
            raise RuntimeError('ROS action deadline exceeded')
        return future.result()

    def interrupted(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        if not navigator.wait_for_server(timeout_sec=5) or not planner.wait_for_server(timeout_sec=5):
            raise RuntimeError('Navigation/planning server unavailable')
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and (accepted < 3 or robot() is None):
            step()
        p = robot()
        if p is None or math.hypot(p[0]-start[0], p[1]-start[1]) > 0.20:
            raise RuntimeError('Current pose differs from reviewed start')
        if mode != 2 or wheels is None or any(abs(v) > 10 for v in wheels):
            raise RuntimeError('Require acknowledged API run with stationary wheels')
        step(True)
        result['start_pose'] = p
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = goal_xy
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        goal.use_start = False
        planning = await_future(planner.send_goal_async(goal), 5, True)
        if not planning.accepted:
            raise RuntimeError('Short path rejected')
        planned = await_future(planning.get_result_async(), 8, True)
        path = [[p.pose.position.x, p.pose.position.y] for p in planned.result.path.poses]
        result['planned_path_xy_m'] = path
        if planned.status != 4 or not path or any(corridor_error(start, goal_xy, p, corridor_width) for p in path):
            raise RuntimeError('Planned path leaves reviewed short corridor')
        if require_initial_alignment:
            initial_alignment_target = sampled_path_point(path)
            result['initial_alignment_target_xy_m'] = list(initial_alignment_target)
        navigation = NavigateToPose.Goal()
        navigation.pose = goal.goal
        # First trial excludes recovery spin/backup outside the reviewed segment.
        navigation.behavior_tree = str(Path(get_package_share_directory('mower_nav2')) / 'behavior_trees/navigate_ground_short.xml')
        handle = await_future(navigator.send_goal_async(navigation), 5, True)
        if not handle.accepted:
            raise RuntimeError('Navigation goal rejected')
        result['navigation_goal_sent'] = True
        finished = await_future(handle.get_result_async(), 30, True)
        result['action_status'] = finished.status
        if finished.status != 4:
            raise RuntimeError('Navigation did not report success')
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            step(True)
        p = robot()
        result['final_pose'] = p
        result['goal_error_m'] = math.hypot(p[0]-goal_xy[0], p[1]-goal_xy[1])
        result['goal_yaw_error_deg'] = math.degrees(abs(math.atan2(math.sin(p[2]-yaw), math.cos(p[2]-yaw))))
        if result['goal_yaw_error_deg'] > math.degrees(yaw_tolerance):
            raise RuntimeError('Final heading outside reviewed tolerance')
        if result['goal_error_m'] > 0.25 or any(abs(v) > 10 for v in wheels):
            raise RuntimeError('Goal error or final wheel speed outside acceptance')
        result['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or 'interrupted'
    finally:
        if handle is not None and handle.accepted:
            try:
                await_future(handle.cancel_goal_async(), 3)
            except (Exception, KeyboardInterrupt):
                pass
        result.update(events=events, final_api_mode=mode, final_wheel_mm_s=list(wheels) if wheels else None)
        args.output.write_text(json.dumps(result, indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('events','planned_path_xy_m')}), flush=True)
        bus.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
