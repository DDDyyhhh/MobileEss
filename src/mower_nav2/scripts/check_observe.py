"""Bounded live localization and ComputePathToPose observation; no motion goal."""
import argparse
import json
import math
from pathlib import Path
import platform
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--goal-x', type=float, default=5.0)
    parser.add_argument('--goal-y', type=float, default=-6.0)
    parser.add_argument('--goal-yaw-deg', type=float, default=0.0)
    parser.add_argument('--relative-forward-m', type=float)
    parser.add_argument('--relative-right-m', type=float)
    parser.add_argument('--relative-yaw-deg', type=float, default=-35.0)
    args = parser.parse_args()
    if (args.relative_forward_m is None) != (args.relative_right_m is None):
        parser.error('Relative forward and right offsets must be supplied together')
    if platform.machine() not in ('aarch64', 'arm64'):
        raise SystemExit('Run on RK3588')
    import rclpy
    from rclpy.action import ActionClient
    from diagnostic_msgs.msg import DiagnosticArray
    from nav2_msgs.action import ComputePathToPose
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node('field_observe_check')
    events, edges = [], {}
    poses = {}
    def on_pose(key, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses[key] = {'xy_m': [p.x, p.y], 'yaw_rad': math.atan2(
            2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
    node.create_subscription(Odometry, '/fixed_map_localization/odometry',
        lambda msg: on_pose('map_to_odom', msg), 10)
    node.create_subscription(Odometry, '/odometry/filtered',
        lambda msg: on_pose('odom_to_base', msg), 10)
    node.create_subscription(DiagnosticArray, '/fixed_map_localization/diagnostics',
        lambda msg: events.extend(s.message for s in msg.status), 10)
    def on_tf(msg):
        for tf in msg.transforms:
            key = f'{tf.header.frame_id}->{tf.child_frame_id}'
            edges[key] = edges.get(key, 0)+1
    node.create_subscription(TFMessage, '/tf', on_tf, 100)
    begin = time.monotonic()
    while time.monotonic()-begin < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = {'events': events, 'tf_message_counts': edges, 'path_computation_only': True,
              'accepted_count': events.count('accepted')}
    if len(poses) == 2:
        t, o = poses['map_to_odom'], poses['odom_to_base']
        x, y = o['xy_m']
        tx, ty = t['xy_m']
        yaw = t['yaw_rad']
        result['robot_in_map'] = {'xy_m': [tx+math.cos(yaw)*x-math.sin(yaw)*y,
            ty+math.sin(yaw)*x+math.cos(yaw)*y], 'yaw_deg': math.degrees(yaw+o['yaw_rad'])}
    if args.relative_forward_m is not None and result.get('robot_in_map'):
        pose = result['robot_in_map']
        yaw = math.radians(pose['yaw_deg'])
        args.goal_x = pose['xy_m'][0] + math.cos(yaw)*args.relative_forward_m + math.sin(yaw)*args.relative_right_m
        args.goal_y = pose['xy_m'][1] + math.sin(yaw)*args.relative_forward_m - math.cos(yaw)*args.relative_right_m
        args.goal_yaw_deg = pose['yaw_deg'] + args.relative_yaw_deg
    result['planning_goal'] = {'xy_m': [args.goal_x, args.goal_y], 'yaw_deg': args.goal_yaw_deg}
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if result['accepted_count'] >= 3 and 'robot_in_map' in result and client.wait_for_server(timeout_sec=5):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = args.goal_x, args.goal_y
        yaw = math.radians(args.goal_yaw_deg)
        goal.goal.pose.orientation.z = math.sin(yaw/2)
        goal.goal.pose.orientation.w = math.cos(yaw/2)
        goal.use_start = False
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if future.done() and future.result() and future.result().accepted:
            completed = future.result().get_result_async()
            rclpy.spin_until_future_complete(node, completed, timeout_sec=15)
            if completed.done() and completed.result():
                response = completed.result()
                path = response.result.path.poses
                result.update(action_status=response.status, path_poses=len(path),
                              path_xy_m=[[p.pose.position.x, p.pose.position.y] for p in path],
                              path_length_m=sum(math.hypot(b.pose.position.x-a.pose.position.x,
                                  b.pose.position.y-a.pose.position.y) for a,b in zip(path,path[1:])))
    result['node_names'] = node.get_node_names()
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps({key: value for key, value in result.items() if key != 'path_xy_m'}), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
