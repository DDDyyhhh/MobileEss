"""One bounded reviewed FollowPath trial; no CAN transmission, unlock or retry."""
import argparse
import json
import math
from pathlib import Path
import platform
import signal
import socket
import struct
import time

from mower_nav2.reviewed_route import RouteGuard,can_control_valid,validate_review
from mower_nav2.obstacle_wait import (
    ObstacleWait, finish_controller_pause, goal_uuid, remaining_straight_path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--observe-only',action='store_true')
    args=parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():
        parser.error('Require RK3588 and new evidence')
    p=json.loads(args.proposal.read_text())
    try:reference,execution=validate_review(p,require_physical_confirmation=not args.observe_only)
    except (ValueError,KeyError,TypeError) as exc:parser.error(str(exc))
    if not args.observe_only and not p.get('motion_approved'):
        parser.error('Fresh whole-route motion approval required')
    import rclpy
    from rclpy.action import ActionClient
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped,Twist
    from nav_msgs.msg import Odometry,Path as RosPath
    from nav2_msgs.action import FollowPath
    from nav2_msgs.msg import Costmap
    from sensor_msgs.msg import Imu
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('reviewed_route_check')
    if args.observe_only and 'cmd_vel_can_bridge' in node.get_node_names():
        node.destroy_node();rclpy.shutdown()
        raise RuntimeError('No-motion review requires command bridge absent')
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    begin=time.monotonic();poses={};pose_dirty=False;accepted=0;last_accept=0.
    mode=None;wheels=None;last_status=last_wheel=gate_time=0.
    zero_wheels=zero_commands=0;stopping=False;request_seen=False
    wait_policy = ObstacleWait() if p['route_profile'] == 'static_obstacle_wait_straight_03' else None
    wait_active = False; hold_zero = False; last_wait_state = None
    probe_odom = None; probe_map = None; probe_map_time = 0.
    if wait_policy is not None:
        # Observation only: call the unchanged gate's predictor; never publish
        # the probe velocity. Its fixed forward probe still detects a box when
        # Nav2 or the gate is already outputting zero.
        from mower_nav2.collision_gate import CollisionGate
        probe = object.__new__(CollisionGate)
        probe._radius = .7; probe._horizon = 2.; probe._dt = .05
        probe._collision_cost = 253; probe._last_reason = ''
        probe_command = Twist(); probe_command.linear.x = .3
    faults=[];guard=RouteGuard(reference,p['max_travel_m']);handle=None;last_step=time.monotonic()
    result=dict(observe_only=args.observe_only,completed=False,dry_run_completed=False,
                automatic_unlock=False,automatic_retry=False,follow_path_goal_sent=False,
                nonzero_gated_samples=0,events=[])
    def record(**fields):result['events'].append(dict(t_s=time.monotonic()-begin,**fields))
    def pose_cb(key,msg):
        nonlocal pose_dirty, probe_odom
        v=msg.pose.pose;q=v.orientation
        poses[key]=(v.position.x,v.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        pose_dirty=True
        if key == 'odom': probe_odom = msg
    def robot():
        if len(poses)!=2:return None
        tx,ty,a=poses['map'];x,y,b=poses['odom']
        return [tx+math.cos(a)*x-math.sin(a)*y,ty+math.sin(a)*x+math.cos(a)*y,a+b]
    def loc_cb(msg):
        nonlocal accepted,last_accept
        for s in msg.status:
            if s.message=='accepted':accepted+=1;last_accept=time.monotonic()
    def cmd_cb(msg):
        if (not all(math.isfinite(v) for v in (msg.linear.x,msg.angular.z)) or
                not -.001<=msg.linear.x<=p['max_linear_speed_mps']+.001 or abs(msg.angular.z)>.201):
            faults.append('Gated velocity outside reviewed ceiling')
        if abs(msg.linear.x)>.001 or abs(msg.angular.z)>.001:
            result['nonzero_gated_samples']+=1
        record(cmd_vel_gated=[msg.linear.x,msg.angular.z])
    gate_values={};gate_reason=''
    def gate_cb(msg):
        nonlocal gate_values,gate_reason,gate_time,request_seen
        for s in msg.status:
            if s.name!='mower_nav2/collision_gate':continue
            gate_time=time.monotonic();gate_values={v.key:v.value for v in s.values};gate_reason=s.message
            try:
                request_seen=request_seen or abs(float(gate_values['input_linear_x']))>.001 or abs(float(gate_values['input_angular_z']))>.001
            except (ValueError,KeyError):faults.append('Invalid collision gate diagnostics')
            record(gate_reason=gate_reason,values=gate_values)
    node.create_subscription(Odometry,'/fixed_map_localization/odometry',lambda m:pose_cb('map',m),10)
    node.create_subscription(Odometry,'/odometry/filtered',lambda m:pose_cb('odom',m),10)
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',loc_cb,10)
    node.create_subscription(DiagnosticArray,'/collision_gate/diagnostics',gate_cb,10)
    node.create_subscription(Twist,'/cmd_vel_gated',cmd_cb,10)
    if wait_policy is not None:
        def probe_map_cb(msg):
            nonlocal probe_map, probe_map_time
            probe_map = msg; probe_map_time = time.monotonic()
        node.create_subscription(Costmap,'/local_costmap/costmap_raw',probe_map_cb,1)
    # Preserve independent yaw evidence for curve tracking diagnosis.
    last_imu_record=0.
    def imu_cb(msg):
        nonlocal last_imu_record
        now=time.monotonic()
        if now-last_imu_record<.02:return
        last_imu_record=now
        record(imu_stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
               imu_frame=msg.header.frame_id,
               imu_angular_velocity_rad_s=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
    def lio_cb(msg):
        v=msg.pose.pose;q=v.orientation
        record(fast_lio_pose=[v.position.x,v.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
               fast_lio_stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
    node.create_subscription(Imu,'/livox/imu',imu_cb,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/fast_lio/odometry',lio_cb,20)
    bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);bus.bind(('can0',));bus.setblocking(False)
    def step(protect=False):
        nonlocal mode,wheels,last_status,last_wheel,zero_wheels,zero_commands,pose_dirty,last_step,hold_zero,last_wait_state
        if hold_zero: pub.publish(Twist())
        now=time.monotonic()
        if protect and now-last_step>.25:faults.append('Supervisor event loop stalled')
        last_step=now;rclpy.spin_once(node,timeout_sec=.01)
        for _ in range(256):
            try:raw=bus.recv(16)
            except BlockingIOError:break
            cid,dlc,data=struct.unpack('=IB3x8s',raw)
            if cid not in (5,0x507,0x50a):continue
            if dlc!=8:faults.append('Invalid CAN DLC');continue
            now=time.monotonic();record(can_id=hex(cid),payload=data.hex())
            if cid==5:
                if args.observe_only:faults.append('Unexpected command CAN during no-motion review')
                elif not can_control_valid(data,round(p['max_linear_speed_mps']*10)):
                    faults.append('CAN command boundary violated')
                if stopping:zero_commands=zero_commands+1 if data[1]==data[2]==data[3]==0 else 0
            elif cid==0x507:mode=data[4];last_status=now
            else:
                wheels=list(struct.unpack('>hhhh',data));last_wheel=now
                if any(abs(v)>550 for v in wheels):faults.append('Wheel ceiling exceeded')
                if args.observe_only and any(abs(v)>10 for v in wheels):faults.append('Wheel response during no-motion review')
                if stopping:zero_wheels=zero_wheels+1 if all(abs(v)<=10 for v in wheels) else 0
        if protect:
            now=time.monotonic();expected_mode=0 if args.observe_only else 2
            if mode!=expected_mode:faults.append('API ownership changed; no reacquisition')
            if now-last_accept>2.5 or now-last_status>.25 or now-last_wheel>.25:
                faults.append('Localization or CAN telemetry stale')
            if now-gate_time>.3:faults.append('Collision gate diagnostics stale')
            try:
                for key in ('costmap_age_sec','odom_age_sec','obstacle_age_sec'):
                    value=float(gate_values[key])
                    if not math.isfinite(value) or not 0<=value<=.3:faults.append('Collision gate evidence stale')
                if request_seen and not stopping and (gate_reason.startswith('stale ') or gate_reason.startswith('missing ')):
                    faults.append('Collision gate input fault; no automatic recovery')
            except (KeyError,ValueError):faults.append('Collision gate evidence unavailable')
            pos=robot()
            if pos is None:faults.append('Robot pose unavailable')
            elif pose_dirty:
                try:
                    distance,progress=guard.update(pos[:2]);record(robot_in_map=pos,route_distance_m=distance,route_progress_m=progress,
                        filtered_odom=poses['odom'],map_to_odom=poses['map'])
                except ValueError as exc:
                    record(guard_rejected_pose=pos,route_guard_error=str(exc),
                        filtered_odom=poses['odom'],map_to_odom=poses['map'])
                    faults.append(str(exc))
                pose_dirty=False
            if wait_active and wait_policy is not None and not stopping and not faults:
                if (probe_map is None or probe_odom is None or now-probe_map_time>.3 or
                        probe_map.header.frame_id != probe_odom.header.frame_id):
                    faults.append('Wait probe costmap missing, stale or wrong frame')
                else:
                    probe._costmap = probe_map; probe._odom = probe_odom
                    blocked = probe._collision(probe_command)
                    if blocked and not probe._last_reason.startswith('predicted footprint cost '):
                        faults.append('Wait probe failed: '+probe._last_reason)
                    state = wait_policy.update(now, blocked=blocked,
                        stationary=all(abs(v)<=10 for v in wheels), healthy=not faults)
                    if state != 'following': hold_zero = True; pub.publish(Twist())
                    if state != last_wait_state:
                        record(obstacle_wait_state=state,probe_reason=probe._last_reason,
                               resume_count=wait_policy.resume_count,
                               original_goal_xy_m=p['goal_xy_m'])
                        last_wait_state = state
                    if state == 'fault': faults.append(wait_policy.error)
        if faults:raise RuntimeError(faults[0])
    def await_future(future,seconds,protect=True):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:step(protect)
        if not future.done():raise RuntimeError('Action deadline exceeded')
        return future.result()
    def interrupted(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        end=time.monotonic()+7
        while time.monotonic()<end and (accepted<3 or robot() is None or wheels is None):step()
        current=robot();result['preflight_observed_pose']=current
        expected=0 if args.observe_only else 2
        if current is None or math.dist(current[:2],p['pose']['xy_m'])>.15:
            raise RuntimeError('Current position differs from reviewed start')
        yaw=math.radians(p['pose']['yaw_deg'])
        if abs(math.atan2(math.sin(current[2]-yaw),math.cos(current[2]-yaw)))>.1:
            raise RuntimeError('Current heading differs from reviewed start')
        if accepted<3 or mode!=expected or wheels is None or any(abs(v)>10 for v in wheels):
            raise RuntimeError('Require accepted localization and stationary acknowledged API')
        step(True)
        client=ActionClient(node,FollowPath,'/follow_path')
        if not client.wait_for_server(timeout_sec=3):raise RuntimeError('FollowPath server unavailable')
        last_step=time.monotonic()
        path=RosPath();path.header.frame_id='map';path.header.stamp=node.get_clock().now().to_msg()
        for x,y,a in execution:
            pose=PoseStamped();pose.header=path.header;pose.pose.position.x=x;pose.pose.position.y=y
            pose.pose.orientation.z=math.sin(a/2);pose.pose.orientation.w=math.cos(a/2);path.poses.append(pose)
        def send_path(current_path):
            goal=FollowPath.Goal();goal.path=current_path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
            accepted_handle=await_future(client.send_goal_async(goal),5)
            if not accepted_handle.accepted:raise RuntimeError('FollowPath rejected')
            record(controller_goal_uuid=goal_uuid(accepted_handle),original_goal_xy_m=p['goal_xy_m'])
            return accepted_handle
        handle=send_path(path)
        result['follow_path_goal_sent']=True;result['start_pose']=current
        finished=handle.get_result_async();started=time.monotonic()
        wait_active = wait_policy is not None and not args.observe_only
        duration=4. if args.observe_only else p['max_run_seconds']
        while time.monotonic()-started<duration:
            step(True)
            if wait_active and wait_policy.state in ('waiting', 'resume'):
                if handle is not None:
                    # Cancel the follower before its 15 s progress limit. This
                    # is one planned pause, not a retry of a failed action.
                    finish_controller_pause(handle,finished,await_future)
                    record(obstacle_pause_cancelled=True)
                    handle=None
                if wait_policy.state == 'resume':
                    remaining=remaining_straight_path(execution,robot())
                    resumed=RosPath();resumed.header.frame_id='map';resumed.header.stamp=node.get_clock().now().to_msg()
                    for x,y,a in remaining:
                        v=PoseStamped();v.header=resumed.header;v.pose.position.x=x;v.pose.position.y=y
                        v.pose.orientation.z=math.sin(a/2);v.pose.orientation.w=math.cos(a/2);resumed.poses.append(v)
                    # Policy will return to following on the next fresh tick.
                    # Stop pumping during send; the collision gate remains live.
                    wait_policy.acknowledge_resume();hold_zero=False
                    handle=send_path(resumed);finished=handle.get_result_async()
                    record(obstacle_resume_sent=True,remaining_points=len(remaining),original_goal_xy_m=p['goal_xy_m'])
                continue
            if finished.done(): break
        if args.observe_only:
            if finished.done():raise RuntimeError('Controller ended before passive observation completed')
            await_future(handle.cancel_goal_async(),3)
            if result['nonzero_gated_samples']<3:raise RuntimeError('No live gated controller output observed')
            result['dry_run_completed']=True
        else:
            if not finished.done():raise RuntimeError('Reviewed route time budget exceeded')
            response=finished.result();result['action_status']=response.status
            if response.status!=4:raise RuntimeError('FollowPath did not succeed')
            if wait_active and wait_policy.resume_count != 1:
                raise RuntimeError('Goal reached without the required obstacle wait/resume episode')
            stopping=True;zero_wheels=zero_commands=0;end=time.monotonic()+2
            while time.monotonic()<end:step(True)
            final=robot();result['final_pose']=final
            result['goal_error_m']=math.dist(final[:2],p['goal_xy_m'])
            target=math.radians(p['goal_yaw_deg']);error=abs(math.atan2(math.sin(final[2]-target),math.cos(final[2]-target)))
            result['goal_yaw_error_deg']=math.degrees(error)
            if result['goal_error_m']>.25 or error>.15 or zero_wheels<3 or zero_commands<3:
                raise RuntimeError('Final pose or continuous stop outside acceptance')
            result['completed']=True
    except (Exception,KeyboardInterrupt) as exc:result['error']=str(exc) or 'interrupted'
    finally:
        if handle is not None and handle.accepted:
            # Existing faults must not prevent pumping the cancellation ack.
            try:
                cancel=handle.cancel_goal_async();deadline=time.monotonic()+3
                while not cancel.done() and time.monotonic()<deadline:
                    pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
                result['cancel_ack_received']=cancel.done()
            except (Exception,KeyboardInterrupt):result['cancel_ack_received']=False
        end=time.monotonic()+.5
        while time.monotonic()<end:
            pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
        result.update(final_api_mode=mode,final_wheel_mm_s=wheels,travel_m=guard.travel,
                      maximum_route_progress_m=guard.peak_progress)
        if wait_policy is not None:
            result.update(obstacle_wait_state=wait_policy.state,
                          obstacle_resume_count=wait_policy.resume_count,
                          original_goal_xy_m=p['goal_xy_m'],
                          one_task_with_planned_controller_pause=True)
        try:
            serialized=json.dumps(result,indent=2)
            with args.output.open('x') as f:f.write(serialized)
            print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)
        finally:
            bus.close();node.destroy_node();rclpy.shutdown()
    return 0 if result['dry_run_completed'] or result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
