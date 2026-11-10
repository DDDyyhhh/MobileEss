"""Single reviewed straight obstacle-stop check through collision_gate; no CAN unlock."""
import argparse
import json
import math
from pathlib import Path
import platform
import signal
import socket
import struct
import time
from mower_nav2.ground_check import corridor_error


def obstacle_blocks(values, message, expected_speed=.1):
    try:
        return (message.startswith('predicted footprint cost ') and
            abs(float(values['input_linear_x'])-expected_speed) <= .001 and abs(float(values['input_angular_z'])) <= .001 and
            abs(float(values['output_linear_x'])) <= .001 and abs(float(values['output_angular_z'])) <= .001 and
            all(0 <= float(values[k]) <= .3 for k in ('command_age_sec','costmap_age_sec','odom_age_sec','obstacle_age_sec')))
    except (KeyError, ValueError, TypeError):
        return False


def obstacle_profile(proposal):
    profile = proposal.get('obstacle_profile', 'first_ground')
    speed = proposal.get('max_linear_speed_mps', .1)
    duration = proposal.get('max_drive_seconds', 12)
    ceiling = proposal.get('max_approach_distance_m', .9)
    tolerance = proposal.get('preflight_wheel_limit_mm_s', 10)
    if profile == 'first_ground' and tolerance == 10 and (speed, duration, ceiling) == (.1, 12, .9):
        return .1, 12., .9, 250, 1
    if profile == 'obstacle_04' and tolerance in (10,50) and (speed, duration, ceiling) == (.4, 4, .9):
        return .4, 4., .9, 550, 4
    raise ValueError('Unsupported bounded obstacle-stop profile')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():
        parser.error('Run on RK3588 with new evidence')
    p=json.loads(args.proposal.read_text())
    if not p.get('motion_approved') or p.get('test_kind')!='straight_obstacle_stop':
        parser.error('Require reviewed obstacle-stop approval')
    try:
        speed, duration, ceiling, wheel_ceiling, raw_speed = obstacle_profile(p)
    except ValueError as exc:
        parser.error(str(exc))
    startup_wheel_limit=p.get('preflight_wheel_limit_mm_s',10)
    start=p['pose']['xy_m']; yaw=math.radians(p['pose']['yaw_deg'])
    goal=[start[0]+math.cos(yaw),start[1]+math.sin(yaw)]
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from mcu_can_bridge.protocol import parse_wheel_feedback
    rclpy.init(); node=rclpy.create_node('ground_obstacle_stop_check')
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    poses={}; loc_time=0.; accepted=0; gate_time=0.; gate_blocked=False
    result={'test_kind':'straight_obstacle_stop','speed_command_m_s':speed,'maximum_drive_seconds':duration,'maximum_approach_distance_m':ceiling,'preflight_wheel_limit_mm_s':startup_wheel_limit,'stop_wheel_limit_mm_s':10,'nav2_goal_sent':False,'automatic_unlock':False,'automatic_resume':False,'completed':False,'events':[]}
    def pose_cb(key,msg):
        v=msg.pose.pose;p=v.position;q=v.orientation
        poses[key]=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def robot():
        if len(poses)!=2:return None
        tx,ty,a=poses['map'];x,y,b=poses['odom']
        return [tx+math.cos(a)*x-math.sin(a)*y,ty+math.sin(a)*x+math.cos(a)*y,a+b]
    def loc_cb(msg):
        nonlocal loc_time,accepted
        for s in msg.status:
            if s.message=='accepted':loc_time=time.monotonic();accepted+=1
    def gate_cb(msg):
        nonlocal gate_time,gate_blocked
        for s in msg.status:
            if s.name=='mower_nav2/collision_gate':
                values={v.key:v.value for v in s.values};gate_time=time.monotonic()
                gate_blocked=obstacle_blocks(values,s.message,speed)
                result['events'].append({'t_s':gate_time-begin,'gate_reason':s.message,'values':values})
    node.create_subscription(Odometry,'/fixed_map_localization/odometry',lambda m:pose_cb('map',m),10)
    node.create_subscription(Odometry,'/odometry/filtered',lambda m:pose_cb('odom',m),10)
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',loc_cb,10)
    node.create_subscription(DiagnosticArray,'/collision_gate/diagnostics',gate_cb,10)
    bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);bus.bind(('can0',));bus.setblocking(False)
    begin=time.monotonic();mode=None;wheels=None;status_time=wheel_time=0.;zero_frames=0;motion_seen=False
    def step():
        nonlocal mode,wheels,status_time,wheel_time,zero_frames,motion_seen
        rclpy.spin_once(node,timeout_sec=.01)
        for _ in range(256):
            try:raw=bus.recv(16)
            except BlockingIOError:break
            cid,dlc,data=struct.unpack('=IB3x8s',raw)
            if dlc!=8 or cid not in (5,0x507,0x50a):continue
            now=time.monotonic();result['events'].append({'t_s':now-begin,'can_id':hex(cid),'payload':data.hex()})
            if cid==0x507:mode=data[4];status_time=now
            elif cid==0x50a:
                wheels=parse_wheel_feedback(data);wheel_time=now
                if any(abs(v)>wheel_ceiling for v in wheels):raise RuntimeError('Wheel ceiling exceeded')
                motion_seen=motion_seen or sum(wheels)/4>30
                zero_frames=zero_frames+1 if motion_seen and gate_blocked and all(abs(v)<=10 for v in wheels) else 0
            elif data[2]!=0 or data[4]!=0 or data[7]!=0 or data[1] not in (0,raw_speed) or data[3]!=0:
                raise RuntimeError('CAN speed, turn, blade or unlock boundary violated')
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    try:
        deadline=begin+7
        while time.monotonic()<deadline and (accepted<3 or robot() is None or wheels is None):step()
        pos=robot()
        result['preflight_observed_pose']=pos
        result['reviewed_start_pose']=[*start,yaw]
        if pos is None or math.hypot(pos[0]-start[0],pos[1]-start[1])>.15 or abs(math.atan2(math.sin(pos[2]-yaw),math.cos(pos[2]-yaw)))>.1:
            raise RuntimeError('Pose differs from reviewed start')
        if mode!=2 or wheels is None or any(abs(v)>startup_wheel_limit for v in wheels):raise RuntimeError('Require stationary acknowledged API run')
        if accepted<3 or pub.get_subscription_count()<1:raise RuntimeError('Localization or velocity subscriber not ready')
        result['start_pose']=pos;drive_begin=time.monotonic();next_send=0.;blocked_since=None
        while time.monotonic()-drive_begin<duration:
            now=time.monotonic();pos=robot()
            if mode!=2:raise RuntimeError('API exited; no reacquisition')
            if now-status_time>.25 or now-wheel_time>.25 or now-loc_time>2.5:
                result['stale_input_ages_s']={'status':now-status_time,'wheels':now-wheel_time,'localization':now-loc_time}
                raise RuntimeError('Telemetry or localization stale')
            if pos is None or corridor_error(start,goal,pos[:2],.2):raise RuntimeError('Outside reviewed corridor')
            if abs(math.atan2(math.sin(pos[2]-yaw),math.cos(pos[2]-yaw)))>.15:raise RuntimeError('Heading deviates from straight test')
            if math.hypot(pos[0]-result['start_pose'][0],pos[1]-result['start_pose'][1]) >= ceiling:
                raise RuntimeError('Approach exceeds reviewed ceiling without verified obstacle stop')
            result['events'].append({'t_s':now-begin,'robot_in_map':pos})
            if now>=next_send:
                cmd=Twist();cmd.linear.x=speed;pub.publish(cmd);next_send=now+.05
            step()
            now=time.monotonic()
            blocked_since=(blocked_since or now) if gate_blocked and now-gate_time<=.3 else None
            if blocked_since and now-blocked_since>=.75 and zero_frames>=3:
                if not motion_seen or math.hypot(pos[0]-result['start_pose'][0],pos[1]-result['start_pose'][1]) < .1:raise RuntimeError('No real approach observed')
                result.update(completed=True,final_pose=robot(),final_wheel_mm_s=list(wheels),obstacle_block_confirmed=True)
                break
        if not result['completed']:raise RuntimeError('No verified obstacle stop within time limit')
    except (Exception,KeyboardInterrupt) as exc:result['error']=str(exc) or 'interrupted'
    finally:
        end=time.monotonic()+.5
        while time.monotonic()<end:
            pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
        result.update(motion_seen=motion_seen,final_api_mode=mode)
        args.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)
        bus.close();node.destroy_node();rclpy.shutdown()
    return 0 if result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
