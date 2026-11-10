"""One reviewed obstacle-cloud-loss stop; keep commands and CAN heartbeats."""
import argparse
import json
import math
import os
from pathlib import Path
import platform
import signal
import socket
import struct
import time
from mower_nav2.ground_check import corridor_error
from mower_nav2.obstacle_cloud_fault import open_owned_obstacle_process, terminate_obstacle_cloud


def obstacle_cloud_loss_blocks(values, message):
    """Identify obstacle-cloud loss with the other inputs still fresh."""
    try:
        return (message == 'stale command, costmap, odometry, or obstacle cloud' and
            .30 < float(values['obstacle_age_sec']) < 5.0 and
            0 <= float(values['command_age_sec']) <= .25 and
            .09 <= float(values['input_linear_x']) <= .101 and
            abs(float(values['input_angular_z'])) <= .001 and
            abs(float(values['output_linear_x'])) <= .001 and
            abs(float(values['output_angular_z'])) <= .001 and
            all(0 <= float(values[k]) <= .3 for k in
                ('costmap_age_sec', 'odom_age_sec', 'command_age_sec')))
    except (KeyError, ValueError, TypeError):
        return False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--workflow-root-pid',type=int,required=True)
    args=parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():
        parser.error('Run on RK3588 with new evidence')
    p=json.loads(args.proposal.read_text())
    if not p.get('motion_approved') or p.get('test_kind')!='straight_obstacle_cloud_loss_stop':
        parser.error('Require reviewed obstacle-cloud-loss approval')
    start=p['pose']['xy_m']; yaw=math.radians(p['pose']['yaw_deg'])
    if (p.get('max_displacement_m') != .45 or p.get('drive_duration_s') != 1.0
            or p.get('stop_observation_limit_s') != 1.5
            or p.get('corridor_half_width_m') != .2):
        parser.error('Require the exact bounded obstacle-cloud-loss trial parameters')
    goal=[start[0]+.65*math.cos(yaw),start[1]+.65*math.sin(yaw)]
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from mcu_can_bridge.protocol import parse_wheel_feedback
    rclpy.init(); node=rclpy.create_node('ground_obstacle_cloud_loss_check')
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    poses={}; loc_time=0.; accepted=0; gate_time=0.; gate_blocked=False
    result={'drive_before_cloud_loss_s':1.0,'stop_observation_limit_s':1.5,'automatic_resume':False,'test_kind':'straight_obstacle_cloud_loss_stop','nav2_goal_sent':False,'automatic_unlock':False,'completed':False,'events':[]}
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
                gate_blocked=obstacle_cloud_loss_blocks(values,s.message)
                result['events'].append({'t_s':gate_time-begin,'gate_reason':s.message,'values':values})
    node.create_subscription(Odometry,'/fixed_map_localization/odometry',lambda m:pose_cb('map',m),10)
    node.create_subscription(Odometry,'/odometry/filtered',lambda m:pose_cb('odom',m),10)
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',loc_cb,10)
    node.create_subscription(DiagnosticArray,'/collision_gate/diagnostics',gate_cb,10)
    bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);bus.bind(('can0',));bus.setblocking(False)
    begin=time.monotonic();mode=None;wheels=None;status_time=wheel_time=0.;zero_frames=0;motion_seen=False; loss_zero_can_frames=0
    def step():
        nonlocal mode,wheels,status_time,wheel_time,zero_frames,motion_seen,loss_zero_can_frames
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
                if any(abs(v)>250 for v in wheels):raise RuntimeError('Wheel ceiling exceeded')
                motion_seen=motion_seen or sum(wheels)/4>30
                zero_frames=zero_frames+1 if motion_seen and gate_blocked and all(abs(v)<=10 for v in wheels) else 0
            elif data[2]!=0 or data[4]!=0 or data[7]!=0 or data[1] not in (0,1) or data[3]!=0:
                raise RuntimeError('CAN speed, turn, blade or unlock boundary violated')
            elif gate_blocked:
                loss_zero_can_frames=loss_zero_can_frames+1 if data[1]==0 else 0
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    cloud_fd=None
    try:
        cloud_pid,cloud_fd=open_owned_obstacle_process(args.workflow_root_pid)
        result['owned_obstacle_cloud_pid']=cloud_pid
        deadline=begin+7
        while time.monotonic()<deadline and (accepted<3 or robot() is None or wheels is None or pub.get_subscription_count()<1):step()
        pos=robot()
        if pos is None or math.hypot(pos[0]-start[0],pos[1]-start[1])>.15 or abs(math.atan2(math.sin(pos[2]-yaw),math.cos(pos[2]-yaw)))>.1:
            raise RuntimeError('Pose differs from reviewed start')
        if mode!=2 or wheels is None or any(abs(v)>10 for v in wheels):raise RuntimeError('Require stationary acknowledged API run')
        if accepted<3:raise RuntimeError('Localization not accepted')
        if pub.get_subscription_count()<1:raise RuntimeError('Velocity subscriber not connected')
        result['start_pose']=pos
        drive_begin=time.monotonic();next_send=0.;pause_begin=None
        while time.monotonic()-drive_begin<4.5:
            now=time.monotonic();pos=robot()
            if mode!=2:raise RuntimeError('API exited; no reacquisition')
            if now-status_time>.25 or now-wheel_time>.25 or now-loc_time>2.5:
                raise RuntimeError('Telemetry or localization stale')
            if pos is None or corridor_error(start,goal,pos[:2],.2):
                raise RuntimeError('Outside reviewed corridor')
            if abs(math.atan2(math.sin(pos[2]-yaw),math.cos(pos[2]-yaw)))>.15:
                raise RuntimeError('Heading deviates from straight test')
            displacement=math.hypot(pos[0]-result['start_pose'][0],pos[1]-result['start_pose'][1])
            if displacement>=.45:raise RuntimeError('Exceeded 0.45 m test displacement')
            result['events'].append({'t_s':now-begin,'robot_in_map':pos})
            if now>=next_send:
                cmd=Twist();cmd.linear.x=.1;pub.publish(cmd);next_send=now+.05
            if now-drive_begin>=1.0 and pause_begin is None:
                if not motion_seen or displacement<.03:
                    raise RuntimeError('No actual forward motion before cloud loss')
                terminate_obstacle_cloud(cloud_fd)
                pause_begin=time.monotonic()
                result['cloud_termination_t_s']=pause_begin-begin
            # Keep requesting forward motion; the normal gate must stop it
            # because obstacle data expires, not because the command expires.
            step()
            now=time.monotonic()
            if pause_begin is not None:
                if gate_blocked and now-gate_time<=.3 and zero_frames>=3 and loss_zero_can_frames>=3:
                    result.update(completed=True, final_pose=robot(),
                        final_wheel_mm_s=list(wheels), obstacle_cloud_loss_confirmed=True,
                        observed_stop_after_cloud_termination_s=now-pause_begin,
                        zero_can_frames_after_cloud_loss=loss_zero_can_frames)
                    break
                if now-pause_begin>1.5:
                    raise RuntimeError('Obstacle-cloud-loss stop not verified within 1.5 s')
        if not result['completed']:raise RuntimeError('No verified obstacle-cloud-loss stop')

    except (Exception,KeyboardInterrupt) as exc:result['error']=str(exc) or 'interrupted'
    finally:
        end=time.monotonic()+.5
        while time.monotonic()<end:
            pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
        if cloud_fd is not None:os.close(cloud_fd)
        result.update(motion_seen=motion_seen,final_api_mode=mode)
        args.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)
        bus.close();node.destroy_node();rclpy.shutdown()
    return 0 if result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
