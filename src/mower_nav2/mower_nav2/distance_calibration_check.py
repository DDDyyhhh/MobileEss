"""One reviewed bounded straight or turning measurement; no parameter adjustment."""
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


def integrate_wheel_distance(samples, begin, end, max_gap=.15):
    """Signed mean-wheel distance, using host CAN receipt times, not source stamps."""
    if not all(math.isfinite(v) for v in (begin,end,max_gap)) or end<=begin or max_gap<=0:
        raise ValueError('Invalid integration interval')
    if len(samples)<2 or any(len(s)!=5 or not all(math.isfinite(v) for v in s) for s in samples):
        raise ValueError('Invalid wheel samples')
    if samples[0][0]>begin or samples[-1][0]<end:
        raise ValueError('Wheel samples do not bracket the interval')
    total=0.
    for a,b in zip(samples,samples[1:]):
        dt=b[0]-a[0]
        if dt<=0:raise ValueError('Wheel timestamps not strictly increasing')
        lo,hi=max(a[0],begin),min(b[0],end)
        if hi<=lo:continue
        if dt>max_gap:raise ValueError('Wheel feedback gap too large')
        va,vb=sum(a[1:])/4000.,sum(b[1:])/4000.
        vlo=va+(vb-va)*(lo-a[0])/dt
        vhi=va+(vb-va)*(hi-a[0])/dt
        total+=(vlo+vhi)*.5*(hi-lo)
    return total


def fresh_gate(values):
    try:
        return (0<=float(values['command_age_sec'])<=.25 and
            all(0<=float(values[k])<=.3 for k in ('costmap_age_sec','odom_age_sec','obstacle_age_sec')))
    except (KeyError,ValueError,TypeError):return False


def measurement_profile(proposal):
    """Only the historical low-speed and reviewed 0.4 short profiles are allowed."""
    speed = proposal.get('speed_command_m_s', .1)
    duration = proposal.get('drive_duration_s')
    ceiling = proposal.get('max_displacement_m')
    if proposal.get('stop_observation_limit_s') != 1.5 or proposal.get('corridor_half_width_m') != .2:
        raise ValueError('Invalid measurement safety limits')
    if (speed, duration, ceiling) == (.1, 5.0, .90):
        return speed, duration, ceiling, 1.0, 250, 1
    if (speed, duration, ceiling) == (.4, 2.0, 1.2):
        return speed, duration, ceiling, 2.0, 550, 4
    raise ValueError('Unsupported bounded measurement profile')


def turn_profile(proposal):
    if (proposal.get('speed_command_m_s') != 0.0
            or proposal.get('angular_command_rad_s') not in (-.2, .2)
            or proposal.get('drive_duration_s') != 3.0
            or proposal.get('max_displacement_m') != .2
            or proposal.get('max_turn_angle_rad') != math.pi/4
            or proposal.get('stop_observation_limit_s') != 1.5
            or proposal.get('track_width_m') != .640):
        raise ValueError('Unsupported bounded turn measurement profile')
    return 0.0, 3.0, .2, 1.0, 250, 0


def signed_yaw_change(start, current):
    if not all(math.isfinite(x) for x in (start, current)):
        raise ValueError('Nonfinite yaw')
    return math.atan2(math.sin(current-start), math.cos(current-start))


def turn_guard(start, current, direction=1):
    if direction not in (-1, 1):
        return 'Invalid turn direction'
    if len(start) != 3 or len(current) != 3 or not all(math.isfinite(v) for v in (*start, *current)):
        return 'Invalid turn pose'
    if math.dist(start[:2], current[:2]) >= .2:
        return 'Turn translation exceeds 0.20 m'
    delta = direction * signed_yaw_change(start[2], current[2])
    if delta < -.05:
        return 'Unexpected right turn' if direction == 1 else 'Unexpected left turn'
    if delta >= math.pi/4:
        return 'Turn angle exceeds 45 degrees'
    return None


def integrate_wheel_yaw(samples, begin, end, track_width=.640):
    if not math.isfinite(track_width) or track_width <= 0:
        raise ValueError('Invalid track width')
    differential = [[t, -fl, fr, -rl, rr] for t, fl, fr, rl, rr in samples]
    return 2 * integrate_wheel_distance(differential, begin, end) / track_width


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():
        parser.error('Run on RK3588 with new evidence')
    p=json.loads(args.proposal.read_text())
    if not p.get('motion_approved') or p.get('test_kind') not in ('straight_distance_calibration', 'turn_angle_calibration'):
        parser.error('Require reviewed measurement approval')
    turning = p['test_kind'] == 'turn_angle_calibration'
    start=p['pose']['xy_m']; yaw=math.radians(p['pose']['yaw_deg'])
    try:
        speed, duration, ceiling, reference, wheel_ceiling, raw_speed = (turn_profile(p) if turning else measurement_profile(p))
    except ValueError as exc:
        parser.error(str(exc))
    goal=[start[0]+reference*math.cos(yaw),start[1]+reference*math.sin(yaw)]
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import Imu
    from mcu_can_bridge.protocol import parse_wheel_feedback
    rclpy.init(); node=rclpy.create_node('ground_distance_calibration_check')
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    poses={}; loc_time=0.; accepted=0; gate_time=0.; gate_values={}; gate_reason=""
    angular_speed = p['angular_command_rad_s'] if turning else 0.0
    turn_direction = 1 if angular_speed >= 0 else -1
    raw_angular = (2 * turn_direction) & 0xff if turning else 0
    result={'speed_command_m_s':speed,'angular_command_rad_s':angular_speed,'command_publish_duration_s':duration,'physical_distance_pending':not turning,'physical_angle_pending':turning,'parameter_adjustment':False,'wheel_integral_uses_receipt_timestamps':True,'stop_observation_limit_s':1.5,'automatic_resume':False,'test_kind':p['test_kind'],'nav2_goal_sent':False,'automatic_unlock':False,'completed':False,'events':[]}
    def pose_cb(key,msg):
        v=msg.pose.pose;p=v.position;q=v.orientation
        pose_times[key]=time.monotonic()
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
        nonlocal gate_time,gate_values,gate_reason
        for s in msg.status:
            if s.name=='mower_nav2/collision_gate':
                values={v.key:v.value for v in s.values};gate_time=time.monotonic()
                gate_values=values;gate_reason=s.message
                result['events'].append({'t_s':gate_time-begin,'gate_reason':s.message,'values':values})
    last_imu_record = 0.0
    def imu_cb(msg):
        nonlocal last_imu_record
        now = time.monotonic()
        if now-last_imu_record < .02:
            return
        last_imu_record = now
        result['events'].append({'t_s':now-begin, 'imu_frame':msg.header.frame_id,
            'imu_stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            'imu_angular_velocity_rad_s':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]})
    node.create_subscription(Odometry,'/fixed_map_localization/odometry',lambda m:pose_cb('map',m),10)
    node.create_subscription(Odometry,'/odometry/filtered',lambda m:pose_cb('odom',m),10)
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',loc_cb,10)
    node.create_subscription(DiagnosticArray,'/collision_gate/diagnostics',gate_cb,10)
    node.create_subscription(Imu,'/livox/imu',imu_cb,10)
    bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);bus.bind(('can0',));bus.setblocking(False)
    begin=time.monotonic();mode=None;wheels=None;status_time=wheel_time=0.;zero_frames=0;motion_seen=False; stop_zero_can_frames=0; stop_begin=None; wheel_samples=[];pose_times={}
    def step():
        nonlocal mode,wheels,status_time,wheel_time,zero_frames,motion_seen,stop_zero_can_frames
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
                motion_seen=motion_seen or (any(abs(v)>15 for v in wheels) if turning else sum(wheels)/4>30)
                wheel_samples.append([now-begin,*wheels])
                zero_frames=zero_frames+1 if motion_seen and stop_begin is not None and all(abs(v)<=10 for v in wheels) else 0
            elif data[2]!=0 or data[4]!=0 or data[7]!=0 or data[1] not in (0,raw_speed) or data[3] not in (0,raw_angular):
                raise RuntimeError('CAN speed, turn, blade or unlock boundary violated')
            elif stop_begin is not None:
                stop_zero_can_frames=stop_zero_can_frames+1 if data[1]==0 and data[3]==0 else 0
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    try:
        deadline=begin+7
        while time.monotonic()<deadline and (accepted<3 or robot() is None or wheels is None or pub.get_subscription_count()<1):step()
        pos=robot()
        result['preflight_observed_pose'] = pos
        result['reviewed_start_pose'] = [*start, yaw]
        if pos is None or math.hypot(pos[0]-start[0],pos[1]-start[1])>.15 or abs(math.atan2(math.sin(pos[2]-yaw),math.cos(pos[2]-yaw)))>.1:
            raise RuntimeError('Pose differs from reviewed start')
        if mode!=2 or wheels is None or any(abs(v)>10 for v in wheels):raise RuntimeError('Require stationary acknowledged API run')
        if accepted<3:raise RuntimeError('Localization not accepted')
        if pub.get_subscription_count()<1:raise RuntimeError('Velocity subscriber not connected')
        result['start_pose']=pos;result['start_filtered_odom_pose']=list(poses['odom'])
        result['start_map_to_odom']=list(poses['map'])
        drive_begin=time.monotonic();next_send=0.;measurement_begin=drive_begin-begin
        result['measurement_begin_t_s']=measurement_begin
        while time.monotonic()-drive_begin<duration+3:
            now=time.monotonic();pos=robot()
            if mode!=2:raise RuntimeError('API exited; no reacquisition')
            if now-status_time>.25 or now-wheel_time>.25 or now-loc_time>2.5 or now-pose_times.get('odom',0)>.3:
                result['stale_input_ages_s'] = {
                    'status': now-status_time, 'wheels': now-wheel_time,
                    'localization': now-loc_time,
                    'odom': now-pose_times.get('odom',0),
                }
                result['stale_detection_t_s'] = now-begin
                raise RuntimeError('Telemetry or localization stale')
            if pos is None or (not turning and corridor_error(start,goal,pos[:2],.2)):
                raise RuntimeError('Outside reviewed corridor')
            if turning:
                reason = turn_guard(result['start_pose'], pos, turn_direction)
                if reason:
                    result['guard_rejected_pose'] = pos
                    result['guard_rejected_filtered_odom'] = list(poses['odom'])
                    result['guard_rejected_map_to_odom'] = list(poses['map'])
                    raise RuntimeError(reason)
            elif abs(math.atan2(math.sin(pos[2]-yaw),math.cos(pos[2]-yaw)))>.15:
                raise RuntimeError('Heading deviates from straight test')
            displacement=math.hypot(pos[0]-result['start_pose'][0],pos[1]-result['start_pose'][1])
            if displacement>=ceiling:raise RuntimeError('Exceeded reviewed measurement ceiling')
            result['events'].append({'t_s':now-begin,'robot_in_map':pos,
                'filtered_odom':list(poses['odom']),'map_to_odom':list(poses['map'])})
            if now-drive_begin>.35:
                if now-gate_time>.3 or not fresh_gate(gate_values):
                    raise RuntimeError('Gate or its inputs stale; invalid measurement')
                if stop_begin is None and gate_reason!='collision-free prediction':
                    raise RuntimeError('Gate blocked forward motion; invalid measurement')
            if now-drive_begin>=duration and stop_begin is None:
                stop_begin=now;result['zero_command_begin_t_s']=now-begin
                actual = turn_direction * signed_yaw_change(result['start_pose'][2], pos[2]) if turning else displacement
                if not motion_seen or actual < (.05 if turning else .10):raise RuntimeError('No actual reviewed movement')
                next_send=0.
            if now>=next_send:
                cmd=Twist();cmd.linear.x=speed if stop_begin is None else 0.
                cmd.angular.z=angular_speed if stop_begin is None else 0.
                pub.publish(cmd);next_send=now+.05
            step()
            if stop_begin is not None:
                now=time.monotonic()
                if zero_frames>=3 and stop_zero_can_frames>=3:
                    result.update(completed=True,final_pose=robot(),final_wheel_mm_s=list(wheels),
                        final_filtered_odom_pose=list(poses['odom']),final_map_to_odom=list(poses['map']),
                        observed_stop_after_zero_command_s=now-stop_begin,
                        measurement_end_t_s=wheel_time-begin,
                        estimated_map_displacement_m=math.dist(result['start_pose'][:2],robot()[:2]),
                        filtered_odom_displacement_m=math.dist(result['start_filtered_odom_pose'][:2],poses['odom'][:2]))
                    try:
                        result['wheel_integrated_distance_m']=integrate_wheel_distance(wheel_samples,measurement_begin,wheel_time-begin)
                        if turning:
                            result['map_yaw_change_rad'] = signed_yaw_change(result['start_pose'][2], robot()[2])
                            result['filtered_odom_yaw_change_rad'] = signed_yaw_change(result['start_filtered_odom_pose'][2], poses['odom'][2])
                            result['wheel_integrated_yaw_rad'] = integrate_wheel_yaw(wheel_samples,measurement_begin,wheel_time-begin,p['track_width_m'])
                    except ValueError as exc:result['wheel_integration_error']=str(exc)
                    break
                if now-stop_begin>1.5:raise RuntimeError('Stop not confirmed within 1.5 s')
        if not result['completed']:raise RuntimeError('No completed distance measurement')

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
