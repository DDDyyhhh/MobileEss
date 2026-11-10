"""Bounded Nav2 controller diagnosis on RK3588; bridge absent, CAN receive only."""
import argparse
import json
import math
from pathlib import Path
import platform
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--path-angle-probe', type=float)
    parser.add_argument('--expect-align-first', action='store_true')
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64') or args.output.exists():
        parser.error('Run on RK3588 with a new evidence file')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters, SetParameters
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import ComputePathToPose, FollowPath
    rclpy.init()
    node = rclpy.create_node('controller_no_can_check')
    if 'cmd_vel_can_bridge' in node.get_node_names():
        raise RuntimeError('Stop bridge before no-motion diagnosis')
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind(('can0',)); bus.setblocking(False)
    poses, events = {}, []
    accepted, can_count = 0, 0
    begin = time.monotonic()
    handle = None
    report = {'can_transmission': False, 'trials': []}
    trial = None
    def on_pose(key, msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        poses[key] = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def on_diag(msg):
        nonlocal accepted
        accepted += sum(s.message == 'accepted' for s in msg.status)
    def on_cmd(key, msg):
        events.append({'t_s':time.monotonic()-begin,'trial':trial,'topic':key,'vx':msg.linear.x,'wz':msg.angular.z})
    node.create_subscription(Odometry,'/fixed_map_localization/odometry',lambda m:on_pose('map',m),10)
    node.create_subscription(Odometry,'/odometry/filtered',lambda m:on_pose('odom',m),10)
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',on_diag,10)
    for topic in ('/cmd_vel','/cmd_vel_smoothed','/cmd_vel_gated'):
        node.create_subscription(Twist,topic,lambda m,t=topic:on_cmd(t,m),10)
    def robot():
        tx,ty,a=poses['map']; x,y,b=poses['odom']
        return [tx+math.cos(a)*x-math.sin(a)*y,ty+math.sin(a)*x+math.cos(a)*y,a+b]
    def step():
        nonlocal can_count
        rclpy.spin_once(node,timeout_sec=.02)
        for _ in range(256):
            try: raw=bus.recv(16)
            except BlockingIOError: break
            cid,dlc,data=struct.unpack('=IB3x8s',raw)
            if cid==5:
                can_count+=1
                raise RuntimeError('Unexpected control frame during receive-only diagnosis')
    def await_result(future, seconds):
        deadline=time.monotonic()+seconds
        while not future.done() and time.monotonic()<deadline:step()
        if not future.done():raise RuntimeError('Diagnostic action timed out')
        return future.result()
    get_params=node.create_client(GetParameters,'/controller_server/get_parameters')
    set_params=node.create_client(SetParameters,'/controller_server/set_parameters')
    parameter='FollowPath.PathAngleCritic.max_angle_to_furthest'
    original=None
    try:
        deadline=time.monotonic()+7
        while (accepted<3 or len(poses)<2) and time.monotonic()<deadline:step()
        if accepted<3 or len(poses)<2:raise RuntimeError('Localization not accepted')
        p=robot();report['start_pose']=p
        planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        controller=ActionClient(node,FollowPath,'/follow_path')
        if not planner.wait_for_server(timeout_sec=5) or not controller.wait_for_server(timeout_sec=5):raise RuntimeError('Servers unavailable')
        goal=ComputePathToPose.Goal();goal.goal.header.frame_id='map';goal.goal.header.stamp=node.get_clock().now().to_msg()
        goal.goal.pose.position.x=p[0]+.65*math.cos(p[2])+.45*math.sin(p[2])
        goal.goal.pose.position.y=p[1]+.65*math.sin(p[2])-.45*math.cos(p[2])
        yaw=p[2]-math.radians(35)
        goal.goal.pose.orientation.z=math.sin(yaw/2);goal.goal.pose.orientation.w=math.cos(yaw/2)
        planning=await_result(planner.send_goal_async(goal),5)
        planned=await_result(planning.get_result_async(),8)
        if planned.status!=4 or not planned.result.path.poses:raise RuntimeError('No diagnostic path')
        report['path_xy_m']=[[s.pose.position.x,s.pose.position.y] for s in planned.result.path.poses]
        modes=[('baseline',None)]
        if args.path_angle_probe is not None:
            if not get_params.wait_for_service(timeout_sec=3) or not set_params.wait_for_service(timeout_sec=3):raise RuntimeError('Controller parameter service absent')
            original=await_result(get_params.call_async(GetParameters.Request(names=[parameter])),3).values[0].double_value
            report['original_path_angle_threshold']=original
            modes.append(('path_angle_probe',args.path_angle_probe))
        for label,value in modes:
            if value is not None:
                response=await_result(set_params.call_async(SetParameters.Request(parameters=[Parameter(parameter,value=value).to_parameter_msg()])),3)
                if not all(s.successful for s in response.results):raise RuntimeError('Probe parameter rejected')
            for index in range(2):
                trial=f'{label}-{index+1}'
                follow=FollowPath.Goal();follow.path=planned.result.path;follow.controller_id='FollowPath';follow.goal_checker_id='goal_checker'
                handle=await_result(controller.send_goal_async(follow),5)
                if not handle.accepted:raise RuntimeError('Controller action rejected')
                deadline=time.monotonic()+4
                while time.monotonic()<deadline:step()
                await_result(handle.cancel_goal_async(),3);handle=None
                data=[e for e in events if e['trial']==trial and e['topic']=='/cmd_vel' and e['vx']>.01]
                wz=[e['wz'] for e in data]
                report['trials'].append({'name':trial,'samples':len(wz),'mean_wz':sum(wz)/len(wz) if wz else None,'right_command_fraction':sum(w<-.05 for w in wz)/len(wz) if wz else 0})
                trial=None
                deadline=time.monotonic()+.5
                while time.monotonic()<deadline:step()
        if args.expect_align_first:
            checks=[]
            for item in report['trials']:
                commands=[e for e in events if e['trial']==item['name'] and e['topic']=='/cmd_vel']
                rotating=[e for e in commands if abs(e['wz'])>.05]
                check={'trial':item['name'],'peak_vx_mps':max((abs(e['vx']) for e in commands),default=0),
                       'right_rotation_fraction':sum(e['wz']<-.05 for e in rotating)/len(rotating) if rotating else 0}
                checks.append(check)
            report['alignment_checks']=checks
            if any(c['peak_vx_mps']>.001 or c['right_rotation_fraction']<.9 for c in checks):
                raise RuntimeError('Controller advances before heading alignment or rotates the wrong way')
        report['completed']=True
    except Exception as exc:
        report['error']=str(exc);report['completed']=False
    finally:
        if handle is not None:
            try:await_result(handle.cancel_goal_async(),3)
            except Exception:pass
        if original is not None:
            try:
                response=await_result(set_params.call_async(SetParameters.Request(parameters=[Parameter(parameter,value=original).to_parameter_msg()])),3)
                report['probe_restored']=all(s.successful for s in response.results)
            except Exception:report['probe_restored']=False
        report.update(can_control_frames=can_count,events=events,final_pose=robot() if len(poses)==2 else None)
        args.output.write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ('events','path_xy_m')}),flush=True)
        bus.close();node.destroy_node();rclpy.shutdown()
    return 0 if report.get('completed') else 1


if __name__=='__main__':
    raise SystemExit(main())
