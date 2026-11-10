#!/usr/bin/env python3
"""Isolated static-map ComputePathToPose integration; no CAN/drive actions."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import platform
import signal
import subprocess
import time
import yaml


def main():
    if platform.machine() not in ('aarch64','arm64') or os.environ.get('ROS_DOMAIN_ID')!='198':
        raise SystemExit('Run on RK3588 with isolated ROS_DOMAIN_ID=198')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TransformStamped,PoseStamped
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose
    from tf2_ros import StaticTransformBroadcaster
    from ament_index_python.packages import get_package_share_directory,get_package_prefix
    from mower_console.assets import MapAssets
    from mower_console.geometry import split_path
    parser=argparse.ArgumentParser();parser.add_argument('--map-dir',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('mower_console'));assets=MapAssets(args.map_dir)
    params=yaml.safe_load((share/'config/nav2_console.yaml').read_text())
    params={name:params[name] for name in ('planner_server','global_costmap')}
    global_cfg=params['global_costmap']['global_costmap']['ros__parameters']
    global_cfg['plugins']=['static_layer','inflation_layer']
    for name in ('ForwardGrid','ReverseGrid'):
        params['planner_server']['ros__parameters'][name]['lattice_filepath']=str(share/'config/lattice_r4_10cm.json')
    reverse_params=copy.deepcopy(params)
    reverse_params['planner_server']['ros__parameters']['planner_plugins']=['ReverseGrid']
    reverse_params['planner_server']['ros__parameters'].pop('ForwardGrid')
    reverse_params['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic']='/map'
    params['planner_server']['ros__parameters']['planner_plugins']=['ForwardGrid']
    params['planner_server']['ros__parameters'].pop('ReverseGrid')
    reverse_config=args.output.with_suffix('.reverse.params.yaml')
    reverse_config.write_text(yaml.safe_dump({'reverse_planning':reverse_params}))
    config=args.output.with_suffix('.params.yaml');config.write_text(yaml.safe_dump(params))
    rclpy.init();node=rclpy.create_node('console_planning_check')
    publisher=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width,grid.info.height=assets.grid.width,assets.grid.height
    grid.info.resolution=assets.grid.resolution;grid.info.origin.position.x,grid.info.origin.position.y=assets.grid.origin[:2]
    grid.info.origin.orientation.w=1.;grid.data=assets.grid.data;publisher.publish(grid)
    tf=StaticTransformBroadcaster(node);transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_link'
    transform.transform.translation.x=1.5;transform.transform.translation.y=.7;transform.transform.rotation.w=1.;tf.sendTransform(transform)
    def wait(future,seconds=10):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        if not future.done():raise TimeoutError('ROS response timeout')
        return future.result()
    def pose(x,y,yaw):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.position.x=float(x);msg.pose.position.y=float(y);msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
        return msg
    results=[]
    log=args.output.with_suffix('.log').open('w')
    executable=Path(get_package_prefix('nav2_planner'))/'lib/nav2_planner/planner_server'
    process=subprocess.Popen([str(executable),'--ros-args','--params-file',str(config)],
                             stdout=log,stderr=log,start_new_session=True)
    reverse_process=subprocess.Popen([str(executable),'--ros-args','--params-file',str(reverse_config),
                                      '-r','__ns:=/reverse_planning'],stdout=log,stderr=log,start_new_session=True)
    try:
        for namespace in ('','/reverse_planning'):
            change=node.create_client(ChangeState,namespace+'/planner_server/change_state')
            if not change.wait_for_service(timeout_sec=12):raise TimeoutError('planner lifecycle unavailable')
            for transition in (1,3):
                request=ChangeState.Request();request.transition.id=transition
                if not wait(change.call_async(request)).success:raise RuntimeError('planner lifecycle failed')
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        reverse_client=ActionClient(node,ComputePathToPose,'/reverse_planning/compute_path_to_pose')
        if not client.wait_for_server(timeout_sec=3):raise RuntimeError('planner action unavailable')
        for name,target,planner in [('straight',[3.,.7,0.],'ForwardGrid'),('turn',[1.5,1.8,math.pi/2],'ForwardGrid'),('reverse',[1.1,.7,0.],'ReverseGrid')]:
            goal=ComputePathToPose.Goal();goal.start=pose(1.5,.7,0);goal.goal=pose(*target);goal.use_start=True;goal.planner_id=planner
            handle=wait((reverse_client if planner=='ReverseGrid' else client).send_goal_async(goal));response=wait(handle.get_result_async())
            result=dict(case=name,status=response.status,passed=False)
            if response.status==4:
                path=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in response.result.path.poses]
                try:
                    segments=split_path(path);assets.grid.validate(segments)
                    error=math.dist(path[-1][:2],target[:2])
                    result.update(passed=error<=.25 and (name!='reverse' or any(s['kind']=='reverse' for s in segments)),endpoint_error_m=error,segments=segments)
                except ValueError as exc:result['error']=str(exc);result['path']=path
            results.append(result);print(json.dumps({k:v for k,v in result.items() if k not in ('segments','path')}),flush=True)
        results.append(dict(case='unsafe_goal',passed=not assets.grid.clear_disk(1.5,2.2,.98),
                            expected='reject before action'))
        args.output.write_text(json.dumps(dict(type='isolated_static_planner_integration',can_used=False,motion_actions_sent=False,cases=results),indent=2))
    finally:
        for child in (process,reverse_process):
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
