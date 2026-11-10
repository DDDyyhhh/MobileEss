#!/usr/bin/env python3
"""RK3588 isolated-domain coordinate replay; NO CAN, drivers or motion nodes.

Transforms recorded scans into five alternative odom frames x four headings.
This validates global-search coordinates and acceptance, not five physical
sensor viewpoints, rear visibility, or hardware navigation.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import platform
import signal
import sqlite3
import subprocess
import time

import numpy as np


def main():
    if platform.machine() not in ('aarch64','arm64'):
        raise SystemExit('Run ROS replay on RK3588')
    if os.environ.get('ROS_DOMAIN_ID') != '197':
        raise SystemExit('Set ROS_DOMAIN_ID=197; replay must not join the vehicle domain')
    parser=argparse.ArgumentParser()
    parser.add_argument('--map-dir',type=Path,required=True)
    parser.add_argument('--bag-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-cases',type=int,default=20)
    parser.add_argument('--start-case',type=int,default=0)
    parser.add_argument('--fault',choices=['no_stationary','stale_source','moving','timeout','floor_only','ambiguous'])
    parser.add_argument('--seed-after',type=float)
    args=parser.parse_args()
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2,PointField
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid,Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Bool
    from ament_index_python.packages import get_package_share_directory
    from mower_console.assets import MapAssets
    assets=MapAssets(args.map_dir)
    database=next(args.bag_dir.glob('*.db3'))
    with sqlite3.connect('file:'+str(database)+'?mode=ro',uri=True) as db:
        topic=db.execute("select id from topics where name='/cloud_registered'").fetchone()[0]
        scans=[deserialize_message(row[0],PointCloud2) for row in db.execute(
            'select data from messages where topic_id=? order by timestamp desc limit 3',(topic,))]
    map_pcd=args.map_dir/'map.pcd'
    if args.fault=='ambiguous':
        structure=np.random.default_rng(42).uniform([-2,-2,.1],[2,2,2.0],(400,3)).astype('<f4')
        target=np.vstack([structure+[1.5,.7,0],structure+[4.13,-7.28,0]]).astype('<f4')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        map_pcd=args.output.with_suffix('.pcd')
        header=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(target)}\nHEIGHT 1\nPOINTS {len(target)}\nDATA binary\n'
        map_pcd.write_bytes(header.encode()+target.tobytes())
        msg=PointCloud2();msg.height=1;msg.width=len(structure)
        msg.fields=[PointField(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
        msg.point_step=12;msg.row_step=len(structure)*12;msg.data=(structure+[1.5,.7,0]).astype('<f4').tobytes()
        scans=[msg]
    rclpy.init()
    node=rclpy.create_node('console_coordinate_replay')
    cloud_pub=node.create_publisher(PointCloud2,'/console_replay/cloud',qos_profile_sensor_data)
    odom_pub=node.create_publisher(Odometry,'/odometry/filtered',10)
    stop_pub=node.create_publisher(Bool,'/mower_console/stationary',1)
    hint_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    grid_pub=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=assets.grid.resolution
    grid.info.width,grid.info.height=assets.grid.width,assets.grid.height
    grid.info.origin.position.x,grid.info.origin.position.y=assets.grid.origin[:2]
    grid.info.origin.orientation.z=math.sin(assets.grid.origin[2]/2);grid.info.origin.orientation.w=math.cos(assets.grid.origin[2]/2)
    grid.data=assets.grid.data;grid_pub.publish(grid)
    odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.
    poses,events,generations=[],[],[]
    node.create_subscription(Odometry,'/fixed_map_localization/odometry',lambda m:poses.append(m),10)
    def diagnostics(msg):
        for status in msg.status:
            events.append(status.message)
            generations.extend(int(v.value) for v in status.values if v.key=='generation')
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',diagnostics,10)
    candidates=[[1.5,.7],[4.13,-7.28]]
    free=[]
    for y in np.arange(assets.grid.origin[1]+1,assets.grid.origin[1]+assets.grid.height*assets.grid.resolution-1,1.):
        for x in np.arange(assets.grid.origin[0]+1,assets.grid.origin[0]+assets.grid.width*assets.grid.resolution-1,1.):
            if assets.grid.clear_disk(x,y,.98):free.append([float(x),float(y)])
    candidates=[p for p in candidates if assets.grid.clear_disk(*p,.98)]
    while len(candidates)<5:
        point=max(free,key=lambda p:min(math.dist(p,q) for q in candidates) if candidates else 1.)
        candidates.append(point);free.remove(point)
    share=Path(get_package_share_directory('mower_console'))
    executable=Path(get_package_share_directory('fast_lio')).parents[1]/'lib/fast_lio/fixed_map_localizer'
    results=[]
    try:
        for index,(xy,degrees) in enumerate(( (p,a) for p in candidates for a in (0,90,180,270) )):
            if args.fault and index>0:break
            if index>=args.max_cases:break
            if index<args.start_case:continue
            poses.clear();events.clear();generations.clear();yaw=math.radians(degrees)
            rotation=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
            transformed=[]
            for original in scans:
                msg=copy.deepcopy(original);buffer=bytearray(msg.data);fields={f.name:f.offset for f in msg.fields}
                arrays=[np.ndarray((msg.width,),dtype='<f4',buffer=buffer,offset=fields[n],strides=(msg.point_step,)) for n in ('x','y','z')]
                xyz=(np.column_stack(arrays)-[xy[0],xy[1],0])@rotation
                for i,array in enumerate(arrays):array[:]=xyz[:,i]
                if args.fault=='floor_only':arrays[2][:]=-1.
                msg.data=bytes(buffer);msg.header.frame_id='odom';transformed.append(msg)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            log=args.output.parent/f'{args.output.stem}-case-{index:02d}.log'
            with log.open('w') as stream:
                process=subprocess.Popen([str(executable),'--ros-args','--params-file',str(share/'config/localizer_console.yaml'),
                    '-p','map_file_path:='+str(map_pcd),'-p','source_topic:=/console_replay/cloud',
                    '-p','publish_tf:=false']+
                    (['-p','startup_search_timeout_s:=1.0'] if args.fault=='timeout' else []),stdout=stream,stderr=stream)
                begin=time.monotonic();sample=0;hint_sent=False
                try:
                    while not poses and time.monotonic()-begin<(35 if args.fault=='ambiguous' else 7 if args.fault else 128) and process.poll() is None:
                        stamp=node.get_clock().now().to_msg();msg=transformed[sample%len(transformed)]
                        msg.header.stamp=stamp;odom.header.stamp=stamp
                        if args.fault=='stale_source':msg.header.stamp=(node.get_clock().now()-rclpy.duration.Duration(seconds=5.)).to_msg()
                        odom.pose.pose.position.x=.2*math.sin((time.monotonic()-begin)*3) if args.fault=='moving' else 0.
                        cloud_pub.publish(msg);odom_pub.publish(odom);stop_pub.publish(Bool(data=args.fault!='no_stationary'));sample+=1
                        if args.seed_after is not None and not hint_sent and time.monotonic()-begin>=args.seed_after:
                            hint=PoseWithCovarianceStamped();hint.header.frame_id='map';hint.header.stamp=stamp
                            hint.pose.pose.position.x=float(xy[0]);hint.pose.pose.position.y=float(xy[1])
                            hint.pose.pose.orientation.z=math.sin(yaw/2);hint.pose.pose.orientation.w=math.cos(yaw/2)
                            hint_pub.publish(hint);hint_sent=True
                        until=time.monotonic()+.08
                        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
                        if 'startup_timeout' in events:break
                        if args.fault=='ambiguous' and any('ambiguous' in e for e in events):break
                    result=dict(case=index,expected=[*xy,degrees],accepted=bool(poses),elapsed_s=time.monotonic()-begin,events=sorted(set(events)),generation_max=max(generations,default=0),manual_hint_sent=hint_sent)
                    result['passed']=False
                    if poses:
                        p=poses[-1].pose.pose;actual_yaw=2*math.atan2(p.orientation.z,p.orientation.w)
                        result['position_error_m']=math.hypot(p.position.x-xy[0],p.position.y-xy[1])
                        result['yaw_error_deg']=math.degrees(abs(math.atan2(math.sin(actual_yaw-yaw),math.cos(actual_yaw-yaw))))
                        result['passed']=result['position_error_m']<.2 and result['yaw_error_deg']<5 and result['elapsed_s']<=120
                    if args.fault:
                        expected={'no_stationary':'startup_waiting_stationary','stale_source':'stale_source_cloud',
                                  'moving':'startup_vehicle_moved','timeout':'startup_timeout',
                                  'floor_only':'startup_insufficient_structure','ambiguous':'startup_ambiguous'}[args.fault]
                        result.update(fault=args.fault,passed=not poses and (expected in events or (args.fault=='ambiguous' and any('ambiguous' in e for e in events))))
                    if args.seed_after is not None:
                        result['passed']=result['passed'] and hint_sent and result['generation_max']>0
                    results.append(result)
                    args.output.write_text(json.dumps(dict(type='transformed_recorded_scan_coordinate_replay',physical_viewpoint_validation=False,can_used=False,cases=results),indent=2))
                    print(json.dumps(result),flush=True)
                finally:
                    process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=10)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
            until=time.monotonic()+.3
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if results and all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
