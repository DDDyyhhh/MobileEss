"""Build a separate ground-visibility candidate from a captured ROS bag."""
import argparse
import json
from pathlib import Path
import platform
import sqlite3

import numpy as np
import yaml

from pcd_to_occupancy import read_pcd_xyz, fit_ground_plane
from mower_nav2.visibility_mapping import (
    visibility_endpoints, mark_visible, filter_self_returns, TemporalStaticGrid)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--self-filter-config', type=Path, required=True)
    parser.add_argument('--temporal-static', action='store_true',
                        help='require distinct-scan persistence and later visibility')
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64'):
        parser.error('ROS bag processing runs on RK3588')
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    root = args.session.resolve()
    self_params = yaml.safe_load(args.self_filter_config.read_text())['livox_obstacle_cloud']['ros__parameters']
    boxes = np.asarray(self_params['self_filter_boxes']).reshape(-1,6)
    out = args.output_dir.resolve()
    if out.exists():
        parser.error('output directory already exists; preserve previous candidates')
    dbfiles = sorted((root/'bag').glob('*.db3'))
    if len(dbfiles) != 1:
        parser.error('this capture must contain exactly one SQLite bag database')
    with sqlite3.connect(f'file:{dbfiles[0]}?mode=ro', uri=True) as db:
        topics = dict(db.execute('select name,id from topics'))
        trajectory = []
        for _, data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                                  (topics['/fast_lio/map_capture/odometry'],)):
            msg = deserialize_message(data, Odometry)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            trajectory.append((stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w))
        poses = np.asarray(trajectory)
        origin = np.floor((poses[:, 1:3].min(axis=0) - 3.0)*10)/10
        size = np.ceil((poses[:, 1:3].max(axis=0)+3.0-origin)/0.1).astype(int)
        grid = np.full((size[1], size[0]), 205, np.uint8)
        points = read_pcd_xyz(root/'map.pcd')
        plane = fit_ground_plane(points)
        counts = np.zeros_like(grid, dtype=np.int32)
        history = TemporalStaticGrid(grid.shape) if args.temporal_static else None
        scans = 0
        for index, (_, data) in enumerate(db.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp',
                (topics['/cloud_registered'],))):
            if index % 5:
                continue
            msg = deserialize_message(data, PointCloud2)
            if msg.header.frame_id != 'map_capture_odom':
                raise ValueError('registered cloud must be in map_capture_odom')
            if msg.is_bigendian or msg.height != 1:
                raise ValueError('expected little-endian unorganized registered cloud')
            offsets = {f.name:f.offset for f in msg.fields if f.datatype == 7}
            dtype = np.dtype({'names':['x','y','z'], 'formats':['<f4']*3,
                              'offsets':[offsets[n] for n in ('x','y','z')],
                              'itemsize':msg.point_step})
            raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width)
            xyz = np.column_stack([raw[n] for n in ('x','y','z')])
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            i = np.searchsorted(poses[:,0], stamp)
            candidates = [j for j in (i-1,i) if 0 <= j < len(poses)]
            j = min(candidates, key=lambda j:abs(poses[j,0]-stamp))
            if abs(poses[j,0]-stamp) > 0.15:
                continue
            sensor = poses[j,1:3]
            x,y,z,w=poses[j,4:8]
            rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            xyz=filter_self_returns(xyz,poses[j,1:4],rotation,boxes)
            heights=xyz@plane[:3]+plane[3]
            obstacles=xyz[(heights>=0.15)&(heights<=0.80),:2]
            cells=np.floor((obstacles-origin)/0.1).astype(int)
            valid=(cells[:,0]>=0)&(cells[:,0]<size[0])&(cells[:,1]>=0)&(cells[:,1]<size[1])
            np.add.at(counts,(cells[valid,1],cells[valid,0]),1)
            ends = visibility_endpoints(xyz, sensor, plane)
            if history is None:
                mark_visible(grid, sensor, ends, origin, 0.1)
            else:
                hits = np.zeros(grid.shape, dtype=bool)
                hits[cells[valid,1],cells[valid,0]] = True
                frame = np.full(grid.shape, 205, dtype=np.uint8)
                mark_visible(frame, sensor, ends, origin, 0.1)
                history.observe(hits, frame == 254)
            scans += 1
        if history is None:
            grid[counts >= 5] = 0
        else:
            grid = history.result()
    out.mkdir(parents=True)
    image = grid[::-1]
    with (out/'map.pgm').open('wb') as stream:
        stream.write(f'P5\n{size[0]} {size[1]}\n255\n'.encode()+image.tobytes())
    (out/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':0.1,
        'origin':[float(origin[0]),float(origin[1]),0.0],'negate':0,
        'occupied_thresh':0.65,'free_thresh':0.25,'mode':'trinary'}, sort_keys=False))
    (out/'map.pcd').symlink_to(root/'map.pcd')
    for name in ('livox.json','mid360_real.yaml','metadata.json','acceptance.md'):
        (out/name).write_bytes((root/name).read_bytes())
    result={'scans_used':scans,'scan_stride':5,'max_visibility_range_m':20,
            'ground_plane':plane.tolist(),'self_filter_boxes':boxes.tolist(),
            'map_reviewed':False, 'temporal_static':args.temporal_static}
    if args.temporal_static:
        result.update(min_occupied_scans=3, min_later_ground_visible_scans=5,
                      same_scan_occupied_wins=True, motion_approved=False)
    (out/'conversion.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
