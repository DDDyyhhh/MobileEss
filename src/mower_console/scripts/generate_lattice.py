#!/usr/bin/env python3
"""Derive an inspectable R4/10cm lattice from the installed Humble diff set.

Scale the official 1m set by four; its 5cm endpoint lattice becomes 20cm,
which is an exact sublattice of the 10cm costmap. Resample intermediate
poses to <=5cm; do not merely relabel a 5cm primitive as a 10cm one.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def generate(source):
    raw = source.read_bytes()
    result = json.loads(raw)
    metadata = result['lattice_metadata']
    if metadata['motion_model'] != 'diff' or metadata['grid_resolution'] != .05 or metadata['turning_radius'] != 1:
        raise ValueError('Expected official Humble 5cm/1m diff primitives')
    metadata['turning_radius'], metadata['grid_resolution'] = 4., .10
    result['source_sha256'] = hashlib.sha256(raw).hexdigest()
    result['derivation'] = 'R4 curves from official set; 10cm rational-heading short straights; <=5cm samples'
    for primitive in result['primitives']:
        for key in ('trajectory_radius','trajectory_length','arc_length','straight_length'):
            primitive[key] *= 4
        poses, previous = [], [0.,0.,metadata['heading_angles'][primitive['start_angle_index']]]
        for x,y,yaw in primitive['poses']:
            current = [4*x,4*y,yaw]
            n=max(1,math.ceil(math.dist(previous[:2],current[:2])/.05),
                  math.ceil(abs(math.atan2(math.sin(yaw-previous[2]),math.cos(yaw-previous[2])))/.05))
            delta=math.atan2(math.sin(yaw-previous[2]),math.cos(yaw-previous[2]))
            poses += [[previous[0]+(current[0]-previous[0])*i/n,
                       previous[1]+(current[1]-previous[1])*i/n,
                       (previous[2]+delta*i/n)%(2*math.pi)] for i in range(1,n+1)]
            previous=current
        for value in previous[:2]:
            if abs(value/.1-round(value/.1)) > .001:
                raise ValueError('Primitive endpoint does not lie on the 10cm grid')
        primitive['poses']=poses
    # Scaling the official minimal control set also scales its straight moves
    # to >=1.2m. Add short grid-aligned moves, otherwise sub-metre reverse is
    # impossible and even a straight nearby goal can require large detours.
    directions=((1,0),(2,1),(1,1),(1,2),(0,1),(-1,2),(-1,1),(-2,1),
                (-1,0),(-2,-1),(-1,-1),(-1,-2),(0,-1),(1,-2),(1,-1),(2,-1))
    for index,(dx,dy) in enumerate(directions):
        yaw=metadata['heading_angles'][index]
        distance=math.hypot(dx,dy)*.1
        count=math.ceil(distance/.05)
        result['primitives'].append(dict(trajectory_id=0,start_angle_index=index,end_angle_index=index,
            left_turn=False,trajectory_radius=0.,trajectory_length=distance,arc_length=0.,
            straight_length=distance,poses=[[dx*.1*i/count,dy*.1*i/count,yaw] for i in range(1,count+1)]))
    result['primitives'].sort(key=lambda p:p['start_angle_index'])
    for index,primitive in enumerate(result['primitives']):primitive['trajectory_id']=index
    metadata['number_of_trajectories']=len(result['primitives'])
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,default=Path('/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/1m_turning_radius/diff/output.json'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.write_text(json.dumps(generate(args.source),separators=(',',':'))+'\n')
