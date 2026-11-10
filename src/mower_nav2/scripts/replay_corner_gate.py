"""Replay a frozen corner through CollisionGate._collision without a node/socket."""
import argparse
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('capture',type=Path)
    parser.add_argument('--policy',choices=('moving_turn','rotate_then_forward'),default='moving_turn')
    args=parser.parse_args();snapshot=json.loads(args.capture.read_text())
    if not snapshot.get('completed'):raise SystemExit('Capture incomplete')
    from mower_nav2.collision_gate import CollisionGate
    from geometry_msgs.msg import Twist
    cm=snapshot['costmap'];p=snapshot['parameters'];x,y,yaw=snapshot['odom_pose']
    costmap=NS(metadata=NS(resolution=cm['resolution'],size_x=cm['size_x'],size_y=cm['size_y'],
                          origin=NS(position=NS(x=cm['origin'][0],y=cm['origin'][1]))),data=cm['data'])
    transform=snapshot['map_from_costmap'];qx,qy,qz,qw=transform['quaternion'];tx,ty,_=transform['translation']
    map_yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));target_odom_yaw=-math.pi/2-map_yaw
    def probe(vx,wz,heading):
        odom=NS(pose=NS(pose=NS(position=NS(x=x,y=y),orientation=NS(x=0.,y=0.,z=math.sin(heading/2),w=math.cos(heading/2)))))
        gate=NS(_costmap=costmap,_odom=odom,_radius=p['robot_radius'],_horizon=p['prediction_horizon'],
                _dt=p['prediction_dt'],_collision_cost=p['collision_cost'],_last_reason='')
        hit=[]
        def cell_cost(a,b):
            cost=CollisionGate._cell_cost(gate,a,b)
            if not hit and (cost is None or cost>=p['collision_cost']):
                hit.append(dict(odom_xy=[a,b],map_xy=[tx+math.cos(map_yaw)*a-math.sin(map_yaw)*b,
                     ty+math.sin(map_yaw)*a+math.cos(map_yaw)*b],cost=cost))
            return cost
        gate._cell_cost=cell_cost;cmd=Twist();cmd.linear.x=vx;cmd.angular.z=wz
        blocked=CollisionGate._collision(gate,cmd)
        return dict(vx=vx,wz=wz,heading=heading,blocked=blocked,reason=gate._last_reason,first_hit=hit)
    moving=probe(.114,-.2,yaw);rotate=probe(0.,-.2,yaw);straight=probe(.114,0.,target_odom_yaw)
    report=dict(policy=args.policy,actual_gate_method=True,velocity_published=False,
                moving_turn=moving,pure_rotation=rotate,forward_after_align=straight)
    clear=not moving['blocked'] if args.policy=='moving_turn' else not rotate['blocked'] and not straight['blocked']
    report['policy_clear']=clear;print(json.dumps(report,indent=2))
    return 0 if clear else 1


if __name__=='__main__':raise SystemExit(main())
