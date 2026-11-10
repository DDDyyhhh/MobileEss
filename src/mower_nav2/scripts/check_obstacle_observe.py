"""RK3588-only passive obstacle evidence; never publishes velocity or CAN."""
import argparse
import json
import math
from pathlib import Path
import platform
import socket
import struct
import time
from types import SimpleNamespace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--probe-linear-speed', type=float, choices=(.1, .4))
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64') or args.output.exists():
        parser.error('Run on RK3588 with a new output file')
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from rcl_interfaces.srv import GetParameters
    from mower_nav2.collision_gate import CollisionGate
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node('obstacle_passive_check')
    if 'cmd_vel_can_bridge' in node.get_node_names():
        raise RuntimeError('Stop control bridge before passive observation')
    buffer = Buffer(); listener = TransformListener(buffer, node)
    result = {'passive_only': True, 'can_control_frames': 0, 'accepted_count': 0,
              'cloud_frames': 0, 'frames_with_target_hits': 0,
              'frames_with_target_cost': 0, 'transform_skips': 0, 'samples': [],
              'target_roi_base_m': {'x': [1.2, 2.8], 'y': [-.6, .6], 'z': [.15, 1.05]}}
    costmap = odometry = None
    costmap_time = odom_time = 0.
    probe_params = None
    if args.probe_linear_speed is not None:
        result.update(prediction_probe_speed_m_s=args.probe_linear_speed,
                      prediction_probe_samples=0,prediction_probe_clear_samples=0,
                      prediction_probe_publishes_velocity=False)
    def diagnostics(msg):
        result['accepted_count'] += sum(s.message == 'accepted' for s in msg.status)
    def cost_cb(msg):
        nonlocal costmap,costmap_time
        costmap = msg; costmap_time=time.monotonic()
    def odom_cb(msg):
        nonlocal odometry,odom_time
        odometry=msg;odom_time=time.monotonic()
    def prediction_probe(msg):
        if probe_params is None or costmap is None or odometry is None:
            return
        now=time.monotonic()
        age=(node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)
        if now-costmap_time>.3 or now-odom_time>.3 or not -.05<=age<=.3:
            return
        if costmap.header.frame_id!=odometry.header.frame_id:
            return
        # Reuse the live gate's collision method and its queried parameters.
        # This object has no ROS publishers, timers or CAN socket.
        probe=SimpleNamespace(_costmap=costmap,_odom=odometry,
            _radius=probe_params['robot_radius'],_horizon=probe_params['prediction_horizon'],
            _dt=probe_params['prediction_dt'],_collision_cost=probe_params['collision_cost'],
            _last_reason='')
        probe._cell_cost=lambda x,y:CollisionGate._cell_cost(probe,x,y)
        cmd=Twist();cmd.linear.x=args.probe_linear_speed
        blocked=CollisionGate._collision(probe,cmd)
        result['prediction_probe_samples']+=1
        result['prediction_probe_clear_samples']+=int(not blocked)
        result['prediction_probe_last_reason']=probe._last_reason
    def transform(point, tf):
        x,y,z = point; q=tf.transform.rotation; t=tf.transform.translation
        # Unit-quaternion vector rotation, including measured roll/pitch.
        ux,uy,uz=q.x,q.y,q.z
        cx,cy,cz=uy*z-uz*y,uz*x-ux*z,ux*y-uy*x
        return (x+2*(q.w*cx+uy*cz-uz*cy)+t.x,
                y+2*(q.w*cy+uz*cx-ux*cz)+t.y,
                z+2*(q.w*cz+ux*cy-uy*cx)+t.z)
    def cloud_cb(msg):
        result['cloud_frames'] += 1
        prediction_probe(msg)
        try:
            base_tf=buffer.lookup_transform('base_link',msg.header.frame_id,Time(),timeout=Duration(seconds=0))
            map_tf=buffer.lookup_transform(costmap.header.frame_id,msg.header.frame_id,Time(),timeout=Duration(seconds=0)) if costmap else None
        except Exception:
            result['transform_skips'] += 1
            return
        hits=[]; marked=0
        for point in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True):
            b=transform(point,base_tf)
            if not (1.2<=b[0]<=2.8 and abs(b[1])<=.6 and .15<=b[2]<=1.05):
                continue
            hits.append(b)
            if map_tf and costmap:
                m=transform(point,map_tf); md=costmap.metadata
                ix=math.floor((m[0]-md.origin.position.x)/md.resolution)
                iy=math.floor((m[1]-md.origin.position.y)/md.resolution)
                if 0<=ix<md.size_x and 0<=iy<md.size_y and costmap.data[iy*md.size_x+ix]>=253:
                    marked+=1
        if len(hits)>=5: result['frames_with_target_hits'] += 1
        if marked>=5: result['frames_with_target_cost'] += 1
        if hits:
            result['samples'].append({'hits':len(hits),'marked_hits':marked,
                'x_min_m':min(p[0] for p in hits),'x_max_m':max(p[0] for p in hits),
                'z_min_m':min(p[2] for p in hits),'z_max_m':max(p[2] for p in hits)})
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',diagnostics,10)
    node.create_subscription(Costmap,'/local_costmap/costmap_raw',cost_cb,1)
    node.create_subscription(Odometry,'/odometry/filtered',odom_cb,10)
    node.create_subscription(PointCloud2,'/nav2/mid360_obstacles',cloud_cb,1)
    bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW); bus.bind(('can0',));bus.setblocking(False)
    start=time.monotonic()
    try:
        if args.probe_linear_speed is not None:
            client=node.create_client(GetParameters,'/collision_gate/get_parameters')
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Collision gate parameters unavailable')
            names=['robot_radius','prediction_horizon','prediction_dt','collision_cost']
            request=GetParameters.Request();request.names=names
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=3)
            if not future.done() or future.result() is None:
                raise RuntimeError('Collision gate parameter query timeout')
            values=future.result().values
            probe_params=dict(zip(names,[values[0].double_value,values[1].double_value,
                                         values[2].double_value,values[3].integer_value]))
            if any(not math.isfinite(v) or v<=0 for v in probe_params.values()):
                raise RuntimeError('Invalid gate prediction parameters')
            result['prediction_probe_parameters']=probe_params
        while time.monotonic()-start<20:
            rclpy.spin_once(node,timeout_sec=.02)
            for _ in range(256):
                try: frame=bus.recv(16)
                except BlockingIOError: break
                cid,dlc,data=struct.unpack('=IB3x8s',frame)
                if cid==5:
                    result['can_control_frames']+=1
                    raise RuntimeError('Unexpected control CAN during passive observation')
        usable=result['cloud_frames']-result['transform_skips']
        result['usable_cloud_frames']=usable
        result['recognition_passed']=(result['accepted_count']>=3 and usable>=50
            and result['frames_with_target_hits']/usable>=.8
            and result['frames_with_target_cost']/usable>=.8
            and result['can_control_frames']==0)
        if args.probe_linear_speed is not None:
            n=result['prediction_probe_samples']
            result['approach_probe_passed']=(n>=50 and result['prediction_probe_clear_samples']/n>=.8)
    except Exception as exc:
        result.update(recognition_passed=False,error=str(exc))
    finally:
        args.output.write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
        bus.close();node.destroy_node();rclpy.shutdown()
    return 0 if result['recognition_passed'] and result.get('approach_probe_passed',True) else 1


if __name__=='__main__':
    raise SystemExit(main())
