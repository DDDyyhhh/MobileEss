"""Capture a fresh stationary corner and replay the real gate; no commands."""
import argparse
import json
import math
from pathlib import Path
import platform
import time
from types import SimpleNamespace


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():parser.error('Require board and new evidence')
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from nav2_msgs.msg import Costmap
    from sensor_msgs.msg import PointCloud2
    from rcl_interfaces.srv import GetParameters
    from tf2_ros import Buffer,TransformListener
    rclpy.init();node=rclpy.create_node('corner_costmap_capture')
    buffer=Buffer();listener=TransformListener(buffer,node)
    state={};accepted=0
    def received(key,msg):state[key]=(time.monotonic(),msg)
    def diagnostics(msg):
        nonlocal accepted
        accepted+=sum(s.message=='accepted' for s in msg.status)
    node.create_subscription(Costmap,'/local_costmap/costmap_raw',lambda m:received('costmap',m),1)
    node.create_subscription(Odometry,'/odometry/filtered',lambda m:received('odom',m),10)
    node.create_subscription(PointCloud2,'/nav2/mid360_obstacles',lambda m:received('cloud',m),1)
    node.create_subscription(DiagnosticArray,'/fixed_map_localization/diagnostics',diagnostics,10)
    result=dict(passive_only=True,completed=False)
    try:
        if 'cmd_vel_can_bridge' in node.get_node_names():raise RuntimeError('Control bridge must be absent')
        client=node.create_client(GetParameters,'/collision_gate/get_parameters')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Gate parameters unavailable')
        request=GetParameters.Request();request.names=['robot_radius','prediction_horizon','prediction_dt','collision_cost']
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=4)
        if not future.done() or future.result() is None:raise RuntimeError('Gate query timed out')
        values=future.result().values
        params=dict(robot_radius=values[0].double_value,prediction_horizon=values[1].double_value,
                    prediction_dt=values[2].double_value,collision_cost=values[3].integer_value)
        start=time.monotonic()
        while time.monotonic()-start<20:
            rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if accepted<3 or set(state)!={'costmap','odom','cloud'}:raise RuntimeError('Fresh accepted localization required')
        ages={k:now-v[0] for k,v in state.items()}
        if any(v>.3 for v in ages.values()):raise RuntimeError('Capture inputs stale')
        cm=state['costmap'][1];od=state['odom'][1];cloud=state['cloud'][1]
        cloud_age=node.get_clock().now().nanoseconds/1e9-cloud.header.stamp.sec-cloud.header.stamp.nanosec/1e9
        if not -.05<=cloud_age<=.3 or cm.header.frame_id!=od.header.frame_id:raise RuntimeError('Invalid cloud age or frames')
        tf=buffer.lookup_transform('map',cm.header.frame_id,Time(),timeout=Duration(seconds=.1))
        q=tf.transform.rotation;t=tf.transform.translation;v=od.pose.pose;qod=v.orientation
        yaw=math.atan2(2*(qod.w*qod.z+qod.x*qod.y),1-2*(qod.y*qod.y+qod.z*qod.z))
        result.update(completed=True,accepted_count=accepted,input_ages_s=ages,cloud_age_s=cloud_age,
            parameters=params,odom_pose=[v.position.x,v.position.y,yaw],frame_id=cm.header.frame_id,
            map_from_costmap=dict(translation=[t.x,t.y,t.z],quaternion=[q.x,q.y,q.z,q.w]),
            costmap=dict(resolution=cm.metadata.resolution,size_x=cm.metadata.size_x,size_y=cm.metadata.size_y,
                         origin=[cm.metadata.origin.position.x,cm.metadata.origin.position.y],data=list(cm.data)))
    except Exception as exc:result['error']=str(exc)
    finally:
        with args.output.open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps({k:v for k,v in result.items() if k!='costmap'}),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
