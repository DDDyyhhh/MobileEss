"""Plan around the right of the marked central keepout; never execute motion."""
import argparse
import json
import math
from pathlib import Path
import platform
import runpy
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():
        parser.error('Require board and new evidence')
    sys.argv=['check_observe.py',str(args.output),'--goal-x','7.5','--goal-y','.25','--goal-yaw-deg','-90']
    runpy.run_path(str(Path(__file__).with_name('check_observe.py')),run_name='__main__')
    result=json.loads(args.output.read_text())
    result.update(u_route_planning_only=True,motion_approved=False,
                  u_route_via_xy_yaw_deg=[
                      [7.5,.25,-90.], [7.5,-.5,-90.], [7.5,-1.,-90.],
                      [7.5,-1.5,-90.], [7.5,-2.,-90.], [7.5,-2.6,-90.],
                      [7.5,-3.5,-180.], [4.6,-3.5,-180.]])
    if result.get('accepted_count',0)<3 or 'robot_in_map' not in result or 'cmd_vel_can_bridge' in result['node_names']:
        raise RuntimeError('Require passive accepted localization without control bridge')
    import rclpy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathThroughPoses
    rclpy.init();node=rclpy.create_node('keepout_u_path_review')
    try:
        client=ActionClient(node,ComputePathThroughPoses,'/compute_path_through_poses')
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Through-poses planner unavailable')
        request=ComputePathThroughPoses.Goal();request.use_start=True;request.planner_id='GridBased'
        def stamped(x,y,deg):
            msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
            msg.pose.position.x=float(x);msg.pose.position.y=float(y)
            msg.pose.orientation.z=math.sin(math.radians(deg)/2);msg.pose.orientation.w=math.cos(math.radians(deg)/2)
            return msg
        p=result['robot_in_map'];request.start=stamped(*p['xy_m'],p['yaw_deg'])
        request.goals=[stamped(*p) for p in result['u_route_via_xy_yaw_deg']]
        future=client.send_goal_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=8)
        if not future.done() or not future.result() or not future.result().accepted:raise RuntimeError('U path rejected')
        done=future.result().get_result_async();rclpy.spin_until_future_complete(node,done,timeout_sec=15)
        if not done.done() or done.result() is None:raise RuntimeError('U planning timeout')
        response=done.result();path=[]
        for item in response.result.path.poses:
            p=item.pose.position;q=item.pose.orientation
            path.append([p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
        result.update(u_action_status=response.status,u_route_xy_yaw_rad=path,
                      u_route_length_m=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:])))
        result['u_path_computed']=response.status==4 and len(path)>2
    except Exception as exc:
        result.update(u_path_computed=False,u_error=str(exc))
    finally:
        args.output.write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('events','path_xy_m','u_route_xy_yaw_rad')},ensure_ascii=False),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if result.get('u_path_computed') else 1


if __name__=='__main__':raise SystemExit(main())
