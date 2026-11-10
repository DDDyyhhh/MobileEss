"""RK3588-only SmoothPath review; no FollowPath, velocity or CAN commands."""
import argparse
import json
import math
from pathlib import Path
import platform


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():
        parser.error('Require board and new review evidence')
    from mower_nav2.reviewed_route import validate_review,project,path_length
    proposal=json.loads(args.proposal.read_text());reference,_=validate_review(proposal)
    import rclpy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import SmoothPath
    rclpy.init();node=rclpy.create_node('reviewed_route_smoothing_only')
    result=dict(passive_only=True,motion_goal_sent=False,completed=False)
    try:
        if 'cmd_vel_can_bridge' in node.get_node_names():raise RuntimeError('Command bridge must be absent')
        client=ActionClient(node,SmoothPath,'/smooth_path')
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Smoother unavailable')
        request=SmoothPath.Goal();request.path.header.frame_id='map';request.path.header.stamp=node.get_clock().now().to_msg()
        request.check_for_collisions=True;request.max_smoothing_duration.sec=2
        for x,y,a in reference:
            p=PoseStamped();p.header=request.path.header;p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);request.path.poses.append(p)
        future=client.send_goal_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or not future.result() or not future.result().accepted:raise RuntimeError('Smoothing rejected')
        done=future.result().get_result_async();rclpy.spin_until_future_complete(node,done,timeout_sec=8)
        if not done.done() or done.result() is None:raise RuntimeError('Smoothing timed out')
        response=done.result();result['action_status']=response.status;result['was_completed']=response.result.was_completed
        path=[]
        for p in response.result.path.poses:
            q=p.pose.orientation
            path.append([p.pose.position.x,p.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
        result['execution_path_xy_yaw_rad']=path
        if response.status!=4 or not response.result.was_completed:raise RuntimeError('Smoothing did not complete')
        candidate=dict(proposal,execution_path_xy_yaw_rad=path);validate_review(candidate)
        result.update(completed=True,path_length_m=path_length(path),
                      maximum_vertex_deviation_m=max(project(reference,p[:2])[0] for p in path))
    except Exception as exc:result['error']=str(exc)
    finally:
        with args.output.open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps({k:v for k,v in result.items() if k!='execution_path_xy_yaw_rad'}),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
