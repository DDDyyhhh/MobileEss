"""Board-only planning of two reviewed short legs; never sends motion goals."""
import argparse
import json
import math
from pathlib import Path
import platform
import runpy
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--relative-forward-m', type=float, default=.6)
    parser.add_argument('--relative-right-m', type=float, default=0.)
    parser.add_argument('--relative-yaw-deg', type=float, default=0.)
    args = parser.parse_args()
    if platform.machine() not in ('aarch64','arm64') or args.output.exists():
        parser.error('Require RK3588 and new evidence')
    if (args.relative_forward_m,args.relative_right_m,args.relative_yaw_deg) != (.6,0.,0.):
        parser.error('Only the fixed forward-then-right review is supported')
    sys.argv = ['check_observe.py',str(args.output),'--relative-forward-m','.6',
                '--relative-right-m','0','--relative-yaw-deg','0']
    runpy.run_path(str(Path(__file__).with_name('check_observe.py')),run_name='__main__')
    result = json.loads(args.output.read_text())
    result['sequence_planning_only'] = True
    result['sequence_motion_approved'] = False
    if result.get('action_status') != 4 or 'cmd_vel_can_bridge' in result['node_names']:
        raise RuntimeError('Require a successful passive first path without a control bridge')
    from mower_nav2.forward_right_sequence import waypoints
    from mower_nav2.ground_check import corridor_error
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    first, second, final_yaw = waypoints(result['robot_in_map'])
    result['second_reviewed_start_xy_m'] = first
    result['second_planning_goal'] = {'xy_m':second,'yaw_deg':final_yaw}
    rclpy.init(); node = rclpy.create_node('forward_right_path_review')
    try:
        if 'cmd_vel_can_bridge' in node.get_node_names():
            raise RuntimeError('Control bridge present during passive review')
        client = ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Planner unavailable')
        request = ComputePathToPose.Goal(); request.use_start = True
        for msg, xy, deg in ((request.start,first,result['robot_in_map']['yaw_deg']),
                             (request.goal,second,final_yaw)):
            msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
            msg.pose.position.x,msg.pose.position.y=xy
            msg.pose.orientation.z=math.sin(math.radians(deg)/2)
            msg.pose.orientation.w=math.cos(math.radians(deg)/2)
        future=client.send_goal_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=8)
        if not future.done() or not future.result() or not future.result().accepted:
            raise RuntimeError('Second path rejected')
        done=future.result().get_result_async()
        rclpy.spin_until_future_complete(node,done,timeout_sec=12)
        if not done.done() or done.result() is None:raise RuntimeError('Second path timeout')
        response=done.result();path=[[s.pose.position.x,s.pose.position.y] for s in response.result.path.poses]
        result.update(second_action_status=response.status,second_path_xy_m=path)
        if response.status!=4 or not path or any(corridor_error(first,second,p,.2) for p in path):
            raise RuntimeError('Second path failed or outside the reviewed short corridor')
        start=result['robot_in_map']['xy_m']
        if any(corridor_error(start,first,p,.2) for p in result['path_xy_m']):
            raise RuntimeError('First path outside the reviewed short corridor')
        result['both_paths_passed']=True
    except Exception as exc:
        result.update(both_paths_passed=False,second_error=str(exc))
    finally:
        args.output.write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('events','path_xy_m','second_path_xy_m')},ensure_ascii=False),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if result.get('both_paths_passed') else 1


if __name__=='__main__':raise SystemExit(main())
