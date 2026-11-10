"""RK3588-only ROS actions and receive-only CAN evidence for the console."""
from __future__ import annotations

import asyncio
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import platform
import signal
import socket
import struct
import subprocess
import threading
import time

import numpy as np

from .assets import downsample
from .can_feedback import TIMESTAMPNS, arrival_time, stationary_evidence
from .geometry import Grid, angle, split_path
from .visibility import rear_observed


class RosAdapter:
    def __init__(self, assets, config):
        if platform.machine() not in ('aarch64', 'arm64'):
            raise RuntimeError('ROS和车辆接口只能在RK3588执行；本机请显式使用--demo')
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.action import ActionClient
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from nav2_msgs.msg import Costmap
        from sensor_msgs.msg import PointCloud2
        from livox_ros_driver2.msg import CustomMsg
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger
        from nav2_msgs.action import ComputePathToPose, FollowPath, Spin
        from tf2_ros import Buffer, TransformListener
        self.ros, self.assets, self.config = rclpy, assets, config
        from ament_index_python.packages import get_package_share_directory
        profile = Path(get_package_share_directory('mower_console'))/'config'
        self.profile_fingerprints = {p:(p.stat().st_size,p.stat().st_mtime_ns)
                                     for p in profile.iterdir() if p.is_file()}
        rclpy.init(args=[])
        self.node = Node('mower_console')
        self.lock = threading.RLock()
        self.times = {}
        self.api, self.wheels, self.echo = None, [], None
        self.events = deque(maxlen=2000)
        self.generation = 0
        self.localization, self.loc_ok = 'awaiting_initial_alignment', False
        self.gate_reason = ''
        self.trajectory, self.obstacles, self.cloud = deque(maxlen=3000), [], []
        self.rear_clear, self.last_raw = False, 0.
        self.last_raw_stamp = -1.
        self.motion_process, self.action_handle = None, None
        self.audit_file = None
        self.closed = threading.Event()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node, spin_thread=False)
        self.permit_pub = self.node.create_publisher(Bool, '/mower_console/motion_permit', 1)
        self.rear_pub = self.node.create_publisher(Bool, '/mower_console/rear_observed', 1)
        self.hint_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.reset = self.node.create_client(Trigger, '/fixed_map_localizer/relocalize')
        self.node.create_subscription(DiagnosticArray, '/fixed_map_localization/diagnostics', self.on_localization, 10)
        self.node.create_subscription(DiagnosticArray, '/collision_gate/diagnostics', self.on_gate, 10)
        self.node.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.node.create_subscription(Costmap, '/local_costmap/costmap_raw', lambda m: self.mark('costmap'), 1)
        self.node.create_subscription(PointCloud2, '/nav2/mid360_obstacles', self.on_obstacles, 1)
        self.node.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, 1)
        self.node.create_subscription(CustomMsg, '/livox/lidar', self.on_raw, 1, raw=True)
        self.compute = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.reverse_compute = ActionClient(self.node, ComputePathToPose, '/reverse_planning/compute_path_to_pose')
        self.follow = ActionClient(self.node, FollowPath, '/follow_path')
        self.spin = ActionClient(self.node, Spin, '/spin')
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.can = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Keep unrelated OID/BMS traffic out of this process and its receive
        # queue. Retain 0x005 for detecting another command sender; only exact
        # standard data frames may match (extended/RTR frames stay excluded).
        mask = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
        self.can.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                            b''.join(struct.pack('=II', cid, mask) for cid in (5, 0x507, 0x50A)))
        self.can.setsockopt(socket.SOL_SOCKET, TIMESTAMPNS, 1)
        self.can.bind((config.get('can_interface', 'can0'),))
        self.can.settimeout(.1)
        self.can_receive_lag_s, self.can_max_receive_lag_s = {}, {}
        self.can_timestamp_errors = 0
        self.can_thread = threading.Thread(target=self.receive_can, daemon=True)
        self.can_thread.start()
        self.node.create_timer(.1, self.publish_stationary)

    def publish_stationary(self):
        from std_msgs.msg import Bool
        publisher = getattr(self, 'stationary_pub', None)
        if publisher is None:
            self.stationary_pub = self.node.create_publisher(Bool, '/mower_console/stationary', 1)
        with self.lock:
            idle_bus = not any(cid == 5 and time.monotonic()-t < 1. for t,cid,_ in self.events)
        self.stationary_pub.publish(Bool(data=self.stationary() and idle_bus))

    def audit_start(self, task, plan, confirmations):
        directory = Path(self.config.get('evidence_directory', str(Path.home()/'MobileEss_data/console-runs')))
        directory.mkdir(parents=True,exist_ok=True)
        with self.lock:
            stream = (directory/(task['id']+'.jsonl')).open('x',buffering=1)
            try:
                stream.write(json.dumps(dict(kind='start',task=task,plan=plan,
                                            confirmations=confirmations,config=self.config))+'\n')
            except BaseException:
                stream.close()
                raise
            self.audit_file = stream

    def audit_end(self, task):
        with self.lock:
            if self.audit_file:
                self.audit_file.write(json.dumps(dict(kind='end',task=task))+'\n')
                self.audit_file.close()
                self.audit_file=None

    def mark(self, key):
        with self.lock:
            self.times[key] = time.monotonic()

    def receive_can(self):
        while not self.closed.is_set():
            try:
                raw, ancillary, flags, _ = self.can.recvmsg(16, socket.CMSG_SPACE(16))
            except socket.timeout:
                continue
            except OSError:
                return
            if len(raw) != 16:
                continue
            cid, dlc, data = struct.unpack('=IB3x8s', raw)
            if cid & 0xE0000000 or dlc != 8:
                continue
            before = time.monotonic()
            realtime_ns = time.time_ns()
            after = time.monotonic()
            now = (arrival_time(ancillary, (before + after) / 2., realtime_ns)
                   if after - before <= .01 and not flags & socket.MSG_CTRUNC else None)
            if now is None:
                with self.lock:
                    self.can_timestamp_errors += 1
                continue
            with self.lock:
                if cid in (5, 0x507, 0x50A):
                    channel = hex(cid)
                    self.can_receive_lag_s[channel] = after - now
                    self.can_max_receive_lag_s[channel] = max(after - now, self.can_max_receive_lag_s.get(channel, 0.))
                    self.events.append((now, cid, bytes(data)))
                    if self.audit_file:
                        self.audit_file.write(json.dumps(dict(kind='can',t=now,id=cid,data=data.hex()))+'\n')
                if cid == 0x507:
                    self.api = data[4]
                    self.echo = [v/1000. for v in struct.unpack('>hh', data[:4])]
                    self.times['status'] = now
                elif cid == 0x50A:
                    self.wheels = list(struct.unpack('>hhhh', data))
                    self.times['wheels'] = now

    def on_localization(self, msg):
        with self.lock:
            for item in msg.status:
                if item.name != 'fixed_map_localization':
                    continue
                self.localization = item.message
                values = {v.key: v.value for v in item.values}
                generation = int(values.get('generation', self.generation))
                if generation != self.generation:
                    self.trajectory.clear()
                self.generation = generation
                if item.message == 'accepted':
                    self.times['localization'] = time.monotonic()
                    self.loc_ok = True
                elif item.message not in ('heartbeat',):
                    # A transient rejected local ICP doesn't refresh accepted time.
                    if item.message.startswith(('startup_', 'awaiting_', 'relocaliz')) or item.message == 'heartbeat_stale':
                        self.loc_ok = False

    def on_gate(self, msg):
        with self.lock:
            for item in msg.status:
                if item.name == 'mower_nav2/collision_gate':
                    self.gate_reason = item.message
                    self.times['gate'] = time.monotonic()

    def on_odom(self, msg):
        age = self.node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        if -.05 <= age <= .3:
            self.mark('odom')

    def transform(self, source):
        return self.tf.lookup_transform('map', source, self.ros.time.Time())

    @staticmethod
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def pose(self):
        try:
            transform = self.transform('base_link')
            age = self.node.get_clock().now().nanoseconds/1e9-transform.header.stamp.sec-transform.header.stamp.nanosec/1e9
            if not -.05 <= age <= .30:
                return None
            t = transform.transform
            return [t.translation.x, t.translation.y, self.yaw(t.rotation)]
        except Exception:
            return None

    def xyz_in_map(self, msg, maximum):
        age = self.node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        if not -.05 <= age <= .6 or msg.is_bigendian:
            return []
        names = {f.name: f.offset for f in msg.fields if f.datatype == 7}
        if not all(n in names for n in ('x', 'y', 'z')):
            return []
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                          'offsets': [names[n] for n in ('x', 'y', 'z')], 'itemsize': msg.point_step})
        rows = [np.frombuffer(msg.data, dtype=dtype, count=msg.width, offset=i*msg.row_step) for i in range(msg.height)]
        values = np.concatenate(rows)
        points = downsample(np.column_stack([values[n] for n in ('x','y','z')]), .2, maximum)
        t = self.transform(msg.header.frame_id).transform
        q = t.rotation
        x,y,z,w = q.x,q.y,q.z,q.w
        rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        return (points@rotation.T+np.array([t.translation.x,t.translation.y,t.translation.z])).tolist()

    def on_obstacles(self, msg):
        try:
            points = self.xyz_in_map(msg, 2000)
            with self.lock:
                self.obstacles = points
            age = self.node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
            if -.05 <= age <= .3:
                self.mark('obstacles')
        except Exception:
            pass

    def on_cloud(self, msg):
        now = time.monotonic()
        if now-self.times.get('display_cloud', 0.) < .5:
            return
        self.mark('display_cloud')
        try:
            points = self.xyz_in_map(msg, 5000)
            with self.lock:
                self.cloud = points
        except Exception:
            pass

    def on_raw(self, raw):
        now = time.monotonic()
        if now-self.last_raw < .19:
            return
        self.last_raw = now
        from mower_nav2.livox_serialized import decode_livox_xyz
        from std_msgs.msg import Bool
        try:
            frame = decode_livox_xyz(raw, 8)
            stamp = frame.stamp_sec+frame.stamp_nanosec/1e9
            age = self.node.get_clock().now().nanoseconds/1e9-stamp
            clear = (stamp > self.last_raw_stamp and -.05 <= age <= 1. and rear_observed(frame.points))
            self.last_raw_stamp = max(self.last_raw_stamp,stamp)
        except Exception:
            clear = False
        with self.lock:
            self.rear_clear = clear
            self.times['rear'] = now
        self.rear_pub.publish(Bool(data=clear))

    def stationary(self, api_zero=True):
        return self.stationary_evidence(api_zero)['ready']

    def stationary_evidence(self, api_zero=True):
        with self.lock:
            return stationary_evidence(self.events, time.monotonic(), api_zero)

    def snapshot(self):
        now, pose = time.monotonic(), self.pose()
        with self.lock:
            localized = self.loc_ok and now-self.times.get('localization', 0) < 2.5 and pose is not None
            faults = [key+'数据过期' for key,age in [('status',.25),('wheels',.25),('odom',.3),('costmap',.3),('obstacles',.3)] if now-self.times.get(key,0) > age]
            if not localized:
                faults.append('定位未就绪')
            if not self.assets.unchanged() or any(not p.exists() or
                    (p.stat().st_size,p.stat().st_mtime_ns)!=v for p,v in self.profile_fingerprints.items()):
                faults.append('地图或运行配置已改变，请重启观测并重新预览')
            if self.motion_process is not None and self.gate_reason.startswith(('predicted', 'rear', 'non-finite')):
                faults.append('安全门控：'+self.gate_reason)
            if self.motion_process is not None and now-self.times.get('gate',0) > .3:
                faults.append('安全门控诊断过期')
            if self.motion_process is not None and self.api != 2:
                faults.append('STM32 API已退出或被遥控接管')
            if pose and localized and (not self.trajectory or math.dist(pose[:2],self.trajectory[-1][:2]) > .03):
                self.trajectory.append(pose)
            wheels = list(self.wheels)
            wheel_fresh = now-self.times.get('wheels',0) <= .25
            status_fresh = now-self.times.get('status',0) <= .25
            stopped = self.stationary_evidence()
            return dict(demo=False,localized=localized,pose=pose if localized else None,
                        generation=self.generation,map_revision=self.assets.revision,
                        localization=self.localization,stationary=stopped['ready'],healthy=not faults,
                        stationary_evidence=stopped,can_receive_lag_s=dict(self.can_receive_lag_s),
                        can_max_receive_lag_s=dict(self.can_max_receive_lag_s),can_timestamp_errors=self.can_timestamp_errors,
                        fault='；'.join(faults),wheel_mm_s=wheels if wheel_fresh else [],
                        actual_v=sum(wheels)/4000. if wheels and wheel_fresh else 0.,
                        api_mode=self.api if status_fresh else None,feedback_fresh=wheel_fresh,
                        command_echo=self.echo if status_fresh else None,trajectory=list(self.trajectory),
                        obstacles=self.obstacles,cloud=self.cloud,rear_observed=self.rear_clear)

    async def future(self, future, timeout=3.):
        until = time.monotonic()+timeout
        while not future.done():
            if time.monotonic() > until:
                raise TimeoutError('ROS响应超时')
            await asyncio.sleep(.02)
        if future.exception():
            raise future.exception()
        return future.result()

    async def action(self, client, goal, timeout):
        if not client.server_is_ready():
            raise ValueError('Nav2 action未就绪')
        send = client.send_goal_async(goal)
        handle = None
        try:
            handle = await self.future(send)
            if not handle.accepted:
                raise ValueError('Nav2拒绝目标')
            self.action_handle = handle
            result = await self.future(handle.get_result_async(), timeout)
            if result.status != 4:
                raise ValueError('Nav2执行失败或取消：'+str(result.status))
            return result.result
        except BaseException:
            if handle is not None:
                await self.future(handle.cancel_goal_async())
            else:
                send.add_done_callback(lambda f: f.result().cancel_goal_async() if not f.exception() and f.result().accepted else None)
            raise
        finally:
            if self.action_handle is handle:
                self.action_handle = None

    def stamped_pose(self, pose):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(pose[0]), float(pose[1])
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        return msg

    async def plan(self, xy):
        from nav2_msgs.action import ComputePathToPose
        await asyncio.to_thread(self.assets.grid.validate,
                                [{'poses':[[*xy,0.],[*xy,0.]],'kind':'forward'}])
        start = self.snapshot()['pose']
        bearing = math.atan2(xy[1]-start[1],xy[0]-start[0])
        headings = [math.atan2(y,x)%(2*math.pi) for x,y in ((2,0),(2,1),(1,1),(1,2),(0,2),(-1,2),(-1,1),(-2,1),(-2,0),(-2,-1),(-1,-1),(-1,-2),(0,-2),(1,-2),(1,-1),(2,-1))]
        headings.sort(key=lambda h: abs(angle(h-bearing)))
        deadline = time.monotonic()+10.
        last = '未找到满足约束的路线'
        for planner in ('ForwardGrid','ReverseGrid'):
            for yaw in headings:
                if time.monotonic() > deadline:
                    raise ValueError('规划超过10秒，请调整目标')
                goal = ComputePathToPose.Goal()
                goal.goal, goal.start = self.stamped_pose([*xy,yaw]), self.stamped_pose(start)
                goal.use_start, goal.planner_id = True, planner
                try:
                    result = await self.action(self.compute if planner=='ForwardGrid' else self.reverse_compute,goal,.7)
                    poses = [[p.pose.position.x,p.pose.position.y,self.yaw(p.pose.orientation)] for p in result.path.poses]
                    segments = split_path(poses)
                    await asyncio.to_thread(self.assets.grid.validate, segments)
                    return poses
                except (ValueError, TimeoutError) as exc:
                    last = str(exc)
        raise ValueError(last)

    async def validate(self, segments):
        await asyncio.to_thread(self.assets.grid.validate, segments)
        snap = self.snapshot()
        if not snap['healthy']:
            raise ValueError(snap['fault'])
        if any(s['kind']=='reverse' for s in segments):
            if not self.rear_clear or time.monotonic()-self.times.get('rear',0) > .3:
                raise ValueError('后方观测不足，倒车路线不可执行')

    def permit(self, enabled):
        from std_msgs.msg import Bool
        self.permit_pub.publish(Bool(data=bool(enabled)))

    async def arm(self):
        from ament_index_python.packages import get_package_share_directory,get_package_prefix
        if not self.stationary() or any(c==5 and time.monotonic()-t<1. for t,c,_ in self.events):
            raise ValueError('未满足API0停稳条件或存在其他CAN发送者')
        cfg = Path(get_package_share_directory('mower_console'))/'config/cmd_vel_console.yaml'
        executable = Path(get_package_prefix('mcu_can_bridge'))/'lib/mcu_can_bridge/cmd_vel_can_bridge'
        self.motion_process = subprocess.Popen([str(executable),
                                                '--ros-args','--params-file',str(cfg)],start_new_session=True)
        deadline = time.monotonic()+5.
        while self.api != 2:
            if time.monotonic()>deadline or self.motion_process.poll() is not None:
                raise ValueError('STM32未确认API运行状态')
            self.permit(True)
            await asyncio.sleep(.05)

    async def execute_segment(self, segment):
        from nav_msgs.msg import Path as RosPath
        from nav2_msgs.action import FollowPath, Spin
        if segment['kind']=='rotate':
            goal = Spin.Goal()
            goal.target_yaw = float(segment['turn'])
            goal.time_allowance.sec = math.ceil(abs(goal.target_yaw)/.1+15.)
            await self.action(self.spin,goal,goal.time_allowance.sec+3.)
        else:
            path = RosPath()
            path.header.frame_id = 'map'
            path.header.stamp = self.node.get_clock().now().to_msg()
            path.poses = [self.stamped_pose(p) for p in segment['poses']]
            goal = FollowPath.Goal()
            goal.path = path
            goal.controller_id = 'ReversePath' if segment['kind']=='reverse' else 'FollowPath'
            goal.goal_checker_id = 'goal_checker'
            await self.action(self.follow,goal,600.)

    async def wait_stationary(self, require_api_zero=True):
        until = time.monotonic()+5.
        while not self.stationary(api_zero=require_api_zero):
            if time.monotonic()>until:
                raise ValueError('未收到连续1秒的新鲜停稳反馈')
            # Needed at between-segment stops: retain this task's lease only.
            if not require_api_zero:
                self.permit(True)
            await asyncio.sleep(.05)

    async def stop(self):
        self.permit(False)
        if self.action_handle is not None:
            await self.future(self.action_handle.cancel_goal_async())
        try:
            await self.wait_stationary()
        finally:
            process, self.motion_process = self.motion_process, None
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    await asyncio.to_thread(process.wait,3.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    await asyncio.to_thread(process.wait)

    async def relocalize(self, seed):
        from std_srvs.srv import Trigger
        from geometry_msgs.msg import PoseWithCovarianceStamped
        if not self.stationary() or self.motion_process is not None:
            raise ValueError('必须API0停稳后重新定位')
        if seed is None:
            response = await self.future(self.reset.call_async(Trigger.Request()))
            if not response.success:
                raise ValueError(response.message)
        else:
            p = self.stamped_pose(seed)
            msg = PoseWithCovarianceStamped()
            msg.header, msg.pose.pose = p.header, p.pose
            self.hint_pub.publish(msg)
        with self.lock:
            self.loc_ok = False
            self.trajectory.clear()

    def close(self):
        self.closed.set()
        self.permit(False)
        self.can.close()
        self.executor.shutdown(timeout_sec=2.)
        self.node.destroy_node()
        self.ros.shutdown()
