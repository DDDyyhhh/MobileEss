"""RK3588-only localization replay: transforms recorded scans, never controls CAN."""
import json
import math
import os
from pathlib import Path
import platform
import sqlite3
import subprocess
import sys
import time

import numpy as np


def main():
    if platform.machine() not in ('aarch64', 'arm64'):
        raise SystemExit('Run ROS replay on RK3588')
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    session = Path(sys.argv[1])
    output = Path(sys.argv[2])
    dbfile = next((session/'bag').glob('*.db3'))
    with sqlite3.connect(f'file:{dbfile}?mode=ro', uri=True) as db:
        topic = db.execute("select id from topics where name='/cloud_registered'").fetchone()[0]
        blobs = [row[0] for row in db.execute(
            'select data from messages where topic_id=? order by timestamp desc limit 5', (topic,))]
    scans = [deserialize_message(blob, PointCloud2) for blob in blobs]
    rclpy.init()
    node = rclpy.create_node('startup_replay_check')
    publisher = node.create_publisher(PointCloud2, '/test/startup_cloud', qos_profile_sensor_data)
    poses = []
    events = []
    node.create_subscription(Odometry, '/fixed_map_localization/odometry',
                             lambda msg: poses.append(msg.pose.pose), 10)
    node.create_subscription(DiagnosticArray, '/fixed_map_localization/diagnostics',
                             lambda msg: events.extend(s.message for s in msg.status), 10)
    results = []
    executable = '/home/cat/MobileEss_ws/install/fast_lio/lib/fast_lio/fixed_map_localizer'
    config = '/home/cat/MobileEss_ws/src/mower_nav2/config/localizer_field.yaml'
    try:
        for x, y, degrees in [(0, 0, 0), (1.2, -0.6, 90), (-1.3, 0.5, 180), (0.7, 1.1, 45), (2.6, 0.7, 270)]:
            poses.clear()
            events.clear()
            yaw = math.radians(degrees)
            rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                                 [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            transformed = []
            for original in scans:
                msg = deserialize_message(blobs[len(transformed)], PointCloud2)
                buffer = bytearray(msg.data)
                fields = {f.name:f.offset for f in msg.fields}
                arrays = [np.ndarray((msg.width,), dtype='<f4', buffer=buffer,
                                     offset=fields[n], strides=(msg.point_step,)) for n in ('x', 'y', 'z')]
                xyz = (np.column_stack(arrays) - [x, y, 0]) @ rotation
                for i, array in enumerate(arrays):
                    array[:] = xyz[:, i]
                msg.data = bytes(buffer)
                msg.header.frame_id = 'odom'
                transformed.append(msg)
            log_path = output.parent/f'startup-replay-{degrees}.log'
            with log_path.open('w') as log:
                process = subprocess.Popen([executable, '--ros-args', '--params-file', config,
                    '-p', f'map_file_path:={session / "map.pcd"}', '-p', 'source_topic:=/test/startup_cloud',
                    '-p', 'publish_tf:=false'], stdout=log, stderr=log)
                begin = time.monotonic()
                try:
                    index = 0
                    while time.monotonic()-begin < 40 and not poses:
                        msg = transformed[index % len(transformed)]
                        msg.header.stamp = node.get_clock().now().to_msg()
                        publisher.publish(msg)
                        index += 1
                        until = time.monotonic()+0.55
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=0.05)
                    result = {'expected': [x, y, degrees], 'events': list(events),
                              'elapsed_s': time.monotonic()-begin, 'accepted': bool(poses)}
                    if poses:
                        pose = poses[-1]
                        actual_yaw = 2*math.atan2(pose.orientation.z, pose.orientation.w)
                        yaw_error = abs(math.atan2(math.sin(actual_yaw-yaw), math.cos(actual_yaw-yaw)))
                        distance = math.hypot(pose.position.x-x, pose.position.y-y)
                        result.update(position_error_m=distance, yaw_error_deg=math.degrees(yaw_error),
                                      passed=distance < 0.2 and yaw_error < math.radians(5))
                    else:
                        result['passed'] = False
                    results.append(result)
                    print(json.dumps(result), flush=True)
                finally:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
        output.write_text(json.dumps({'type': 'transformed_recorded_scan_replay', 'cases': results}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if all(result['passed'] for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
