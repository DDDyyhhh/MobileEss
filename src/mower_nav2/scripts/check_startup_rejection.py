"""Synthetic ambiguous/featureless startup checks on RK3588, no CAN nodes."""
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np


def main():
    if platform.machine() not in ('aarch64', 'arm64'):
        raise SystemExit('Run on RK3588')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    output = Path(sys.argv[1])
    rclpy.init()
    node = rclpy.create_node('startup_rejection_check')
    publisher = node.create_publisher(PointCloud2, '/test/startup_cloud', qos_profile_sensor_data)
    poses, events, results = [], [], []
    node.create_subscription(Odometry, '/fixed_map_localization/odometry', lambda msg: poses.append(msg), 10)
    node.create_subscription(DiagnosticArray, '/fixed_map_localization/diagnostics',
                             lambda msg: events.extend(s.message for s in msg.status), 10)
    random = np.random.default_rng(42)
    structure = random.uniform([-6, -5, 0], [6, 5, 2.5], (300, 3)).astype('<f4')
    try:
        for name, cloud, target, expected in [
            ('ambiguous', structure, np.vstack([structure, structure+[1.5, 0, 0]]), 'startup_ambiguous'),
            ('floor_only', np.column_stack([structure[:, :2], np.full(300, -0.49)]),
             structure, 'startup_insufficient_structure')]:
            poses.clear()
            events.clear()
            target = target.astype('<f4')
            path = output.parent/f'startup-{name}.pcd'
            header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                      f'WIDTH {len(target)}\nHEIGHT 1\nPOINTS {len(target)}\nDATA binary\n')
            path.write_bytes(header.encode()+target.tobytes())
            msg = PointCloud2()
            msg.header.frame_id = 'odom'
            msg.height, msg.width = 1, len(cloud)
            msg.fields = [PointField(name=n, offset=4*i, datatype=7, count=1)
                          for i, n in enumerate(('x', 'y', 'z'))]
            msg.point_step, msg.row_step = 12, len(cloud)*12
            msg.data = cloud.astype('<f4').tobytes()
            with (output.parent/f'startup-{name}.log').open('w') as log:
                process = subprocess.Popen([
                    '/home/cat/MobileEss_ws/install/fast_lio/lib/fast_lio/fixed_map_localizer',
                    '--ros-args', '--params-file', '/home/cat/MobileEss_ws/src/mower_nav2/config/localizer_field.yaml',
                    '-p', f'map_file_path:={path}', '-p', 'source_topic:=/test/startup_cloud',
                    '-p', 'publish_tf:=false', '-p', 'map_voxel_size_m:=0.05'], stdout=log, stderr=log)
                try:
                    begin = time.monotonic()
                    while time.monotonic()-begin < 10:
                        msg.header.stamp = node.get_clock().now().to_msg()
                        publisher.publish(msg)
                        until = time.monotonic()+0.55
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=0.05)
                    result = dict(case=name, accepted_count=len(poses), events=sorted(set(events)),
                                  passed=not poses and expected in events)
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
        output.write_text(json.dumps(results, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
