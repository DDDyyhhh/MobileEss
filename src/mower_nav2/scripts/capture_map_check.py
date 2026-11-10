"""Save stationary registered scans aligned by accepted fixed-map localization."""
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np


def main():
    if platform.machine() not in ('aarch64', 'arm64'):
        raise SystemExit('Run on RK3588')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('stationary_map_check')
    transform = None
    last_transform = 0.0
    batches = []
    def on_pose(msg):
        nonlocal transform, last_transform
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        transform = rotation, np.array([p.x, p.y, p.z])
        last_transform = time.monotonic()
    def on_cloud(msg):
        if transform is None or time.monotonic()-last_transform > 2.5 or msg.header.frame_id != 'odom':
            return
        if msg.height != 1 or msg.is_bigendian:
            return
        offsets = {f.name: f.offset for f in msg.fields}
        points = np.column_stack([np.ndarray((msg.width,), dtype='<f4', buffer=bytes(msg.data),
            offset=offsets[name], strides=(msg.point_step,)) for name in ('x', 'y', 'z')])
        rotation, position = transform
        batches.append((points@rotation.T+position).astype('<f4'))
    node.create_subscription(Odometry, '/fixed_map_localization/odometry', on_pose, 10)
    node.create_subscription(PointCloud2, '/cloud_registered', on_cloud, qos_profile_sensor_data)
    begin = time.monotonic()
    while time.monotonic()-begin < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    out = Path(sys.argv[1])
    if batches:
        np.savez_compressed(out, points=np.concatenate(batches),
                            batch_sizes=np.array([len(p) for p in batches]))
    print(json.dumps({'scans': len(batches), 'points': sum(len(p) for p in batches)}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if batches else 1


if __name__ == '__main__':
    raise SystemExit(main())
