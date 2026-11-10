"""Bounded, read-only ROS sensor probe; emits frequency and source age evidence."""
import argparse
import json
import platform
import time
import struct
from types import SimpleNamespace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64'):
        parser.error('run on RK3588')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('mid360_read_only_probe')
    stats = {}
    def callback(topic, msg):
        if isinstance(msg, bytes):
            # Inspect only the CDR Header; do not deserialize ~20k CustomPoints
            # just to measure frequency on the target computer.
            endian = '<' if msg[1] & 1 else '>'
            sec, nanosec, length = struct.unpack_from(endian + 'iII', msg, 4)
            msg = SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
                frame_id=msg[16:16 + length - 1].decode()))
        entry = stats.setdefault(topic, {'count': 0, 'ages': [], 'frames': set(),
                                        'first': time.monotonic(), 'last': 0.0})
        entry['count'] += 1
        entry['last'] = time.monotonic()
        entry['ages'].append(node.get_clock().now().nanoseconds / 1e9
                             - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9)
        entry['frames'].add(msg.header.frame_id)
        if hasattr(msg, 'point_num'):
            entry['last_point_num'] = msg.point_num
        if hasattr(msg, 'pose'):
            p = msg.pose.pose.position
            entry['last_xyz'] = [p.x, p.y, p.z]
    topics = {'/livox/lidar': CustomMsg, '/livox/imu': Imu,
              '/fast_lio/odometry': Odometry}
    subscriptions = [node.create_subscription(kind, topic,
        lambda msg, topic=topic: callback(topic, msg), qos_profile_sensor_data,
        raw=(topic == '/livox/lidar'))
        for topic, kind in topics.items()]
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        output = {}
        for topic in topics:
            e = stats.get(topic)
            if e is None:
                output[topic] = {'count': 0}
                continue
            ages = sorted(e.pop('ages'))
            duration = e['last'] - e['first']
            e['hz'] = (e['count'] - 1) / duration if duration > 0 else 0.0
            e['age_median_sec'] = ages[len(ages)//2]
            e['age_max_sec'] = max(ages)
            e['frames'] = sorted(e['frames'])
            e['last_receipt_age_sec'] = time.monotonic() - e.pop('last')
            e.pop('first')
            output[topic] = e
        print(json.dumps(output, indent=2))
        healthy = all(output[t]['count'] >= 5 and
                      output[t]['last_receipt_age_sec'] < 1.0 for t in topics)
        return 0 if healthy else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
