#!/usr/bin/env python3
"""Record compact evidence that FAST-LIO published its configured odometry.

Run only against an isolated rosbag replay namespace. The observer does not publish
commands or TF; it writes one JSON summary after its timeout.
"""

import argparse
import json
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class OdometryObserver(Node):
    def __init__(self, duration_s: float):
        super().__init__("fast_lio_odometry_observer")
        self._messages = []
        self._duration_s = duration_s
        self._max_publishers = []
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscription = self.create_subscription(
            Odometry, "/fast_lio/odometry", self._callback, qos)

    def _callback(self, message: Odometry) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self._messages.append(
            {
                "stamp_s": stamp,
                "frame_id": message.header.frame_id,
                "child_frame_id": message.child_frame_id,
                "covariance_finite": all(value == value for value in message.pose.covariance),
            }
        )

    def _record_publishers(self) -> list[dict]:
        publishers = self.get_publishers_info_by_topic("/fast_lio/odometry")
        snapshot = [
            {
                "node_name": endpoint.node_name,
                "node_namespace": endpoint.node_namespace,
                "reliability": str(endpoint.qos_profile.reliability),
                "depth": endpoint.qos_profile.depth,
            }
            for endpoint in publishers
        ]
        if len(snapshot) >= len(self._max_publishers):
            self._max_publishers = snapshot
        return snapshot

    def summary(self) -> dict:
        publishers = self._record_publishers() if rclpy.ok() else []
        stamps = [message["stamp_s"] for message in self._messages]
        return {
            "topic": "/fast_lio/odometry",
            "observer_qos": {"reliability": "BEST_EFFORT", "depth": 20},
            "publisher_endpoint_count": len(publishers),
            "publisher_qos": publishers,
            "max_publisher_endpoint_count": len(self._max_publishers),
            "max_publisher_qos": self._max_publishers,
            "callback_count": len(self._messages),
            "stamps_monotonic": all(right > left for left, right in zip(stamps, stamps[1:])),
            "first_message": self._messages[0] if self._messages else None,
            "last_message": self._messages[-1] if self._messages else None,
            "duration_s": self._duration_s,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = OdometryObserver(args.duration)
    completed = False
    try:
        rclpy.spin_once(node, timeout_sec=0.0)
        end = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            node._record_publishers()
        completed = True
    finally:
        summary = node.summary()
        summary["completed_observation_period"] = completed
        args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
