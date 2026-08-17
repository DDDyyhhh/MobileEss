"""Expose FAST-LIO's Livox CustomMsg as a separate Nav2 PointCloud2 stream."""

from __future__ import annotations

import struct

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

POINT_PACKER = struct.Struct('<fff')


def is_inside_self_filter_box(point, boxes: list[float]) -> bool:
    """Return whether a Livox point is inside one configured vehicle box."""
    for offset in range(0, len(boxes), 6):
        min_x, max_x, min_y, max_y, min_z, max_z = boxes[offset:offset + 6]
        if (min_x <= point.x <= max_x and min_y <= point.y <= max_y
                and min_z <= point.z <= max_z):
            return True
    return False


class LivoxObstacleCloud(Node):
    """Preserve FAST-LIO's CustomMsg input while giving Nav2 PointCloud2 data."""

    def __init__(self) -> None:
        super().__init__('livox_obstacle_cloud')
        self.declare_parameter('source_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/nav2/mid360_obstacles')
        self.declare_parameter(
            'self_filter_boxes', [-0.55, -0.18, -0.42, 0.42, -0.15, 1.20])
        self.declare_parameter('point_stride', 4)
        self._self_filter_boxes = list(
            self.get_parameter('self_filter_boxes').value)
        self._point_stride = int(self.get_parameter('point_stride').value)
        if len(self._self_filter_boxes) % 6 != 0:
            raise ValueError(
                'self_filter_boxes must contain six values per axis-aligned box')
        if self._point_stride < 1:
            raise ValueError('point_stride must be at least one')
        self.create_subscription(CustomMsg, str(self.get_parameter('source_topic').value),
                                 self._cloud_callback, 1)
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), 1)

    def _cloud_callback(self, cloud: CustomMsg) -> None:
        output = PointCloud2()
        output.header = cloud.header
        output.height = 1
        output.fields = POINT_FIELDS
        output.is_bigendian = False
        output.point_step = 12
        output.is_dense = False
        # Keep only the newest cloud while the processor is busy. Nav2's rolling
        # local map is safer with recent sparse observations than stale dense ones.
        data = bytearray((len(cloud.points) // self._point_stride + 1) * output.point_step)
        retained_count = 0
        for source_index, point in enumerate(cloud.points):
            if (source_index % self._point_stride != 0
                    or is_inside_self_filter_box(point, self._self_filter_boxes)):
                continue
            POINT_PACKER.pack_into(data, retained_count * output.point_step,
                                   point.x, point.y, point.z)
            retained_count += 1
        output.width = retained_count
        output.row_step = retained_count * output.point_step
        output.data = bytes(data[:output.row_step])
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxObstacleCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
