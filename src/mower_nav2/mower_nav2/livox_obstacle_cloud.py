"""Publish filtered Livox obstacle evidence for the local TTL layer."""

from __future__ import annotations

import struct

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from .livox_costmap_evidence import filter_livox_costmap_evidence
from .livox_serialized import decode_livox_xyz


POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
POINT_PACKER = struct.Struct('<fff')


class LivoxObstacleCloud(Node):
    """Convert CustomMsg into filtered transient obstacle marks.

    FAST-LIO continues to consume the original CustomMsg. The timed dynamic
    layer is the sole local-costmap owner of these marks and expires its own
    cells without relying on geometric clearing rays.
    """

    def __init__(self) -> None:
        super().__init__('livox_obstacle_cloud')
        self.declare_parameter('source_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/nav2/mid360_obstacles')
        self.declare_parameter('stamp_with_receipt_time', True)
        # Opt-in until the serialized path has passed target/field validation.
        self.declare_parameter('serialized_input', False)
        self.declare_parameter(
            'self_filter_boxes', [-0.55, -0.18, -0.42, 0.42, -0.15, 1.20])
        self.declare_parameter('point_stride', 4)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('ground_cell_size', 0.20)
        self.declare_parameter('ground_band', 0.10)
        self.declare_parameter('obstacle_min_height', 0.15)
        self.declare_parameter('obstacle_max_height', 1.50)
        self.declare_parameter('min_points_per_cell', 3)
        self.declare_parameter('obstacle_cell_size', 0.10)
        self._params = {
            name: self.get_parameter(name).value for name in (
                'self_filter_boxes', 'point_stride', 'min_range', 'max_range',
                'ground_cell_size', 'ground_band', 'obstacle_min_height',
                'obstacle_max_height', 'min_points_per_cell',
                'obstacle_cell_size')}
        self._stamp_with_receipt_time = bool(
            self.get_parameter('stamp_with_receipt_time').value)
        if self._params['point_stride'] < 1:
            raise ValueError('point_stride must be at least one')
        if len(self._params['self_filter_boxes']) % 6 != 0:
            raise ValueError(
                'self_filter_boxes must contain six values per axis-aligned box')
        source_topic = str(self.get_parameter('source_topic').value)
        serialized = bool(self.get_parameter('serialized_input').value)
        if serialized:
            self.create_subscription(
                CustomMsg, source_topic, self._serialized_callback, 1, raw=True)
        else:
            self.create_subscription(CustomMsg, source_topic, self._cloud_callback, 1)
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), 1)

    @staticmethod
    def _to_cloud(source: CustomMsg, points: list[object],
                  receipt_stamp=None) -> PointCloud2:
        output = PointCloud2()
        # Never alias source.header: FAST-LIO must retain the raw Livox
        # acquisition timestamp even when Nav2 evidence uses receipt time.
        output.header.frame_id = source.header.frame_id
        if receipt_stamp is None:
            output.header.stamp.sec = source.header.stamp.sec
            output.header.stamp.nanosec = source.header.stamp.nanosec
        else:
            output.header.stamp.sec = receipt_stamp.sec
            output.header.stamp.nanosec = receipt_stamp.nanosec
        output.height = 1
        output.fields = POINT_FIELDS
        output.is_bigendian = False
        output.point_step = POINT_PACKER.size
        output.is_dense = True
        data = bytearray(len(points) * output.point_step)
        for index, point in enumerate(points):
            POINT_PACKER.pack_into(
                data, index * output.point_step, point.x, point.y, point.z)
        output.width = len(points)
        output.row_step = len(data)
        output.data = bytes(data)
        return output

    def _cloud_callback(self, cloud: CustomMsg) -> None:
        marks, _ = filter_livox_costmap_evidence(cloud.points, **self._params)
        self._publish_marks(cloud, marks)

    def _serialized_callback(self, data: bytes) -> None:
        try:
            frame = decode_livox_xyz(data, self._params['point_stride'])
        except ValueError as exc:
            # Do not publish an empty/fresh cloud when decoding fails. The
            # existing evidence timeout must stop navigation on missing data.
            self.get_logger().error('Rejected serialized Livox input: ' + str(exc))
            return
        params = dict(self._params, point_stride=1)
        marks, _ = filter_livox_costmap_evidence(frame.points, **params)
        source = CustomMsg()
        source.header.frame_id = frame.frame_id
        source.header.stamp.sec = frame.stamp_sec
        source.header.stamp.nanosec = frame.stamp_nanosec
        self._publish_marks(source, marks)

    def _publish_marks(self, cloud, marks) -> None:
        # Raw Livox stamps can lag receipt by hundreds of milliseconds. Nav2's
        # rolling local frame requires a current TF instant; the source message
        # remains untouched for FAST-LIO.
        receipt_stamp = self.get_clock().now().to_msg()
        nav2_stamp = receipt_stamp if self._stamp_with_receipt_time else None
        self._publisher.publish(self._to_cloud(cloud, marks, nav2_stamp))


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
