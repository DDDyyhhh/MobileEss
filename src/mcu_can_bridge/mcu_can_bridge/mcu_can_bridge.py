"""Bridge STM32 SocketCAN wheel feedback into a ROS 2 twist observation."""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node

from .protocol import WHEEL_FEEDBACK_CAN_ID, body_twist_from_wheels, parse_wheel_feedback


CAN_RAW = getattr(socket, 'CAN_RAW', 1)
SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
CAN_RAW_FILTER = 1
CAN_FRAME_STRUCT = struct.Struct('=IB3x8s')
CAN_FILTER_STRUCT = struct.Struct('=II')


class McuCanBridge(Node):
    def __init__(self) -> None:
        super().__init__('mcu_can_bridge')
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('can_id', WHEEL_FEEDBACK_CAN_ID)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('twist_topic', '/wheel/odometry/twist')
        self.declare_parameter('diagnostics_topic', '/wheel/diagnostics')
        self.declare_parameter('track_width_m', 0.690)
        self.declare_parameter('feedback_timeout_sec', 0.150)
        self.declare_parameter('recovery_frames', 3)
        self.declare_parameter('linear_x_variance', 0.0025)
        self.declare_parameter('angular_z_variance', 0.0025)

        self._interface = str(self.get_parameter('interface').value)
        self._can_id = int(self.get_parameter('can_id').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._track_width_m = float(self.get_parameter('track_width_m').value)
        self._timeout_sec = float(self.get_parameter('feedback_timeout_sec').value)
        self._recovery_frames = max(1, int(self.get_parameter('recovery_frames').value))
        self._linear_x_variance = float(self.get_parameter('linear_x_variance').value)
        self._angular_z_variance = float(self.get_parameter('angular_z_variance').value)
        if self._track_width_m <= 0.0 or self._timeout_sec <= 0.0:
            raise ValueError('track_width_m and feedback_timeout_sec must be positive')

        self._twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            str(self.get_parameter('twist_topic').value), 20)
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value), 10)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._socket: Optional[socket.socket] = None
        self._last_feedback_monotonic: Optional[float] = None
        self._valid_frames = 0
        self._malformed_frames = 0
        self._dropped_frames = 0
        self._active = False
        self._last_error = ''
        self._last_timeout_report = 0.0
        self._worker = threading.Thread(target=self._receive_loop, daemon=True)
        self._worker.start()

    def _open_socket(self) -> socket.socket:
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
        can_socket.setsockopt(
            SOL_CAN_RAW, CAN_RAW_FILTER,
            CAN_FILTER_STRUCT.pack(self._can_id, 0x7FF))
        can_socket.bind((self._interface,))
        can_socket.settimeout(0.05)
        return can_socket

    def _receive_loop(self) -> None:
        try:
            self._socket = self._open_socket()
            self.get_logger().info(
                f'Listening for CAN 0x{self._can_id:03X} on {self._interface}')
        except Exception as exc:
            self._record_error(f'CAN open failed: {exc}')
            return
        while not self._stop.is_set():
            try:
                raw_frame = self._socket.recv(CAN_FRAME_STRUCT.size)
            except socket.timeout:
                self._deactivate_if_timed_out()
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self._record_error(f'CAN receive failed: {exc}')
                break
            self._handle_frame(raw_frame)

    def _handle_frame(self, raw_frame: bytes) -> None:
        if len(raw_frame) < CAN_FRAME_STRUCT.size:
            with self._lock:
                self._malformed_frames += 1
            return
        can_id, dlc, data = CAN_FRAME_STRUCT.unpack(raw_frame[:CAN_FRAME_STRUCT.size])
        if can_id & 0x7FF != self._can_id or dlc != 8:
            with self._lock:
                self._dropped_frames += 1
            return
        try:
            wheels = parse_wheel_feedback(data)
            linear_x, angular_z = body_twist_from_wheels(wheels, self._track_width_m)
        except ValueError as exc:
            with self._lock:
                self._malformed_frames += 1
            self._record_error(str(exc))
            return
        now = time.monotonic()
        with self._lock:
            self._last_feedback_monotonic = now
            self._valid_frames += 1
            if self._valid_frames >= self._recovery_frames:
                self._active = True
        if self._active:
            self._publish_twist(linear_x, angular_z)

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        message = TwistWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.twist.twist.linear.x = linear_x
        message.twist.twist.angular.z = angular_z
        message.twist.covariance[0] = self._linear_x_variance
        message.twist.covariance[35] = self._angular_z_variance
        self._twist_pub.publish(message)

    def _deactivate_if_timed_out(self) -> None:
        now = time.monotonic()
        with self._lock:
            last_feedback = self._last_feedback_monotonic
            timed_out = last_feedback is not None and now - last_feedback > self._timeout_sec
            if timed_out:
                self._active = False
                self._valid_frames = 0
        if timed_out and now - self._last_timeout_report > 1.0:
            self._last_timeout_report = now
            self.get_logger().warn(
                'Wheel feedback timed out; observation disabled without publishing zero')

    def _record_error(self, error: str) -> None:
        with self._lock:
            self._last_error = error
        self.get_logger().error(error)

    def _publish_diagnostics(self) -> None:
        with self._lock:
            active = self._active
            values = {
                'interface': self._interface,
                'can_id': f'0x{self._can_id:03X}',
                'active': str(active).lower(),
                'valid_frames': str(self._valid_frames),
                'malformed_frames': str(self._malformed_frames),
                'dropped_frames': str(self._dropped_frames),
                'last_error': self._last_error,
            }
        status = DiagnosticStatus()
        status.name = 'mcu_can_bridge/wheel_feedback'
        status.level = DiagnosticStatus.OK if active else DiagnosticStatus.WARN
        status.message = 'active' if active else 'wheel observation disabled'
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostic_pub.publish(diagnostics)

    def destroy_node(self):
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
