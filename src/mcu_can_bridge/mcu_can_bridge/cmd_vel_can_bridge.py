"""Safely relay Nav2 velocity commands through the STM32 0x005 API."""

from __future__ import annotations

import socket
import struct
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from .motion_permit import MotionPermit

from .protocol import (
    localization_is_accepted,
    quantize_command_speed,
    remote_takeover_detected,
    requested_api_state,
)


CAN_RAW = getattr(socket, 'CAN_RAW', 1)
SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
CAN_RAW_FILTER = 1
CAN_FRAME_STRUCT = struct.Struct('=IB3x8s')
CAN_FILTER_STRUCT = struct.Struct('=II')
HOST_COMMAND_CAN_ID = 0x005


class CmdVelCanBridge(Node):
    """Require STM32's 1 -> 2 API acknowledgement before accepting motion."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_can_bridge')
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('command_topic', '/cmd_vel_nav')
        self.declare_parameter('status_can_id', 0x507)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('status_timeout_sec', 0.25)
        self.declare_parameter('localization_timeout_sec', 2.5)
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('max_linear_speed_mps', 0.7)
        self.declare_parameter('max_angular_speed_radps', 0.5)
        self.declare_parameter('allow_reverse', True)
        self.declare_parameter('max_reverse_speed_mps', 1.0)
        self.declare_parameter('require_motion_permit', False)
        self.declare_parameter('motion_permit_timeout_sec', 0.25)
        self._interface = str(self.get_parameter('interface').value)
        self._status_can_id = int(self.get_parameter('status_can_id').value)
        self._timeout = float(self.get_parameter('command_timeout_sec').value)
        self._status_timeout = float(self.get_parameter('status_timeout_sec').value)
        self._localization_timeout = float(
            self.get_parameter('localization_timeout_sec').value)
        self._period = float(self.get_parameter('control_period_sec').value)
        self._max_linear = float(self.get_parameter('max_linear_speed_mps').value)
        self._max_angular = float(self.get_parameter('max_angular_speed_radps').value)
        self._allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self._max_reverse = float(self.get_parameter('max_reverse_speed_mps').value)
        self._require_permit = bool(self.get_parameter('require_motion_permit').value)
        self._permit = MotionPermit(float(self.get_parameter('motion_permit_timeout_sec').value))
        if not 0.0 < self._max_reverse <= 1.0:
            raise ValueError('maximum reverse speed must be within the STM32 1.0-unit API limit')
        if (self._timeout <= 0.0 or self._status_timeout <= 0.0
                or self._localization_timeout <= 0.0 or self._period <= 0.0):
            raise ValueError('command, status, and control periods must be positive')
        if not 0.0 < self._max_linear <= 1.0 or not 0.0 < self._max_angular <= 1.0:
            raise ValueError('maximum speeds must be within the STM32 1.0-unit API limit')

        self._lock = threading.Lock()
        self._command = Twist()
        self._last_command = 0.0
        self._last_status = 0.0
        self._last_localization_acceptance = 0.0
        self._api_state = 0
        self._api_was_running = False
        self._remote_takeover = False
        self._last_error = ''
        self._socket = self._open_socket()
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, '/cmd_vel_can_bridge/diagnostics', 10)
        self.create_subscription(Twist, str(self.get_parameter('command_topic').value),
                                 self._command_callback, 10)
        self.create_subscription(DiagnosticArray, '/fixed_map_localization/diagnostics',
                                 self._localization_callback, 10)
        self.create_subscription(Bool, '/mower_console/motion_permit', self._permit_callback, 1)
        self.create_timer(self._period, self._send_control_frame)
        self.create_timer(1.0, self._publish_diagnostics)
        self._receiver = threading.Thread(target=self._receive_status, daemon=True)
        self._receiver.start()

    def _open_socket(self) -> socket.socket:
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
        can_socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER,
                              CAN_FILTER_STRUCT.pack(self._status_can_id, 0x7FF))
        can_socket.bind((self._interface,))
        can_socket.settimeout(0.05)
        return can_socket

    def _command_callback(self, command: Twist) -> None:
        with self._lock:
            self._command = command
            self._last_command = time.monotonic()

    def _permit_callback(self, msg: Bool) -> None:
        with self._lock:
            self._permit.update(msg.data, time.monotonic())

    def _localization_callback(self, diagnostics: DiagnosticArray) -> None:
        for status in diagnostics.status:
            if status.name != 'fixed_map_localization':
                continue
            if localization_is_accepted(status.message):
                with self._lock:
                    self._last_localization_acceptance = time.monotonic()
            return

    def _receive_status(self) -> None:
        while rclpy.ok():
            try:
                frame = self._socket.recv(CAN_FRAME_STRUCT.size)
            except socket.timeout:
                continue
            except OSError as exc:
                self._last_error = f'CAN status receive failed: {exc}'
                return
            can_id, dlc, data = CAN_FRAME_STRUCT.unpack(frame)
            if (can_id & 0x7FF) == self._status_can_id and dlc == 8:
                with self._lock:
                    self._api_state = data[4]
                    self._last_status = time.monotonic()
                    if (remote_takeover_detected(self._api_was_running, self._api_state)
                            and not (self._require_permit and self._permit.revoked)):
                        self._remote_takeover = True
                        self._last_error = (
                            'STM32 API state dropped after API run; '
                            'stopped CAN control after remote takeover')
                    if self._api_state == 2:
                        self._api_was_running = True

    def _send_control_frame(self) -> None:
        with self._lock:
            if self._remote_takeover:
                return
            permitted = not self._require_permit or self._permit.valid(time.monotonic())
            if self._require_permit and not self._permit.armed:
                return  # Observation/startup must not emit any control CAN.
            status_fresh = time.monotonic() - self._last_status <= self._status_timeout
            localization_fresh = (
                time.monotonic() - self._last_localization_acceptance
                <= self._localization_timeout)
            reported_state = self._api_state
            state = reported_state if status_fresh else 0
            fresh = time.monotonic() - self._last_command <= self._timeout
            linear = self._command.linear.x if fresh and state == 2 and localization_fresh else 0.0
            if not self._allow_reverse:
                linear = max(0.0, linear)
            linear = max(-self._max_reverse, linear)
            angular = self._command.angular.z if fresh and state == 2 and localization_fresh else 0.0
            if not permitted:
                linear = angular = 0.0
                self._last_error = 'motion permit revoked or expired; restart required'
            api_was_running = self._api_was_running
        # The STM32 requires confirmed standby before API run, then confirmed run
        # before it may receive nonzero commands. A missing status frame must not
        # restart that handshake and demote the STM32 back to API standby.
        requested_state = requested_api_state(api_was_running, reported_state) if permitted else 0
        if not permitted and state == 0:
            return
        payload = bytes((
            requested_state,
            quantize_command_speed(linear, self._max_linear) & 0xFF,
            0,
            quantize_command_speed(angular, self._max_angular) & 0xFF,
            0, 1, 0, 0,
        ))
        try:
            self._socket.send(CAN_FRAME_STRUCT.pack(HOST_COMMAND_CAN_ID, 8, payload))
        except OSError as exc:
            self._last_error = f'CAN command send failed: {exc}'

    def _publish_diagnostics(self) -> None:
        with self._lock:
            status_age = time.monotonic() - self._last_status if self._last_status else float('inf')
            localization_age = (time.monotonic() - self._last_localization_acceptance
                                if self._last_localization_acceptance else float('inf'))
            state = self._api_state if status_age <= self._status_timeout else 0
            age = time.monotonic() - self._last_command if self._last_command else float('inf')
            remote_takeover = self._remote_takeover
        status = DiagnosticStatus()
        status.name = 'mcu_can_bridge/cmd_vel_can_bridge'
        status.level = (DiagnosticStatus.ERROR if remote_takeover else
                        (DiagnosticStatus.OK if state == 2 else DiagnosticStatus.WARN))
        status.message = ('remote takeover detected; restart bridge to request API control'
                          if remote_takeover else
                          ('api running' if state == 2 else 'awaiting STM32 API acknowledgement'))
        status.values = [KeyValue(key=key, value=value) for key, value in {
            'interface': self._interface,
            'api_state': str(state),
            'remote_takeover': str(remote_takeover).lower(),
            'command_age_sec': f'{age:.3f}',
            'status_age_sec': f'{status_age:.3f}',
            'localization_acceptance_age_sec': f'{localization_age:.3f}',
            'last_error': self._last_error,
        }.items()]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self._diagnostic_pub.publish(output)

    def destroy_node(self):
        self._socket.close()
        self._receiver.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
