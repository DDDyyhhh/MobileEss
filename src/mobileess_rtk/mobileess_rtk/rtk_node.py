"""ROS 2 node bridging UM982 NMEA and NTRIP RTCM."""
import os
import threading
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from .nmea import GgaFix, parse_gga
from .ntrip import NtripClient
from .serial_io import SerialPort


class RtkNode(Node):
    def __init__(self) -> None:
        super().__init__('um982_rtk')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('frame_id', 'rtk_link')
        self.declare_parameter('fix_topic', '/rtk/fix')
        self.declare_parameter('diagnostics_topic', '/rtk/diagnostics')
        self.declare_parameter('ntrip_enabled', False)
        self.declare_parameter('ntrip_host', 'caster.example.com')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', 'MOUNTPOINT')
        self.declare_parameter('ntrip_username_env', 'NTRIP_USERNAME')
        self.declare_parameter('ntrip_password_env', 'NTRIP_PASSWORD')
        self.declare_parameter('ntrip_send_gga', True)
        self.declare_parameter('ntrip_reconnect_sec', 5.0)
        self.declare_parameter('ntrip_gga_interval_sec', 10.0)

        self._fix_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter('fix_topic').value), 10)
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter('diagnostics_topic').value), 10)
        self._lock = threading.Lock()
        self._latest_gga: Optional[str] = None
        self._last_error = ''
        self._serial_connected = False
        self._ntrip_connected = False
        self._nmea_lines = 0
        self._fix_count = 0
        self._rtcm_bytes = 0
        self._stop = threading.Event()
        self._serial: Optional[SerialPort] = None
        self._ntrip: Optional[NtripClient] = None
        self._last_gga_sent: Optional[str] = None
        self._last_gga_send_time = 0.0

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)

    def _run(self) -> None:
        try:
            self._serial = SerialPort(
                str(self.get_parameter('serial_port').value),
                int(self.get_parameter('serial_baud').value),
            )
            self._serial_connected = True
            self.get_logger().info('Opened UM982 serial port')
        except Exception as exc:
            if not self._stop.is_set():
                self._record_error(f'serial open failed: {exc}')
            return

        while not self._stop.is_set():
            try:
                raw = self._serial.readline()
                if raw:
                    self._handle_line(raw)
                if bool(self.get_parameter('ntrip_enabled').value):
                    self._pump_ntrip()
            except Exception as exc:
                if self._stop.is_set():
                    break
                self._record_error(str(exc))
                self._stop.wait(float(self.get_parameter('ntrip_reconnect_sec').value))

    def _handle_line(self, raw: bytes) -> None:
        line = raw.decode('ascii', errors='replace').strip()
        with self._lock:
            self._nmea_lines += 1
        fix = parse_gga(line)
        if fix is None:
            return
        with self._lock:
            self._latest_gga = fix.raw
            self._fix_count += 1
        if self._ntrip_connected and self._ntrip is not None:
            now = time.monotonic()
            interval = float(self.get_parameter('ntrip_gga_interval_sec').value)
            if (self._last_gga_sent is None
                    or now - self._last_gga_send_time >= interval):
                try:
                    self._ntrip.send_gga(fix.raw)
                    self._last_gga_sent = fix.raw
                    self._last_gga_send_time = now
                except Exception as exc:
                    if not self._stop.is_set():
                        self._record_error(f'NTRIP GGA send error: {exc}')
        self._publish_fix(fix)

    def _publish_fix(self, fix: GgaFix) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.latitude = fix.latitude
        msg.longitude = fix.longitude
        msg.altitude = fix.altitude
        msg.status.status = self._status_for_quality(fix.quality)
        msg.status.service = msg.status.SERVICE_GPS
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._fix_pub.publish(msg)

    @staticmethod
    def _status_for_quality(quality: int) -> int:
        if quality == 0:
            return -1  # sensor_msgs/NavSatStatus.STATUS_NO_FIX
        if quality in (2, 4, 5):
            return 1  # STATUS_GBAS_FIX, useful for RTK-equivalent diagnostics
        return 0  # STATUS_FIX

    def _pump_ntrip(self) -> None:
        if self._ntrip is None:
            username = os.environ.get(str(self.get_parameter('ntrip_username_env').value), '')
            password = os.environ.get(str(self.get_parameter('ntrip_password_env').value), '')
            if not username or not password:
                self._record_error('NTRIP credentials are missing from environment')
                self._stop.wait(float(self.get_parameter('ntrip_reconnect_sec').value))
                return
            self._ntrip = NtripClient(
                str(self.get_parameter('ntrip_host').value),
                int(self.get_parameter('ntrip_port').value),
                str(self.get_parameter('ntrip_mountpoint').value),
                username,
                password,
            )
        try:
            gga = self._latest_gga if bool(self.get_parameter('ntrip_send_gga').value) else None
            if bool(self.get_parameter('ntrip_send_gga').value) and gga is None:
                return
            if not self._ntrip_connected:
                self._ntrip.connect(gga)
                self._ntrip_connected = True
                if gga:
                    self._last_gga_sent = gga
                    self._last_gga_send_time = time.monotonic()
                self.get_logger().info('Connected to NTRIP caster')
            data = self._ntrip.read()
            if data is None:
                return
            if not data:
                raise ConnectionError('NTRIP caster closed the connection')
            if self._serial is not None:
                self._serial.write(data)
                with self._lock:
                    self._rtcm_bytes += len(data)
        except Exception as exc:
            self._ntrip_connected = False
            if not self._stop.is_set():
                self._record_error(f'NTRIP error: {exc}')
            if self._ntrip is not None:
                self._ntrip.close()
                self._last_gga_sent = None
            self._stop.wait(float(self.get_parameter('ntrip_reconnect_sec').value))

    def _record_error(self, error: str) -> None:
        with self._lock:
            self._last_error = error
        self.get_logger().error(error)

    def _publish_diagnostics(self) -> None:
        with self._lock:
            values = {
                'serial_connected': str(self._serial_connected).lower(),
                'ntrip_connected': str(self._ntrip_connected).lower(),
                'nmea_lines': str(self._nmea_lines),
                'fix_messages': str(self._fix_count),
                'rtcm_bytes_forwarded': str(self._rtcm_bytes),
                'last_error': self._last_error,
            }
        status = DiagnosticStatus()
        status.name = 'mobileess_rtk/um982'
        status.level = DiagnosticStatus.OK if self._serial_connected else DiagnosticStatus.ERROR
        status.message = 'running' if self._serial_connected else 'serial unavailable'
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        self._diagnostic_pub.publish(DiagnosticArray(
            header=Header(stamp=self.get_clock().now().to_msg()),
            status=[status],
        ))

    def destroy_node(self):
        self._stop.set()
        if self._ntrip is not None:
            self._ntrip.close()
        if self._serial is not None:
            self._serial.close()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
