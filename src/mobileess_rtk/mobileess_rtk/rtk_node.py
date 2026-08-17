"""ROS 2 node bridging UM982 NMEA and NTRIP RTCM."""
import os
import threading
import time
import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from .nmea import GgaFix, HeadingFix, parse_gga, parse_uniheading
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
        self.declare_parameter('heading_topic', '/rtk/heading')
        self.declare_parameter('heading_frame_id', 'rtk_link')
        # UNIHEADING is the Ant1 -> Ant2 bearing from true north. With Ant1 on
        # the right and Ant2 on the left, that baseline is vehicle +Y (left),
        # so ROS base_link yaw is the negative bearing.
        self.declare_parameter('heading_to_base_yaw_offset_rad', 0.0)
        self.declare_parameter('heading_max_stddev_deg', 2.0)
        self.declare_parameter('accepted_fix_qualities', [4])
        self.declare_parameter('horizontal_accuracy_m', 0.05)
        self.declare_parameter('vertical_accuracy_m', 0.10)
        self.declare_parameter('ntrip_enabled', False)
        self.declare_parameter('ntrip_host', 'caster.example.com')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', 'MOUNTPOINT')
        self.declare_parameter('ntrip_username_env', 'NTRIP_USERNAME')
        self.declare_parameter('ntrip_password_env', 'NTRIP_PASSWORD')
        self.declare_parameter('ntrip_send_gga', True)
        self.declare_parameter('ntrip_reconnect_sec', 5.0)
        self.declare_parameter('ntrip_gga_interval_sec', 10.0)
        self.declare_parameter('ntrip_rtcm_timeout_sec', 3.0)

        self._fix_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter('fix_topic').value), 10)
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter('diagnostics_topic').value), 10)
        self._heading_pub = self.create_publisher(
            Imu, str(self.get_parameter('heading_topic').value), 10)
        self._lock = threading.Lock()
        self._latest_gga: Optional[str] = None
        self._last_error = ''
        self._serial_connected = False
        self._ntrip_connected = False
        self._nmea_lines = 0
        self._fix_count = 0
        self._published_fix_count = 0
        self._rejected_fix_count = 0
        self._last_quality = 0
        self._last_hdop = 0.0
        self._rtcm_bytes = 0
        self._ntrip_reconnects = 0
        self._ntrip_session_bytes = 0
        self._ntrip_connected_time = 0.0
        self._last_rtcm_time = 0.0
        self._last_ntrip_session_sec = 0.0
        self._heading_count = 0
        self._published_heading_count = 0
        self._last_heading_deg = 0.0
        self._last_heading_stddev_deg = 0.0
        self._last_heading_time = 0.0
        self._stop = threading.Event()
        self._serial: Optional[SerialPort] = None
        self._ntrip: Optional[NtripClient] = None
        self._last_gga_sent: Optional[str] = None
        self._last_gga_send_time = 0.0

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                if self._serial is None:
                    self._serial = SerialPort(
                        str(self.get_parameter('serial_port').value),
                        int(self.get_parameter('serial_baud').value),
                    )
                    self._serial_connected = True
                    self.get_logger().info('Opened UM982 serial port')
                raw = self._serial.readline()
                if raw:
                    self._handle_line(raw)
                if bool(self.get_parameter('ntrip_enabled').value):
                    self._pump_ntrip()
            except Exception as exc:
                if self._stop.is_set():
                    break
                self._serial_connected = False
                if self._serial is not None:
                    self._serial.close()
                    self._serial = None
                if self._ntrip is not None:
                    self._ntrip.close()
                self._ntrip_connected = False
                self._ntrip_connected_time = 0.0
                self._record_error(str(exc))
                self._stop.wait(float(self.get_parameter('ntrip_reconnect_sec').value))

    def _handle_line(self, raw: bytes) -> None:
        line = raw.decode('ascii', errors='replace').strip()
        with self._lock:
            self._nmea_lines += 1
        heading = parse_uniheading(line)
        if heading is not None:
            self._handle_heading(heading)
        fix = parse_gga(line)
        if fix is None:
            return
        with self._lock:
            self._latest_gga = fix.raw
            self._fix_count += 1
            self._last_quality = fix.quality
            self._last_hdop = fix.hdop
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

    def _handle_heading(self, fix: HeadingFix) -> None:
        with self._lock:
            self._heading_count += 1
            self._last_heading_deg = fix.heading_deg
            self._last_heading_stddev_deg = fix.heading_stddev_deg
            self._last_heading_time = time.monotonic()
        if fix.solution_status != 'SOL_COMPUTED':
            return
        if fix.baseline_m <= 0.0:
            return
        max_stddev = float(self.get_parameter('heading_max_stddev_deg').value)
        if not math.isfinite(fix.heading_stddev_deg) or fix.heading_stddev_deg > max_stddev:
            return
        offset = float(self.get_parameter('heading_to_base_yaw_offset_rad').value)
        yaw = offset - math.radians(fix.heading_deg)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('heading_frame_id').value)
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        variance = math.radians(fix.heading_stddev_deg) ** 2
        msg.orientation_covariance[0] = -1.0
        msg.orientation_covariance[4] = -1.0
        msg.orientation_covariance[8] = variance
        self._heading_pub.publish(msg)
        with self._lock:
            self._published_heading_count += 1

    def _publish_fix(self, fix: GgaFix) -> None:
        accepted_qualities = set(
            int(quality)
            for quality in self.get_parameter('accepted_fix_qualities').value
        )
        if fix.quality not in accepted_qualities:
            with self._lock:
                self._rejected_fix_count += 1
            return

        horizontal_accuracy = float(self.get_parameter('horizontal_accuracy_m').value)
        vertical_accuracy = float(self.get_parameter('vertical_accuracy_m').value)
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.latitude = fix.latitude
        msg.longitude = fix.longitude
        msg.altitude = fix.altitude
        msg.status.status = self._status_for_quality(fix.quality)
        msg.status.service = msg.status.SERVICE_GPS
        msg.position_covariance[0] = horizontal_accuracy ** 2
        msg.position_covariance[4] = horizontal_accuracy ** 2
        msg.position_covariance[8] = vertical_accuracy ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(msg)
        with self._lock:
            self._published_fix_count += 1

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
                self._ntrip_reconnects += 1
                self._ntrip_session_bytes = 0
                self._ntrip_connected_time = time.monotonic()
                self._last_rtcm_time = self._ntrip_connected_time
                with self._lock:
                    self._last_error = ''
                if gga:
                    self._last_gga_sent = gga
                    self._last_gga_send_time = time.monotonic()
                self.get_logger().info('Connected to NTRIP caster')
            data = self._ntrip.read()
            if data is None:
                if time.monotonic() - self._last_rtcm_time > float(
                        self.get_parameter('ntrip_rtcm_timeout_sec').value):
                    raise ConnectionError('NTRIP correction stream timed out')
                return
            if not data:
                raise ConnectionError('NTRIP caster closed the connection')
            if self._serial is not None:
                self._serial.write(data)
                with self._lock:
                    self._rtcm_bytes += len(data)
                    self._ntrip_session_bytes += len(data)
                self._last_rtcm_time = time.monotonic()
        except Exception as exc:
            self._ntrip_connected = False
            if self._ntrip_connected_time:
                self._last_ntrip_session_sec = time.monotonic() - self._ntrip_connected_time
            self._ntrip_connected_time = 0.0
            self._last_rtcm_time = 0.0
            if not self._stop.is_set():
                self._record_error(f'NTRIP error: {exc}')
            if self._ntrip is not None:
                self._ntrip.close()
                self._ntrip = None
                self._last_gga_sent = None
            self._stop.wait(float(self.get_parameter('ntrip_reconnect_sec').value))

    def _record_error(self, error: str) -> None:
        with self._lock:
            self._last_error = error
        self.get_logger().error(error)

    def _publish_diagnostics(self) -> None:
        with self._lock:
            ntrip_session_sec = (
                time.monotonic() - self._ntrip_connected_time
                if self._ntrip_connected_time else self._last_ntrip_session_sec
            )
            ntrip_rtcm_age_sec = (
                'inf' if self._last_rtcm_time == 0.0 else
                f'{max(0.0, time.monotonic() - self._last_rtcm_time):.3f}'
            )
            heading_age = (
                'inf' if self._last_heading_time == 0.0 else
                f'{max(0.0, time.monotonic() - self._last_heading_time):.3f}'
            )
            values = {
                'serial_connected': str(self._serial_connected).lower(),
                'ntrip_connected': str(self._ntrip_connected).lower(),
                'nmea_lines': str(self._nmea_lines),
                'fix_messages': str(self._fix_count),
                'published_fixes': str(self._published_fix_count),
                'rejected_fixes': str(self._rejected_fix_count),
                'last_fix_quality': str(self._last_quality),
                'last_hdop': str(self._last_hdop),
                'rtcm_bytes_forwarded': str(self._rtcm_bytes),
                'ntrip_reconnects': str(self._ntrip_reconnects),
                'ntrip_session_rtcm_bytes': str(self._ntrip_session_bytes),
                'ntrip_session_sec': f'{ntrip_session_sec:.3f}',
                'ntrip_rtcm_age_sec': ntrip_rtcm_age_sec,
                'heading_messages': str(self._heading_count),
                'published_headings': str(self._published_heading_count),
                'last_heading_deg': str(self._last_heading_deg),
                'last_heading_stddev_deg': str(self._last_heading_stddev_deg),
                'heading_age_sec': heading_age,
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
