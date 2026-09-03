"""Fail-closed velocity gate for the final vehicle command."""

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CollisionGate(Node):
    """Reject commands whose short predicted footprint enters local costmap."""

    def __init__(self) -> None:
        super().__init__('collision_gate')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_gated')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('robot_radius', 0.45)
        self.declare_parameter('prediction_horizon', 2.0)
        self.declare_parameter('prediction_dt', 0.05)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('costmap_timeout', 0.30)
        self.declare_parameter('odom_timeout', 0.30)
        self.declare_parameter('collision_cost', 253)
        self._radius = float(self.get_parameter('robot_radius').value)
        self._horizon = float(self.get_parameter('prediction_horizon').value)
        self._dt = float(self.get_parameter('prediction_dt').value)
        self._command_timeout = float(self.get_parameter('command_timeout').value)
        self._costmap_timeout = float(self.get_parameter('costmap_timeout').value)
        self._odom_timeout = float(self.get_parameter('odom_timeout').value)
        self._collision_cost = int(self.get_parameter('collision_cost').value)
        self._command = Twist(); self._costmap = None; self._odom = None
        self._command_time = self._costmap_time = self._odom_time = 0.0
        self._last_reason = 'awaiting command, costmap, and odometry'
        self._pub = self.create_publisher(Twist, str(self.get_parameter('output_topic').value), 10)
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, '/collision_gate/diagnostics', 10)
        self.create_subscription(Twist, str(self.get_parameter('input_topic').value), self._on_command, 10)
        self.create_subscription(Costmap, str(self.get_parameter('costmap_topic').value), self._on_costmap, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 10)
        self.create_timer(0.05, self._tick)

    def _on_command(self, msg: Twist) -> None:
        self._command = msg; self._command_time = time.monotonic()

    def _on_costmap(self, msg: Costmap) -> None:
        self._costmap = msg; self._costmap_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg; self._odom_time = time.monotonic()

    def _cell_cost(self, x: float, y: float) -> int | None:
        m = self._costmap
        if m is None: return None
        md = m.metadata; ox = md.origin.position.x; oy = md.origin.position.y
        mx = math.floor((x - ox) / md.resolution); my = math.floor((y - oy) / md.resolution)
        if mx < 0 or my < 0 or mx >= md.size_x or my >= md.size_y: return None
        return int(m.data[my * md.size_x + mx])

    def _collision(self, cmd: Twist) -> bool:
        if self._costmap is None or self._odom is None:
            self._last_reason = 'missing costmap or odometry'
            return True
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y, heading = p.x, p.y, yaw
        steps = max(1, math.ceil(self._horizon / self._dt))
        # Check the present pose too: the gate must not allow an already
        # colliding command for one extra tick. Three disk rings make this a
        # conservative circular-footprint test, rather than a sparse perimeter
        # test that can miss a small obstacle inside the vehicle envelope.
        footprint_offsets = [(0.0, 0.0)]
        for radius, angle_step in ((self._radius * 0.5, 60), (self._radius, 30)):
            footprint_offsets.extend(
                (radius * math.cos(math.radians(angle)),
                 radius * math.sin(math.radians(angle)))
                for angle in range(0, 360, angle_step))
        for step in range(steps + 1):
            for dx, dy in footprint_offsets:
                cost = self._cell_cost(x + dx, y + dy)
                if cost is None:
                    self._last_reason = 'predicted footprint leaves local costmap'
                    return True
                if cost >= self._collision_cost:
                    self._last_reason = f'predicted footprint cost {cost} >= {self._collision_cost}'
                    return True
            if step == steps:
                break
            heading += cmd.angular.z * self._dt
            x += cmd.linear.x * self._dt * math.cos(heading)
            y += cmd.linear.x * self._dt * math.sin(heading)
        self._last_reason = 'collision-free prediction'
        return False

    def _tick(self) -> None:
        now = time.monotonic()
        valid = (now - self._command_time <= self._command_timeout and
                 now - self._costmap_time <= self._costmap_timeout and
                 now - self._odom_time <= self._odom_timeout)
        out = Twist()
        if not valid:
            self._last_reason = 'stale command, costmap, or odometry'
        elif not self._collision(self._command):
            out = self._command
        self._pub.publish(out)
        status = DiagnosticStatus()
        status.name = 'mower_nav2/collision_gate'
        status.level = (DiagnosticStatus.OK if out.linear.x != 0.0 or out.angular.z != 0.0
                        else DiagnosticStatus.WARN)
        status.message = self._last_reason
        status.values = [KeyValue(key=key, value=value) for key, value in {
            'input_linear_x': f'{self._command.linear.x:.3f}',
            'input_angular_z': f'{self._command.angular.z:.3f}',
            'output_linear_x': f'{out.linear.x:.3f}',
            'output_angular_z': f'{out.angular.z:.3f}',
            'command_age_sec': f'{now - self._command_time:.3f}',
            'costmap_age_sec': f'{now - self._costmap_time:.3f}',
            'odom_age_sec': f'{now - self._odom_time:.3f}',
        }.items()]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostic_pub.publish(diagnostics)


def main(args=None) -> None:
    rclpy.init(args=args); node = CollisionGate()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
