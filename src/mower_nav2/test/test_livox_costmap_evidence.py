from dataclasses import dataclass

from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg
from mower_nav2.livox_costmap_evidence import filter_livox_costmap_evidence
from mower_nav2.livox_obstacle_cloud import LivoxObstacleCloud


@dataclass
class Point:
    x: float
    y: float
    z: float


def test_ground_points_clear_but_do_not_mark_obstacles():
    ground = [
        Point(x, y, -0.48)
        for x in (0.8, 1.0, 1.2, 1.4, 1.6)
        for y in (-0.2, 0.0, 0.2)
    ]
    marks, clears = filter_livox_costmap_evidence(ground, point_stride=1)
    assert marks == []
    assert len(clears) == len(ground)


def test_tilted_ground_and_clustered_above_ground_object_are_separated():
    # The ground plane is 0.48 m below the sensor and slopes in the sensor
    # frame, as it does while the chassis is pitched during a turn or bump.
    ground = [
        Point(x, y, -0.48 - 0.06 * x + 0.03 * y)
        for x in (0.8, 1.0, 1.2, 1.4, 1.6, 1.8)
        for y in (-0.3, -0.1, 0.1, 0.3)
    ]
    object_points = [
        Point(1.24 + dx, 0.04 + dy, -0.48 - 0.06 * (1.24 + dx)
              + 0.03 * (0.04 + dy) + 0.45)
        for dx in (-0.02, 0.00, 0.02)
        for dy in (-0.02, 0.02)
    ]
    marks, clears = filter_livox_costmap_evidence(
        ground + object_points, point_stride=1, min_points_per_cell=3)
    assert len(marks) == len(object_points)
    assert len(clears) == len(ground)


def test_nav2_evidence_can_use_receipt_stamp_without_changing_source_message():
    source = CustomMsg()
    source.header.frame_id = 'livox_frame'
    source.header.stamp.sec = 10
    source.header.stamp.nanosec = 20
    receipt_stamp = Time(sec=11, nanosec=30)
    cloud = LivoxObstacleCloud._to_cloud(source, [], receipt_stamp)
    assert (cloud.header.stamp.sec, cloud.header.stamp.nanosec) == (11, 30)
    assert (source.header.stamp.sec, source.header.stamp.nanosec) == (10, 20)
    assert cloud.header.frame_id == 'livox_frame'


def test_isolated_above_ground_return_is_not_an_obstacle_and_zero_returns_are_dropped():
    ground = [
        Point(x, y, -0.48)
        for x in (0.8, 1.0, 1.2, 1.4, 1.6)
        for y in (-0.2, 0.0, 0.2)
    ]
    marks, clears = filter_livox_costmap_evidence(
        ground + [Point(1.2, 0.0, -0.03), Point(0.0, 0.0, 0.0)],
        point_stride=1,
        min_points_per_cell=3)
    assert marks == []
    assert len(clears) == len(ground)
