"""Deterministic differential model for FAST-LIO AVIA feature-disabled preprocessing.

This is deliberately a small behavioral model, not a replacement implementation.  It
pins the meaningful difference between the imported ROS 2 fork and the current
handler so real CustomMsg samples can be classified before changing field code.
"""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float
    line: int
    tag: int
    offset_time: int = 0


def is_valid_livox_point(point: Point, scan_lines: int) -> bool:
    return point.line < scan_lines and (point.tag & 0x30) in (0x10, 0x00)


def outside_blind(point: Point, blind: float) -> bool:
    return point.x * point.x + point.y * point.y + point.z * point.z > blind * blind


def baseline_selected_indices(points: list[Point], scan_lines: int, blind: float, point_filter_num: int) -> list[int]:
    """Model the imported fork's AVIA feature-disabled selection exactly."""
    selected: list[int] = []
    valid_num = 0
    for index in range(1, len(points)):
        point = points[index]
        if not is_valid_livox_point(point, scan_lines):
            continue
        valid_num += 1
        if valid_num % point_filter_num != 0:
            continue
        previous = points[index - 1]
        is_new = (
            abs(point.x - previous.x) > 1e-7
            or abs(point.y - previous.y) > 1e-7
            or abs(point.z - previous.z) > 1e-7
        )
        if is_new and outside_blind(point, blind):
            selected.append(index)
    return selected


def current_selected_indices(points: list[Point], scan_lines: int, blind: float, point_filter_num: int) -> list[int]:
    """Model the current AVIA feature-disabled selection exactly."""
    selected: list[int] = []
    valid_num = 0
    point_filter_num = max(point_filter_num, 1)
    for index, point in enumerate(points):
        if not is_valid_livox_point(point, scan_lines):
            continue
        if valid_num % point_filter_num == 0 and outside_blind(point, blind):
            selected.append(index)
        valid_num += 1
    return selected


def finite_xyz_count(points: list[Point]) -> int:
    return sum(isfinite(point.x) and isfinite(point.y) and isfinite(point.z) for point in points)


def test_current_handler_preserves_the_first_valid_nonblind_point():
    points = [
        Point(1.0, 0.0, 0.0, 0, 0x10),
        Point(2.0, 0.0, 0.0, 0, 0x10),
    ]

    assert baseline_selected_indices(points, scan_lines=4, blind=0.1, point_filter_num=1) == [1]
    assert current_selected_indices(points, scan_lines=4, blind=0.1, point_filter_num=1) == [0, 1]


def test_current_handler_keeps_duplicate_points_that_baseline_removes():
    points = [
        Point(1.0, 0.0, 0.0, 0, 0x10),
        Point(1.0, 0.0, 0.0, 0, 0x10),
        Point(2.0, 0.0, 0.0, 1, 0x00),
    ]

    assert baseline_selected_indices(points, scan_lines=4, blind=0.1, point_filter_num=1) == [2]
    assert current_selected_indices(points, scan_lines=4, blind=0.1, point_filter_num=1) == [0, 1, 2]


def test_filter_phase_differs_without_changing_validity_or_blind_rules():
    points = [
        Point(1.0, 0.0, 0.0, 0, 0x10),  # retained only by current handler
        Point(1.0, 0.0, 0.0, 7, 0x10),  # invalid line
        Point(2.0, 0.0, 0.0, 0, 0x20),  # invalid tag
        Point(3.0, 0.0, 0.0, 2, 0x10),
        Point(4.0, 0.0, 0.0, 3, 0x00),
        Point(5.0, 0.0, 0.0, 0, 0x10),
    ]

    assert baseline_selected_indices(points, scan_lines=4, blind=0.1, point_filter_num=2) == [4]
    assert current_selected_indices(points, scan_lines=4, blind=0.1, point_filter_num=2) == [0, 4]
    assert finite_xyz_count(points) == len(points)
