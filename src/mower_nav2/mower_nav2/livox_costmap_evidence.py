"""Classify Livox returns into obstacle marks and ground clearing evidence."""

from __future__ import annotations

from collections import defaultdict
from math import floor, hypot, isfinite


DEFAULT_GROUND_Z = -0.48


def is_inside_self_filter_box(point, boxes: list[float]) -> bool:
    """Return whether a point is inside one configured vehicle box."""
    for offset in range(0, len(boxes), 6):
        min_x, max_x, min_y, max_y, min_z, max_z = boxes[offset:offset + 6]
        if (min_x <= point.x <= max_x and min_y <= point.y <= max_y
                and min_z <= point.z <= max_z):
            return True
    return False


def _solve_plane(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    """Fit z = ax + by + c using a small, dependency-free least-squares solve."""
    if len(points) < 3:
        return 0.0, 0.0, DEFAULT_GROUND_Z
    sums = [0.0] * 6
    rhs = [0.0] * 3
    for x, y, z in points:
        sums[0] += x * x
        sums[1] += x * y
        sums[2] += y * y
        sums[3] += x
        sums[4] += y
        sums[5] += 1.0
        rhs[0] += x * z
        rhs[1] += y * z
        rhs[2] += z
    matrix = [
        [sums[0], sums[1], sums[3]],
        [sums[1], sums[2], sums[4]],
        [sums[3], sums[4], sums[5]],
    ]
    # Gaussian elimination with partial pivoting.
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(matrix[row][column]))
        if abs(matrix[pivot][column]) < 1e-9:
            return 0.0, 0.0, DEFAULT_GROUND_Z
        matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
        divisor = matrix[column][column]
        for j in range(column, 3):
            matrix[column][j] /= divisor
        rhs[column] /= divisor
        for row in range(3):
            if row == column:
                continue
            factor = matrix[row][column]
            for j in range(column, 3):
                matrix[row][j] -= factor * matrix[column][j]
            rhs[row] -= factor * rhs[column]
    return rhs[0], rhs[1], rhs[2]


def _fit_ground_plane(points: list[object], cell_size: float,
                      ground_band: float) -> tuple[float, float, float]:
    """Fit a robust local plane from the lowest return in each XY cell."""
    cells: dict[tuple[int, int], object] = {}
    for point in points:
        key = (floor(point.x / cell_size), floor(point.y / cell_size))
        if key not in cells or point.z < cells[key].z:
            cells[key] = point
    candidates = [(p.x, p.y, p.z) for p in cells.values()]
    if len(candidates) < 3:
        return 0.0, 0.0, DEFAULT_GROUND_Z
    plane = _solve_plane(candidates)
    for _ in range(2):
        inliers = [p for p in candidates
                   if abs(p[2] - (plane[0] * p[0] + plane[1] * p[1] + plane[2]))
                   <= ground_band]
        if len(inliers) < 3:
            break
        plane = _solve_plane(inliers)
    return plane


def filter_livox_costmap_evidence(
    points: list[object],
    *,
    point_stride: int = 4,
    min_range: float = 0.20,
    max_range: float = 4.0,
    self_filter_boxes: list[float] | None = None,
    ground_cell_size: float = 0.20,
    ground_band: float = 0.10,
    obstacle_min_height: float = 0.15,
    obstacle_max_height: float = 1.50,
    min_points_per_cell: int = 3,
    obstacle_cell_size: float = 0.10,
) -> tuple[list[object], list[object]]:
    """Return ``(obstacle_marks, ground_clearing_points)``.

    The function deliberately emits no mark for an isolated above-ground return.
    A ground endpoint is emitted only for a return close to the fitted local
    ground plane, allowing Nav2's clearing ray to erase stale local cells.
    """
    if point_stride < 1 or min_points_per_cell < 1:
        raise ValueError('point_stride and min_points_per_cell must be positive')
    if max_range <= min_range or ground_cell_size <= 0.0 or obstacle_cell_size <= 0.0:
        raise ValueError('invalid range or cell-size parameters')
    boxes = self_filter_boxes or []
    if len(boxes) % 6 != 0:
        raise ValueError('self_filter_boxes must contain six values per box')

    retained = []
    for index, point in enumerate(points):
        if index % point_stride != 0 or is_inside_self_filter_box(point, boxes):
            continue
        if not (isfinite(point.x) and isfinite(point.y) and isfinite(point.z)):
            continue
        if not min_range <= hypot(point.x, point.y) <= max_range:
            continue
        # Livox uses zero XYZ for invalid returns in CustomMsg streams.
        if point.x == 0.0 and point.y == 0.0 and point.z == 0.0:
            continue
        retained.append(point)
    if len(retained) < 3:
        return [], []

    plane = _fit_ground_plane(retained, ground_cell_size, ground_band)

    def residual(point: object) -> float:
        return point.z - (plane[0] * point.x + plane[1] * point.y + plane[2])

    marks_by_cell: dict[tuple[int, int], list[object]] = defaultdict(list)
    clears = []
    for point in retained:
        height = residual(point)
        if abs(height) <= ground_band:
            clears.append(point)
        elif obstacle_min_height <= height <= obstacle_max_height:
            key = (floor(point.x / obstacle_cell_size),
                   floor(point.y / obstacle_cell_size))
            marks_by_cell[key].append(point)

    marks = [point for cell in marks_by_cell.values()
             if len(cell) >= min_points_per_cell for point in cell]
    return marks, clears
