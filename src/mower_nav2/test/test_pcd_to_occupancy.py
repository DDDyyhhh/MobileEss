import importlib.util
from pathlib import Path

import numpy as np
import yaml


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'pcd_to_occupancy.py'
SPEC = importlib.util.spec_from_file_location('pcd_to_occupancy', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def flat_ground_points() -> np.ndarray:
    x, y = np.meshgrid(np.arange(0.0, 1.1, 0.1), np.arange(0.0, 1.1, 0.1))
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))


def test_ground_plane_is_deterministic_and_upward_facing():
    plane = MODULE.fit_ground_plane(flat_ground_points())
    assert plane[2] > 0.0
    assert np.abs(plane[:3] @ np.array([0.0, 0.0, 0.0]) + plane[3]) < 1e-8


def test_ground_plane_rejects_a_denser_vertical_wall():
    ground = flat_ground_points()
    wall_y, wall_z = np.meshgrid(np.arange(0.0, 2.1, 0.1), np.arange(0.0, 2.1, 0.1))
    wall = np.column_stack((np.full(wall_y.size, 0.5), wall_y.ravel(), wall_z.ravel()))
    plane = MODULE.fit_ground_plane(np.vstack((ground, wall)))
    assert plane[2] > 0.90


def test_ground_plane_does_not_choose_a_higher_parallel_ceiling():
    ground = flat_ground_points()
    ceiling = ground + np.array([0.0, 0.0, 5.0])
    plane = MODULE.fit_ground_plane(np.vstack((ground, ceiling)))
    assert abs(plane[3]) < 1e-8


def test_overhead_returns_are_excluded_from_obstacle_support():
    points = np.vstack((
        flat_ground_points(),
        np.tile([0.5, 0.5, 1.2], (5, 1)),
    ))
    image, _, _ = MODULE.occupancy_image(
        points, resolution=0.1, padding=0.0, obstacle_height=0.15,
        min_obstacle_points=5, z_min=0.15, z_max=0.80)
    assert np.count_nonzero(image == 0) == 0


def test_returns_inside_the_plane_relative_height_band_are_obstacles():
    points = np.vstack((
        flat_ground_points() + np.array([0.0, 0.0, 5.0]),
        np.tile([0.5, 0.5, 5.5], (5, 1)),
    ))
    image, _, _ = MODULE.occupancy_image(
        points, resolution=0.1, padding=0.0, obstacle_height=0.15,
        min_obstacle_points=5, z_min=0.15, z_max=0.80)
    assert np.count_nonzero(image == 0) == 1


def test_sparse_returns_do_not_block_a_ground_cell():
    points = np.vstack((
        flat_ground_points(),
        np.tile([0.5, 0.5, 0.5], (4, 1)),
    ))
    image, _, _ = MODULE.occupancy_image(
        points, resolution=0.1, padding=0.0, obstacle_height=0.15,
        min_obstacle_points=5, z_min=0.15, z_max=0.80)
    assert np.count_nonzero(image == 0) == 0


def test_checked_in_map_keeps_the_map_capture_start_clear_of_static_obstacles():
    root = Path(__file__).resolve().parents[1]
    metadata = yaml.safe_load((root / 'maps' / 'map.yaml').read_text())
    raw = (root / 'maps' / metadata['image']).read_bytes()
    header, pixels = raw.split(b'255\n', 1)
    width, height = map(int, header.splitlines()[1].split())
    image = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width)
    origin_x, origin_y, _ = metadata['origin']
    resolution = metadata['resolution']
    row = height - 1 - int((0.0 - origin_y) / resolution)
    column = int((0.0 - origin_x) / resolution)
    occupied_rows, occupied_columns = np.nonzero(image == 0)
    clearance = np.hypot(occupied_rows - row, occupied_columns - column).min() * resolution
    assert image[row, column] == 254
    assert clearance > 0.45


def test_unobserved_cells_remain_unknown():
    image, _, _ = MODULE.occupancy_image(
        flat_ground_points(), resolution=0.1, padding=0.2, obstacle_height=0.15)
    assert np.count_nonzero(image == 205) > 0
