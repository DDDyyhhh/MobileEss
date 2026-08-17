#!/usr/bin/env python3
"""Convert an XYZ/XYZI PCD into an inspectable Nav2 occupancy map."""

import argparse
from pathlib import Path

import numpy as np


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Read x/y/z fields from an ASCII or binary PCD without requiring PCL."""
    with path.open('rb') as stream:
        header = b''
        while b'DATA ' not in header:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header has no DATA line')
            header += line
        fields = {}
        for line in header.decode('ascii').splitlines():
            key, *values = line.split()
            fields[key.upper()] = values
        data_type = fields['DATA'][0].lower()
        names = fields['FIELDS']
        sizes = [int(value) for value in fields['SIZE']]
        types = fields['TYPE']
        counts = [int(value) for value in fields.get('COUNT', ['1'] * len(names))]
        points = int(fields['POINTS'][0])
        if data_type == 'ascii':
            rows = np.loadtxt(stream, dtype=np.float64, max_rows=points)
            return rows[:, [names.index('x'), names.index('y'), names.index('z')]]
        if data_type != 'binary' or any(count != 1 for count in counts):
            raise ValueError(f'unsupported PCD DATA format: {data_type}')
        type_codes = {('F', 4): 'f4', ('F', 8): 'f8', ('I', 1): 'i1', ('I', 2): 'i2',
                      ('I', 4): 'i4', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4'}
        dtype = np.dtype([(name, '<' + type_codes[(kind, size)])
                          for name, kind, size in zip(names, types, sizes)])
        data = np.frombuffer(stream.read(points * dtype.itemsize), dtype=dtype, count=points)
        return np.column_stack((data['x'], data['y'], data['z'])).astype(np.float64)


def fit_ground_plane(points: np.ndarray, distance_threshold: float = 0.08,
                     num_iterations: int = 1000,
                     seed: int = 0, min_ground_normal_z: float = 0.90,
                     ground_seed_percentile: float = 25.0
                     ) -> np.ndarray:
    """Fit a ground plane with deterministic RANSAC and least-squares refinement."""
    if (distance_threshold <= 0.0 or num_iterations < 1 or
            not 0.0 < min_ground_normal_z <= 1.0 or
            not 0.0 < ground_seed_percentile <= 100.0):
        raise ValueError('invalid RANSAC ground-plane parameters')
    if len(points) < 3:
        raise ValueError('at least three points are required to fit a ground plane')

    seed_points = points[points[:, 2] <= np.percentile(points[:, 2], ground_seed_percentile)]
    if len(seed_points) < 3:
        raise ValueError('ground-plane seed contains fewer than three points')
    rng = np.random.default_rng(seed)
    best_inliers = None
    best_count = 0
    for _ in range(num_iterations):
        sample = seed_points[rng.choice(len(seed_points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if abs(normal[2]) < min_ground_normal_z:
            continue
        offset = -float(np.dot(normal, sample[0]))
        distances = np.abs(seed_points @ normal + offset)
        inliers = distances <= distance_threshold
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
    if best_inliers is None or best_count < 3:
        raise ValueError('RANSAC could not find a ground plane')

    inlier_points = seed_points[best_inliers]
    centered = inlier_points - inlier_points.mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, inlier_points.mean(axis=0)))
    return np.array([normal[0], normal[1], normal[2], offset])


def occupancy_image(points: np.ndarray, resolution: float, padding: float,
                    obstacle_height: float, min_obstacle_points: int = 5,
                    z_min: float = 0.15, z_max: float = 0.80,
                    plane_distance_threshold: float = 0.08,
                    plane_iterations: int = 1000,
                    plane_seed: int = 0, min_ground_normal_z: float = 0.90,
                    ground_seed_percentile: float = 25.0
                    ) -> tuple[np.ndarray, float, float]:
    """Project returns in a ground-relative height band into a 2D map."""
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise ValueError('PCD contains no finite XYZ points')
    if z_min < 0.0 or z_max <= z_min:
        raise ValueError('z-max must exceed non-negative z-min')
    if min_obstacle_points < 1:
        raise ValueError('min_obstacle_points must be positive')
    plane = fit_ground_plane(
        points, plane_distance_threshold, plane_iterations, plane_seed, min_ground_normal_z,
        ground_seed_percentile)
    heights = points @ plane[:3] + plane[3]
    minimum = points[:, :2].min(axis=0) - padding
    maximum = points[:, :2].max(axis=0) + padding
    width, height = np.maximum(
        1, np.ceil((maximum - minimum) / resolution).astype(int))
    columns = np.clip(((points[:, 0] - minimum[0]) / resolution).astype(int), 0, width - 1)
    rows = np.clip(((points[:, 1] - minimum[1]) / resolution).astype(int), 0, height - 1)
    flat = rows * width + columns
    observed_2d = np.zeros(width * height, dtype=bool)
    observed_2d[flat] = True
    observed_2d = observed_2d.reshape((height, width))
    image = np.full(width * height, 205, dtype=np.uint8)
    image_2d = image.reshape((height, width))
    obstacle_hits = ((heights >= obstacle_height) & (heights >= z_min) &
                     (heights <= z_max))
    obstacle_point_count = np.zeros(width * height, dtype=np.int32)
    np.add.at(obstacle_point_count, flat[obstacle_hits], 1)
    is_obstacle = obstacle_point_count.reshape((height, width)) >= min_obstacle_points
    is_ground = observed_2d & ~is_obstacle
    image_2d[is_ground] = 254
    image_2d[is_obstacle] = 0
    return image_2d[::-1], float(minimum[0]), float(minimum[1])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_pcd', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--padding', type=float, default=1.0)
    parser.add_argument('--obstacle-height', type=float, default=0.15)
    parser.add_argument(
        '--z-min', type=float, default=0.15,
        help='Minimum height above the RANSAC ground plane, in metres.')
    parser.add_argument(
        '--z-max', type=float, default=0.80,
        help='Maximum height above the RANSAC ground plane, in metres.')
    parser.add_argument('--plane-distance-threshold', type=float, default=0.08)
    parser.add_argument('--plane-iterations', type=int, default=1000)
    parser.add_argument('--plane-seed', type=int, default=0)
    parser.add_argument(
        '--min-ground-normal-z', type=float, default=0.90,
        help='Minimum absolute map-frame Z component accepted for the ground normal.')
    parser.add_argument(
        '--ground-seed-percentile', type=float, default=25.0,
        help='Lowest map-frame Z percentile used to sample RANSAC ground candidates.')
    parser.add_argument(
        '--min-obstacle-points', type=int, default=15,
        help='Minimum above-ground PCD returns required to mark a cell occupied.')
    args = parser.parse_args()
    if (args.resolution <= 0.0 or args.padding < 0.0 or args.obstacle_height <= 0.0
            or args.min_obstacle_points < 1 or args.z_min < 0.0 or args.z_max <= args.z_min
            or args.plane_distance_threshold <= 0.0 or args.plane_iterations < 1
            or not 0.0 < args.min_ground_normal_z <= 1.0
            or not 0.0 < args.ground_seed_percentile <= 100.0):
        parser.error(
            'resolution, obstacle-height, and min-obstacle-points must be positive; '
            'padding and z-min cannot be negative; z-max must exceed z-min; '
            'plane threshold and iterations must be positive; ground normal Z must be in (0, 1]; '
            'ground seed percentile must be in (0, 100]')
    image, origin_x, origin_y = occupancy_image(
        read_pcd_xyz(args.input_pcd), args.resolution, args.padding, args.obstacle_height,
        args.min_obstacle_points, args.z_min, args.z_max,
        args.plane_distance_threshold, args.plane_iterations, args.plane_seed,
        args.min_ground_normal_z, args.ground_seed_percentile)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    pgm = args.output_dir / 'map.pgm'
    with pgm.open('wb') as stream:
        stream.write(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode('ascii'))
        stream.write(image.tobytes())
    (args.output_dir / 'map.yaml').write_text(
        'image: map.pgm\nresolution: {:.6f}\norigin: [{:.6f}, {:.6f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n'.format(
            args.resolution, origin_x, origin_y), encoding='ascii')
    print(f'wrote {pgm} ({image.shape[1]} x {image.shape[0]})')


if __name__ == '__main__':
    main()
