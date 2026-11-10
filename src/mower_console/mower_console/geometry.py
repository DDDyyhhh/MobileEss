"""Map coordinates and direction-preserving path validation, without ROS."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def finite_pose(pose):
    return len(pose) == 3 and all(math.isfinite(float(v)) for v in pose)


def length(poses):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:]))


def split_path(poses):
    """Retain pose headings; a reverse path points its body opposite travel."""
    if len(poses) < 2 or len(poses) > 10000 or not all(finite_pose(p) for p in poses):
        raise ValueError('无效路线')
    segments = []
    for a, b in zip(poses, poses[1:]):
        distance = math.dist(a[:2], b[:2])
        turn = angle(b[2] - a[2])
        if distance < 1e-5:
            if abs(turn) < 1e-5:
                continue
            kind = 'rotate'
        else:
            mid_yaw = a[2] + turn / 2
            direction = ((b[0]-a[0])*math.cos(mid_yaw) +
                         (b[1]-a[1])*math.sin(mid_yaw))
            if abs(direction) < distance * .65:
                raise ValueError('路线存在侧向平移或不连续朝向')
            kind = 'forward' if direction > 0 else 'reverse'
            if abs(turn) > distance / 4.0 + max(.0001, distance*.001):
                raise ValueError('路线曲率超出4米参考半径')
        # Adjacent rotations with opposite signs must remain distinct.
        same = (segments and segments[-1]['kind'] == kind and
                (kind != 'rotate' or segments[-1]['turn'] * turn >= 0))
        if not same:
            segments.append({'kind': kind, 'poses': [list(a)], 'length_m': 0., 'turn': 0.})
        segment = segments[-1]
        segment['poses'].append(list(b))
        segment['length_m'] += distance
        segment['turn'] += turn
    if not segments or length(poses) > 40.:
        raise ValueError('路线为空或超过40米')
    # A rotation or a segment boundary cannot reset the reverse budget.
    reverse_run = 0.
    for segment in segments:
        if segment['kind'] == 'reverse':
            reverse_run += segment['length_m']
            if reverse_run > 1.0001:
                raise ValueError('连续倒车超过1米')
        elif segment['kind'] == 'forward':
            reverse_run = 0.
    return segments


def distance_to_path(poses, xy):
    result = math.inf
    for a, b in zip(poses, poses[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm = dx*dx + dy*dy
        t = 0. if norm < 1e-12 else max(0., min(1., ((xy[0]-a[0])*dx + (xy[1]-a[1])*dy)/norm))
        result = min(result, math.hypot(xy[0]-a[0]-t*dx, xy[1]-a[1]-t*dy))
    return result


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin: tuple
    data: list
    occupied: int = 65

    def index(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x-ox, y-oy
        col = math.floor((math.cos(yaw)*dx + math.sin(yaw)*dy)/self.resolution)
        row = math.floor((-math.sin(yaw)*dx + math.cos(yaw)*dy)/self.resolution)
        return row*self.width+col if 0 <= col < self.width and 0 <= row < self.height else None

    def clear_disk(self, x, y, radius):
        return self._clear_centers([(x, y)], radius)

    def _clear_centers(self, centers, radius):
        # Cover every intersecting cell, including its half diagonal. Sampling
        # only perimeter/rings misses isolated occupied cells inside the body.
        # Bounded NumPy batches retain that envelope without a Python loop per
        # cell: the old route loop held up ROS/HTTP work beyond its 300ms limit.
        step = self.resolution
        n = math.ceil((radius + step)/step)
        ix, iy = np.meshgrid(np.arange(-n, n+1), np.arange(-n, n+1))
        dx, dy = ix.ravel()*step, iy.ravel()*step
        mask = np.hypot(dx, dy) <= radius + step*math.sqrt(2)/2
        dx, dy = dx[mask], dy[mask]
        points = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
        if not np.isfinite(points).all():
            return False
        cells = np.asarray(self.data, dtype=np.int16)
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        for start in range(0, len(points), 64):
            batch = points[start:start+64]
            x = batch[:, 0, None] + dx - ox
            y = batch[:, 1, None] + dy - oy
            cols = np.floor((c*x+s*y)/step).astype(np.int64)
            rows = np.floor((-s*x+c*y)/step).astype(np.int64)
            if not ((cols >= 0) & (cols < self.width) &
                    (rows >= 0) & (rows < self.height)).all():
                return False
            values = cells[rows*self.width+cols]
            if not ((values >= 0) & (values < self.occupied)).all():
                return False
        return True

    def validate(self, segments):
        centers = []
        for segment in segments:
            # .98m is the existing whole-route review envelope, encompassing
            # the .70m body and the initial .25m rotation-slip allowance.
            for a, b in zip(segment['poses'], segment['poses'][1:]):
                count = max(1, math.ceil(math.dist(a[:2], b[:2])/.05))
                for i in range(count+1):
                    center = (a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count)
                    if not centers or center != centers[-1]:
                        centers.append(center)
        if not self._clear_centers(centers, .98):
            raise ValueError('整车通行范围进入障碍、未知区或地图外')
