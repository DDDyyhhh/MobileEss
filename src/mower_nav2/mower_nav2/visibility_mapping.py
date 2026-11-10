"""Ground-supported visibility rays for an offline navigation-map candidate."""
import numpy as np


def keepout_rectangles(grid, rectangles, origin, resolution, margin=0.0):
    """Block every grid cell intersecting a reviewed rectangle and its margin."""
    rectangles = np.asarray(rectangles, dtype=float)
    if (rectangles.ndim != 2 or rectangles.shape[1] != 4 or
            not np.isfinite(rectangles).all() or not np.isfinite(resolution) or
            resolution <= 0 or not np.isfinite(margin) or margin < 0 or
            np.any(rectangles[:, 0] >= rectangles[:, 2]) or
            np.any(rectangles[:, 1] >= rectangles[:, 3])):
        raise ValueError('valid xmin,ymin,xmax,ymax rectangles and resolution required')
    y, x = np.indices(grid.shape)
    x0 = origin[0] + x*resolution
    y0 = origin[1] + y*resolution
    mask = np.zeros(grid.shape, dtype=bool)
    for xmin, ymin, xmax, ymax in rectangles:
        mask |= ((x0 < xmax+margin) & (x0+resolution > xmin-margin) &
                 (y0 < ymax+margin) & (y0+resolution > ymin-margin))
    output = grid.copy()
    output[mask] = 0
    return output, mask


class TemporalStaticGrid:
    """Count distinct scans and require later ground visibility to clear hits.

    This produces a review candidate, not a classification of people or a
    safety approval. Missing observations never establish free space.
    """

    def __init__(self, shape):
        self.hits = np.zeros(shape, dtype=np.int32)
        self.later_free = np.zeros(shape, dtype=np.int32)
        self.seen_free = np.zeros(shape, dtype=bool)

    def observe(self, occupied, ground_visible):
        occupied = np.asarray(occupied, dtype=bool)
        ground_visible = np.asarray(ground_visible, dtype=bool)
        if occupied.shape != self.hits.shape or ground_visible.shape != self.hits.shape:
            raise ValueError('observation shapes must match the grid')
        free = ground_visible & ~occupied
        self.hits[occupied] += 1
        self.later_free[occupied] = 0
        self.later_free[free] += 1
        self.seen_free |= free

    def result(self):
        grid = np.full(self.hits.shape, 205, dtype=np.uint8)
        cleared = self.later_free >= 5
        grid[self.seen_free & ((self.hits == 0) | cleared)] = 254
        grid[(self.hits >= 3) & ~cleared] = 0
        return grid


def reviewed_free_disk(grid, center_xy, radius, origin, resolution):
    """Apply a human-reviewed free region; retain cells crossing its boundary."""
    if radius <= 0 or resolution <= 0:
        raise ValueError('positive radius and resolution required')
    y, x = np.indices(grid.shape)
    xy_x = origin[0] + (x+0.5)*resolution
    xy_y = origin[1] + (y+0.5)*resolution
    # A whole cell must lie inside the review boundary, not just its center.
    distance = np.hypot(xy_x-center_xy[0], xy_y-center_xy[1])
    mask = distance + resolution/np.sqrt(2) <= radius
    output = grid.copy()
    output[mask] = 254
    return output, mask


def filter_self_returns(points, sensor_position, rotation, boxes):
    """Apply measured chassis boxes in the sensor frame, before world mapping."""
    local = (points - sensor_position) @ rotation
    keep = np.ones(len(points), dtype=bool)
    for x0,x1,y0,y1,z0,z1 in boxes:
        keep &= ~((local[:,0]>=x0)&(local[:,0]<=x1)&
                  (local[:,1]>=y0)&(local[:,1]<=y1)&
                  (local[:,2]>=z0)&(local[:,2]<=z1))
    return points[keep]


def visibility_endpoints(points, sensor_xy, plane, max_range=20.0, bins=720):
    """Stop ground visibility at the nearest same-bearing obstacle return.

    Returns above the selected obstacle band never prove traversable ground.
    No ground return means no clearing ray for that bearing.
    """
    points = points[np.isfinite(points).all(axis=1)]
    delta = points[:, :2] - sensor_xy
    distance = np.linalg.norm(delta, axis=1)
    height = points @ plane[:3] + plane[3]
    valid = (distance >= 0.2) & (distance <= max_range)
    angles = np.arctan2(delta[:, 1], delta[:, 0])
    keys = np.minimum(bins - 1, ((angles + np.pi) * bins / (2*np.pi)).astype(int))
    ground = valid & (np.abs(height) <= 0.10)
    obstacles = valid & (height >= 0.15) & (height <= 0.80)
    far_ground = np.zeros(bins)
    near_obstacle = np.full(bins, np.inf)
    np.maximum.at(far_ground, keys[ground], distance[ground])
    np.minimum.at(near_obstacle, keys[obstacles], distance[obstacles])
    length = np.minimum(far_ground, near_obstacle)
    chosen = length >= 0.2
    angle = (np.flatnonzero(chosen) + 0.5) * 2*np.pi/bins - np.pi
    return sensor_xy + np.column_stack((np.cos(angle), np.sin(angle))) * length[chosen, None]


def mark_visible(grid, sensor_xy, endpoints, origin, resolution):
    """Rasterize measured visibility only; occupied cells always win."""
    if len(endpoints) == 0:
        return
    start = (np.asarray(sensor_xy) - origin) / resolution
    ends = (endpoints - origin) / resolution
    steps = int(np.ceil(np.abs(ends - start).max())) + 1
    samples = start + np.linspace(0, 1, steps)[:, None, None] * (ends - start)
    cells = np.floor(samples).astype(int).reshape(-1, 2)
    h, w = grid.shape
    valid = (cells[:, 0] >= 0) & (cells[:, 0] < w) & (cells[:, 1] >= 0) & (cells[:, 1] < h)
    x, y = cells[valid].T
    clear = grid[y, x] != 0
    grid[y[clear], x[clear]] = 254
