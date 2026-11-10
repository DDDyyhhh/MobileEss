"""Conservative rear ray coverage; missing/self-occluded beams never clear space."""
import math


def rear_observed(points, reach=1.7):
    # Every 5-degree sector in the rear swept corridor needs a usable return
    # beyond the stop horizon. A nearer body/post return shadows that sector.
    nearest = {}
    for p in points:
        x, y, z = float(p.x)+.17, float(p.y), float(p.z)+.48
        if not all(math.isfinite(v) for v in (x, y, z)) or x >= 0:
            continue
        distance = math.hypot(x, y)
        if distance < .20 or z < .10 or z > 1.5:
            continue
        sector = math.floor((math.atan2(y, x)+math.pi)/math.radians(5)) % 72
        nearest[sector] = min(distance, nearest.get(sector, math.inf))
    required = list(range(0, 10)) + list(range(62, 72))
    return bool(points) and all(nearest.get(s, 0.) >= reach for s in required)
