"""Keep the previous filled-cell envelope, including rotated map boundaries."""
import math
import random

import pytest

from mower_console.geometry import Grid


def reference_disk(grid, x, y, radius):
    step=grid.resolution
    n=math.ceil((radius+step)/step)
    for ix in range(-n,n+1):
        for iy in range(-n,n+1):
            if math.hypot(ix*step,iy*step)>radius+step*math.sqrt(2)/2:
                continue
            index=grid.index(x+ix*step,y+iy*step)
            if index is None or not 0<=grid.data[index]<grid.occupied:
                return False
    return True


@pytest.mark.parametrize('yaw',[0.,.37,math.pi/2,-math.pi/2])
def test_batched_envelope_matches_filled_cell_reference(yaw):
    rng=random.Random(378)
    grid=Grid(70,70,.1,(-3,-3,yaw),[0]*4900)
    for _ in range(50):
        grid.data[rng.randrange(len(grid.data))]=rng.choice([-1,100])
    centers=[(rng.uniform(-5,5),rng.uniform(-5,5)) for _ in range(100)]
    for x,y in centers:
        assert grid.clear_disk(x,y,.98)==reference_disk(grid,x,y,.98)
    assert grid._clear_centers(centers,.98)==all(reference_disk(grid,x,y,.98) for x,y in centers)


def test_long_sweep_still_rejects_interior_unknown_obstacle_and_edge():
    grid=Grid(500,60,.1,(-2,-3,0),[0]*30000)
    route=[{'kind':'forward','poses':[[0,0,0],[40,0,0]]}]
    grid.validate(route)
    index=grid.index(21.3,.3)
    for value in (-1,100):
        grid.data[index]=value
        with pytest.raises(ValueError,match='整车通行范围'):
            grid.validate(route)
    grid.data[index]=0
    with pytest.raises(ValueError,match='整车通行范围'):
        grid.validate([{'kind':'forward','poses':[[0,2.5,0],[1,2.5,0]]}])
