import numpy as np
from mower_nav2.visibility_mapping import visibility_endpoints, mark_visible, filter_self_returns


def test_ground_ray_stops_before_nearer_obstacle():
    points=np.array([[4.,0.,0.],[2.,0.,0.5]])
    ends=visibility_endpoints(points,np.array([0.,0.]),np.array([0.,0.,1.,0.]))
    assert np.linalg.norm(ends[0]) <= 2.000001


def test_overhead_returns_do_not_clear_unknown_ground():
    ends=visibility_endpoints(np.array([[4.,0.,2.]]),np.array([0.,0.]),np.array([0.,0.,1.,0.]))
    assert len(ends)==0


def test_obstacle_without_ground_return_does_not_prove_floor():
    ends=visibility_endpoints(np.array([[4.,0.,0.5]]),np.array([0.,0.]),np.array([0.,0.,1.,0.]))
    assert len(ends)==0


def test_visibility_preserves_occupied_cells_and_unseen_space():
    grid=np.full((10,10),205,np.uint8);grid[1,3]=0
    mark_visible(grid,[0.15,0.15],np.array([[0.65,0.15]]),[0.,0.],0.1)
    assert grid[1,3]==0
    assert grid[1,1]==254
    assert grid[9,9]==205


def test_vehicle_returns_removed_after_rotated_world_pose():
    rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    position=np.array([3.,4.,0.])
    local=np.array([[-0.3,0.,0.5],[1.,0.,0.5]])
    world=local@rotation.T+position
    result=filter_self_returns(world,position,rotation,[[-0.55,-0.18,-0.42,0.42,-0.15,1.2]])
    assert len(result)==1
    np.testing.assert_allclose(result[0],world[1])


def test_physical_review_preserves_cells_outside_and_across_boundary():
    from mower_nav2.visibility_mapping import reviewed_free_disk
    original = np.full((9, 9), 205, np.uint8)
    original[4, 4] = 0
    output, mask = reviewed_free_disk(original, [0.45, 0.45], 0.30, [0., 0.], 0.1)
    assert output[4, 4] == 254
    assert original[4, 4] == 0
    assert output[4, 7] == 205
    np.testing.assert_array_equal(output[~mask], original[~mask])
    y, x = np.where(mask)
    for dx in (-0.05, 0.05):
        for dy in (-0.05, 0.05):
            assert np.all(np.hypot((x+0.5)*0.1+dx-0.45, (y+0.5)*0.1+dy-0.45) <= 0.30)
