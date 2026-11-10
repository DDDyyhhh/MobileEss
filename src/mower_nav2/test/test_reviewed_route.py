import copy
import math

import pytest

from mower_nav2.reviewed_route import RouteGuard,can_control_valid,project,validate_review


def review():
    return dict(route_profile='static_keepout_right_u_04',max_linear_speed_mps=.4,
        max_angular_speed_radps=.2,corridor_half_width_m=.2,max_run_seconds=150.,
        max_travel_m=16.,review_radius_m=.98,physical_whole_route_confirmed=True,
        pose={'xy_m':[0.,0.],'yaw_deg':0.},goal_xy_m=[2.,-3.],goal_yaw_deg=180.,
        path_xy_yaw_rad=[[0.,0.,0.],[5.,0.,0.],[5.,-3.,-math.pi/2],[2.,-3.,math.pi]])


def test_projection_follows_the_polyline_not_the_start_goal_chord():
    path=review()['path_xy_yaw_rad']
    assert project(path,[5.1,-1.])==pytest.approx((.1,6.))
    assert project(path,[2.,-1.])[0]==pytest.approx(1.)


def test_review_requires_physical_confirmation_and_unchanged_budgets():
    p=review();validate_review(p)
    for key,value in [('physical_whole_route_confirmed',False),('max_linear_speed_mps',.5),
                      ('corridor_half_width_m',.3),('max_run_seconds',151.),('max_travel_m',17.)]:
        changed=copy.deepcopy(p);changed[key]=value
        with pytest.raises(ValueError):validate_review(changed)


def test_execution_cannot_cut_across_the_inside_of_the_u():
    p=review();p['execution_path_xy_yaw_rad']=[[0.,0.,0.],[5.,-3.,0.],[2.,-3.,math.pi]]
    with pytest.raises(ValueError,match='tube'):validate_review(p)


def test_bad_endpoints_nonfinite_and_short_paths_are_rejected():
    for path in ([[0.,0.,0.],[1.,0.,0.]], [[0.,0.,0.],[math.nan,0.,0.]],
                 [[0.,0.,0.],[5.,0.,0.],[5.,-3.,0.],[3.,-3.,0.]]):
        p=review();p['path_xy_yaw_rad']=path
        with pytest.raises(ValueError):validate_review(p)


def test_route_guard_rejects_sideways_escape_jump_and_backtracking():
    path=review()['path_xy_yaw_rad'];g=RouteGuard(path)
    g.update([0.,0.]);g.update([.4,0.])
    with pytest.raises(ValueError,match='corridor'):g.update([.4,.21])
    with pytest.raises(ValueError,match='jumped'):g.update([1.1,0.])
    g=RouteGuard(path)
    for x in (0.,.4,.8,1.2):g.update([x,0.])
    with pytest.raises(ValueError,match='regressed'):g.update([.7,0.])


def test_stationary_repeats_do_not_accumulate_fake_travel():
    g=RouteGuard(review()['path_xy_yaw_rad'])
    for _ in range(100):g.update([0.,0.])
    assert g.travel==0


@pytest.mark.parametrize('data', [bytes([2,5,0,0,0,1,0,0]),bytes([2,255,0,0,0,1,0,0]),
    bytes([2,4,1,0,0,1,0,0]),bytes([2,4,0,3,0,1,0,0]),bytes([2,4,0,0,1,1,0,0]),
    bytes([2,4,0,0,0,1,0,170]),bytes([1,1,0,0,0,1,0,0]),bytes(7)])
def test_command_boundary_rejects_reverse_overspeed_blade_and_unlock(data):
    assert not can_control_valid(data)


def test_command_boundary_allows_reviewed_forward_and_both_turn_directions():
    assert can_control_valid(bytes([2,4,0,2,0,1,0,0]))
    assert can_control_valid(bytes([2,0,0,254,0,1,0,0]))
