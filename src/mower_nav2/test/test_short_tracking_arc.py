"""Short tracking trials have smaller duration/travel scope than full U trials."""
import math
import pytest
from mower_nav2.reviewed_route import RouteGuard,validate_review


def proposal():
 path=[[4*math.sin(a),4*(math.cos(a)-1),-a] for a in [i*math.pi/4/100 for i in range(101)]]
 return dict(route_profile='static_keepout_gentle_arc_03',max_linear_speed_mps=.3,
  max_angular_speed_radps=.2,corridor_half_width_m=.2,max_run_seconds=45.,
  max_travel_m=5.,review_radius_m=.98,physical_whole_route_confirmed=True,
  pose={'xy_m':[0.,0.],'yaw_deg':0.},goal_xy_m=path[-1][:2],goal_yaw_deg=-45.,
  path_xy_yaw_rad=path)


def test_short_radius_four_arc_is_a_separate_bounded_profile():
 reference,execution=validate_review(proposal())
 assert len(reference)==101 and reference==execution


@pytest.mark.parametrize('change',[{'max_run_seconds':150.},{'max_travel_m':16.},
 {'max_linear_speed_mps':.4},{'max_angular_speed_radps':.3},
 {'corridor_half_width_m':.3},{'review_radius_m':.8},
 {'physical_whole_route_confirmed':False}])
def test_short_trial_does_not_inherit_larger_full_route_or_relaxed_budgets(change):
 with pytest.raises(ValueError):validate_review(dict(proposal(),**change))


def test_short_path_cannot_be_mislabeled_as_a_full_u_trial():
 with pytest.raises(ValueError,match='length'):
  validate_review(dict(proposal(),route_profile='static_keepout_gentle_u_03',max_run_seconds=150.,max_travel_m=16.))


def test_smaller_travel_budget_really_stops_at_the_guard():
 guard=RouteGuard([[0.,0.,0.],[3.,0.,0.]],max_travel_m=1.)
 for x in (0.,.4,.8):guard.update([x,0.])
 with pytest.raises(ValueError,match='Travel'):guard.update([1.2,0.])


@pytest.mark.parametrize('budget',[0.,-1.,17.,float('nan')])
def test_invalid_travel_budget_is_rejected(budget):
 with pytest.raises(ValueError):RouteGuard([[0.,0.,0.],[3.,0.,0.]],budget)


def test_observe_only_geometry_does_not_fabricate_a_physical_confirmation():
 p=dict(proposal(),physical_whole_route_confirmed=False)
 validate_review(p,require_physical_confirmation=False)
 assert not p['physical_whole_route_confirmed']
 with pytest.raises(ValueError,match='physical'):validate_review(p)
