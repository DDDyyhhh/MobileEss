"""Candidate keeps lower physical command ceilings and the original stop budgets."""
import math
from pathlib import Path
import pytest
import yaml
from mower_nav2.reviewed_route import can_control_valid, validate_review
ROOT=Path(__file__).resolve().parents[1]

def candidate():
 return dict(route_profile='static_keepout_gentle_u_03',max_linear_speed_mps=.3,
  max_angular_speed_radps=.2,corridor_half_width_m=.2,max_run_seconds=150.,
  max_travel_m=16.,review_radius_m=.98,physical_whole_route_confirmed=True,
  pose={'xy_m':[0.,0.],'yaw_deg':0.},goal_xy_m=[2.5,-3.],goal_yaw_deg=180.,
  path_xy_yaw_rad=[[0.,0.,0.],[7.,0.,0.],[7.,-3.,-math.pi/2],[2.5,-3.,math.pi]])

def test_broad_candidate_can_use_14_to_15m_without_expanding_travel_budget():
 p=candidate();validate_review(p)
 with pytest.raises(ValueError):validate_review(dict(p,route_profile='static_keepout_right_u_04',max_linear_speed_mps=.4))

@pytest.mark.parametrize('change',[{'max_linear_speed_mps':.4},{'max_angular_speed_radps':.3},
 {'corridor_half_width_m':.3},{'max_run_seconds':151.},{'max_travel_m':17.},
 {'physical_whole_route_confirmed':False}])
def test_new_candidate_cannot_reuse_old_speed_or_expand_safety_budgets(change):
 with pytest.raises(ValueError):validate_review(dict(candidate(),**change))

def test_new_can_limit_rejects_old_04_command_but_accepts_03_turn():
 assert can_control_valid(bytes([2,3,0,254,0,1,0,0]),3)
 assert not can_control_valid(bytes([2,4,0,254,0,1,0,0]),3)
 assert can_control_valid(bytes([2,4,0,254,0,1,0,0]))
 assert not can_control_valid(bytes([2,3,0,254,0,1,0,170]),3)

def test_config_bounds_match_bridge_and_keep_obstacle_checks():
 configs=ROOT/'config';base=yaml.safe_load((configs/'nav2_keepout_u_04.yaml').read_text());new=yaml.safe_load((configs/'nav2_tracking_gentle_03.yaml').read_text());c=new['controller_server']['ros__parameters'];follow=c['FollowPath'];bridge=yaml.safe_load((configs/'cmd_vel_tracking_gentle_03.yaml').read_text())['cmd_vel_can_bridge']['ros__parameters']
 assert follow['vx_max']==new['velocity_smoother']['ros__parameters']['max_velocity'][0]==bridge['max_linear_speed_mps']==.3
 assert follow['wz_max']==bridge['max_angular_speed_radps']==.2
 assert follow['model_dt']*c['controller_frequency']==1
 assert follow['time_steps']*follow['model_dt']==3.6
 assert follow['batch_size']*follow['time_steps']*c['controller_frequency']==27000
 assert follow['PathAngleCritic.cost_weight']>base['controller_server']['ros__parameters']['FollowPath']['PathAngleCritic.cost_weight']
 assert new['local_costmap']==base['local_costmap'] and new['global_costmap']==base['global_costmap']
 assert c['failure_tolerance']==0 and follow['retry_attempt_limit']==0
 assert bridge['command_timeout_sec']==.25 and bridge['localization_timeout_sec']==2.5
