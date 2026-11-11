from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_field_controller_smoother_and_can_share_low_speed_limit():
    params = yaml.safe_load((ROOT / 'config/nav2_field.yaml').read_text())
    controller = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    bridge = yaml.safe_load((ROOT / 'config/cmd_vel_field.yaml').read_text())['cmd_vel_can_bridge']['ros__parameters']
    assert controller['vx_max'] == smoother['max_velocity'][0] == bridge['max_linear_speed_mps'] == 0.2
    assert controller['wz_max'] == smoother['max_velocity'][2] == bridge['max_angular_speed_radps'] == 0.3
    assert controller['vx_min'] == smoother['min_velocity'][0] == 0.0
    assert bridge['command_topic'] == '/cmd_vel_gated'
    assert bridge['control_period_sec'] == 0.05
    assert bridge['allow_reverse'] is False
    local = params['local_costmap']['local_costmap']['ros__parameters']
    assert 'static_layer' in local['plugins']


def test_automatic_startup_is_bounded_and_requires_confirmation():
    p=yaml.safe_load((ROOT/'config/localizer_field.yaml').read_text())['fixed_map_localizer']['ros__parameters']
    assert p['startup_search_enabled'] is True
    assert p['startup_search_radius_m']==p['max_initial_translation_m']==3.0
    assert p['max_initial_yaw_rad']>=3.1415926
    assert p['startup_confirmation_count']>=3
    assert p['startup_min_overlap']>=0.65
    assert p['startup_ambiguity_margin']>0


def test_cad_footprint_and_virtual_bumper_cover_vehicle_corners():
    import ast
    import math
    p = yaml.safe_load((ROOT/'config/nav2_field.yaml').read_text())
    global_map = p['global_costmap']['global_costmap']['ros__parameters']
    local_map = p['local_costmap']['local_costmap']['ros__parameters']
    assert global_map['footprint'] == local_map['footprint']
    points = ast.literal_eval(local_map['footprint'])
    assert max(abs(x) for x, _ in points)*2 >= 1.08606
    assert max(abs(y) for _, y in points)*2 >= 0.768
    pad = local_map['footprint_padding']
    assert max(math.hypot(abs(x)+pad, abs(y)+pad) for x, y in points) < 0.70
    assert global_map['inflation_layer']['inflation_radius'] == local_map['inflation_layer']['inflation_radius'] == 0.90
    controller = p['controller_server']['ros__parameters']['FollowPath']
    assert controller['ObstaclesCritic.consider_footprint'] is True
    assert controller['CostCritic.consider_footprint'] is True

