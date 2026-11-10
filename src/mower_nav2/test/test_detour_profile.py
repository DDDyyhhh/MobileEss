"""The candidate exposes live obstacles to planning without relaxing controls."""
from copy import deepcopy
from pathlib import Path

import yaml


def test_detour_profile_preserves_static_keepouts_and_motion_safety_configuration():
    configs = Path(__file__).resolve().parents[1] / 'config'
    baseline = yaml.safe_load((configs / 'nav2_ground_short_04.yaml').read_text())
    candidate = yaml.safe_load((configs / 'nav2_ground_detour_04.yaml').read_text())
    global_params = candidate['global_costmap']['global_costmap']['ros__parameters']
    assert global_params['plugins'] == ['static_layer', 'timed_dynamic_layer', 'inflation_layer']
    assert global_params['timed_dynamic_layer'] == baseline['local_costmap']['local_costmap']['ros__parameters']['timed_dynamic_layer']
    assert global_params['timed_dynamic_layer']['enabled'] is True
    assert global_params['timed_dynamic_layer']['topic'] == '/nav2/mid360_obstacles'
    assert global_params['update_frequency'] == 5.0
    assert global_params['publish_frequency'] == 2.0
    assert global_params['always_send_full_costmap'] is True
    # The local gate evidence, footprint, no-reverse controller and speed
    # ceilings must stay identical to the already reviewed short profile.
    expected = deepcopy(baseline)
    expected['global_costmap']['global_costmap']['ros__parameters'] = global_params
    assert candidate == expected
    for key in ('static_layer', 'inflation_layer', 'footprint', 'footprint_padding', 'track_unknown_space'):
        assert global_params[key] == baseline['global_costmap']['global_costmap']['ros__parameters'][key]


def test_keepout_u_profile_only_increases_global_obstacle_clearance():
    configs = Path(__file__).resolve().parents[1] / 'config'
    baseline = yaml.safe_load((configs / 'nav2_ground_detour_04.yaml').read_text())
    candidate = yaml.safe_load((configs / 'nav2_keepout_u_04.yaml').read_text())
    baseline['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = 1.1
    assert candidate == baseline
