from pathlib import Path
from math import radians
import yaml
import xml.etree.ElementTree as ET
import pytest
from mower_nav2.ground_check import corridor_error, heading_error, needs_initial_alignment, navigation_limits


def test_short_navigation_speed_requires_explicit_coupled_profile():
    assert navigation_limits({}) == (.1, 1, 250)
    p = dict(navigation_profile='short_04', max_linear_speed_mps=.4,
             max_angular_speed_radps=.2, corridor_half_width_m=.2)
    assert navigation_limits(p) == (.4, 4, 550)
    for change in (dict(max_linear_speed_mps=.5), dict(max_angular_speed_radps=.4),
                   dict(corridor_half_width_m=.3), dict(navigation_profile='unbounded')):
        with pytest.raises(ValueError):
            navigation_limits(dict(p, **change))


def test_short_04_configs_change_only_linear_ceiling_and_retain_alignment_and_obstacles():
    root = Path(__file__).resolve().parents[1]
    align = yaml.safe_load((root/'config/nav2_ground_turn_align.yaml').read_text())
    short = yaml.safe_load((root/'config/nav2_ground_short_04.yaml').read_text())
    align['controller_server']['ros__parameters']['FollowPath']['vx_max'] = .4
    align['velocity_smoother']['ros__parameters']['max_velocity'][0] = .4
    assert short == align
    bridge = yaml.safe_load((root/'config/cmd_vel_ground_first.yaml').read_text())
    bridge['cmd_vel_can_bridge']['ros__parameters']['max_linear_speed_mps'] = .4
    assert bridge == yaml.safe_load((root/'config/cmd_vel_nav_short_04.yaml').read_text())


@pytest.mark.parametrize('pose', [(0, 0), (0.5, 0.2), (1, 0), (0.9, -0.24)])
def test_reviewed_short_corridor_accepts_internal_points(pose):
    assert corridor_error((0, 0), (1, 0), pose) is None


@pytest.mark.parametrize('pose', [(-0.11, 0), (1.16, 0), (0.5, 0.26), (float('nan'), 0)])
def test_reviewed_short_corridor_rejects_reverse_overshoot_side_detours_and_invalid_pose(pose):
    assert corridor_error((0, 0), (1, 0), pose)


def test_first_ground_tree_has_path_following_and_no_recovery_motion():
    root = Path(__file__).resolve().parents[1]
    tree = ET.parse(root/'behavior_trees/navigate_ground_short.xml')
    tags = {node.tag for node in tree.iter()}
    assert {'ComputePathToPose', 'FollowPath'} <= tags
    assert not {'RecoveryNode', 'Spin', 'BackUp'} & tags



@pytest.mark.parametrize('angle', [0, -15, -35, -40])
def test_reviewed_right_turn_allows_intermediate_headings(angle):
    assert heading_error(0, radians(-35), radians(angle), margin=0.2) is None


@pytest.mark.parametrize('angle', [15, -50, float('nan')])
def test_reviewed_right_turn_rejects_wrong_turn_overshoot_and_invalid_heading(angle):
    assert heading_error(0, radians(-35), radians(angle), margin=0.2)


def test_reviewed_turn_handles_heading_wrap_without_approving_opposite_rotation():
    assert heading_error(radians(175), radians(-150), radians(-175), margin=0.2) is None
    assert heading_error(radians(175), radians(-150), radians(145), margin=0.2)


def test_short_turn_retains_speed_and_collision_limits_with_tighter_final_heading():
    root = Path(__file__).resolve().parents[1]
    first = yaml.safe_load((root/'config/nav2_ground_first.yaml').read_text())
    turn = yaml.safe_load((root/'config/nav2_ground_turn.yaml').read_text())
    assert turn['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance'] == 0.15
    first['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance'] = 0.15
    first['controller_server']['ros__parameters']['FollowPath']['PathAngleCritic.max_angle_to_furthest'] = 0.15
    assert turn == first


def test_align_first_controller_retains_short_route_speed_and_safety_settings():
    root = Path(__file__).resolve().parents[1]
    turn = yaml.safe_load((root/'config/nav2_ground_turn.yaml').read_text())
    align = yaml.safe_load((root/'config/nav2_ground_turn_align.yaml').read_text())
    expected = turn['controller_server']['ros__parameters']['FollowPath']
    expected.update(plugin='nav2_rotation_shim_controller::RotationShimController',
                    primary_controller='nav2_mppi_controller::MPPIController',
                    angular_dist_threshold=0.1, angular_disengage_threshold=0.1,
                    forward_sampling_distance=0.3, rotate_to_heading_angular_vel=0.2,
                    max_angular_accel=0.5, simulate_ahead_time=1.0,
                    rotate_to_goal_heading=False)
    assert turn == align


@pytest.mark.parametrize('start,goal,required', [(0,0,False),(0,4,False),
                                                (0,-35,True),(0,35,True),
                                                (179,-179,False),(175,-150,True)])
def test_short_turn_requires_initial_alignment_and_handles_wrap(start,goal,required):
    assert needs_initial_alignment(radians(start),radians(goal)) is required


def test_initial_alignment_uses_path_sample_instead_of_final_goal_heading():
    from mower_nav2.ground_check import sampled_path_point, initial_path_alignment_error
    path = [(2.9, .4), (3.0, .4), (3.1, .4), (3.2, .4), (3.733, .373)]
    sample = sampled_path_point(path)
    pose = (2.946297730176093, .4372726483377048, -.16915994043864419)
    assert initial_path_alignment_error(pose, sample) is None
    # Replayed trial: still about 10 degrees away from final goal orientation.
    assert abs(pose[2]-radians(.5193178946865586)) > .1
    assert initial_path_alignment_error((pose[0],pose[1],-.6005), sample)


@pytest.mark.parametrize('path', [[], [(0,0)], [(0,0),(.1,0)], [(0,0),(float('nan'),0)]])
def test_initial_alignment_rejects_missing_short_or_invalid_path_sample(path):
    from mower_nav2.ground_check import sampled_path_point
    with pytest.raises(ValueError):
        sampled_path_point(path)


@pytest.mark.parametrize('pose,target', [((0,0,0),None), ((0,0,0),(0,0)),
                                        ((0,0,float('nan')),(1,0))])
def test_initial_alignment_rejects_invalid_or_coincident_target(pose,target):
    from mower_nav2.ground_check import initial_path_alignment_error
    assert initial_path_alignment_error(pose,target)
