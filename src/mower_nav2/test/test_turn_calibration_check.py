import math

import pytest

from mower_nav2.distance_calibration_check import (
    integrate_wheel_yaw, signed_yaw_change, turn_guard, turn_profile,
)


def proposal():
    return dict(speed_command_m_s=0., angular_command_rad_s=.2,
                drive_duration_s=3., max_displacement_m=.2,
                max_turn_angle_rad=math.pi/4, stop_observation_limit_s=1.5,
                track_width_m=.640)


def test_only_reviewed_stationary_turn_profiles_are_allowed():
    assert turn_profile(proposal()) == (0., 3., .2, 1., 250, 0)
    assert turn_profile(dict(proposal(), angular_command_rad_s=-.2)) == (0., 3., .2, 1., 250, 0)


@pytest.mark.parametrize('change', [dict(speed_command_m_s=.1),
    dict(angular_command_rad_s=-.3), dict(angular_command_rad_s=.4),
    dict(drive_duration_s=5.), dict(max_displacement_m=.3),
    dict(max_turn_angle_rad=math.pi/2), dict(stop_observation_limit_s=2.),
    dict(track_width_m=.69)])
def test_approval_cannot_expand_turn_speed_time_geometry_or_track(change):
    with pytest.raises(ValueError):
        turn_profile(dict(proposal(), **change))


def test_turn_guard_allows_left_rotation_across_yaw_wrap():
    start = [0., 0., math.radians(170)]
    end = [.02, -.02, math.radians(-160)]
    assert signed_yaw_change(start[2], end[2]) == pytest.approx(math.pi/6)
    assert turn_guard(start, end) is None


def test_right_turn_guard_preserves_wrap_direction_and_existing_limits():
    start = [0., 0., math.radians(-170)]
    end = [.02, -.02, math.radians(160)]
    assert turn_guard(start, end, direction=-1) is None
    assert turn_guard([0., 0., 0.], [0., 0., .1], direction=-1)
    assert turn_guard([0., 0., 0.], [0., 0., -math.pi/4], direction=-1)
    assert turn_guard([0., 0., 0.], [.2, 0., -.1], direction=-1)
    assert turn_guard(start, end, direction=0)


@pytest.mark.parametrize('end', [[.2, 0., .1], [0., 0., -.1],
    [0., 0., math.pi/4], [0., float('nan'), 0.]])
def test_turn_guard_rejects_translation_wrong_direction_overshoot_and_invalid_pose(end):
    assert turn_guard([0., 0., 0.], end)


def test_wheel_yaw_uses_actual_left_right_speeds_and_includes_ramps():
    samples = [[0., 0., 0., 0., 0.], [.1, -64., 64., -64., 64.],
               [.2, -64., 64., -64., 64.], [.3, 0., 0., 0., 0.]]
    assert integrate_wheel_yaw(samples, 0., .3) == pytest.approx(.04)
    reverse = [[t, -fl, -fr, -rl, -rr] for t, fl, fr, rl, rr in samples]
    assert integrate_wheel_yaw(reverse, 0., .3) == pytest.approx(-.04)
    # Equal wheel speeds are pure translation and must never count as turn.
    assert integrate_wheel_yaw([[t, 64, 64, 64, 64] for t in (0., .1)], 0., .1) == 0.


def test_feedback_gap_cannot_generate_a_turn_angle():
    with pytest.raises(ValueError):
        integrate_wheel_yaw([[0., -64, 64, -64, 64], [.2, -64, 64, -64, 64]], 0., .2)
