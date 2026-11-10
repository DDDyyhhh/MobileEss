import pytest

from mower_nav2.distance_calibration_check import integrate_wheel_distance, fresh_gate, measurement_profile


def test_reviewed_profiles_keep_duration_and_safety_limits_coupled():
    p = dict(stop_observation_limit_s=1.5, corridor_half_width_m=.2,
             drive_duration_s=5., max_displacement_m=.9)
    assert measurement_profile(p) == (.1, 5., .9, 1., 250, 1)
    p.update(speed_command_m_s=.4, drive_duration_s=2., max_displacement_m=1.2)
    assert measurement_profile(p) == (.4, 2., 1.2, 2., 550, 4)
    for overrides in (dict(drive_duration_s=5.), dict(speed_command_m_s=.5),
                      dict(corridor_half_width_m=.3), dict(stop_observation_limit_s=2.),
                      dict(max_displacement_m=2.)):
        with pytest.raises(ValueError):
            measurement_profile(dict(p, **overrides))


def test_integration_clips_boundaries_and_averages_four_wheels():
    samples = [[t, 80, 120, 80, 120] for t in (0, .1, .2, .3)]
    assert integrate_wheel_distance(samples, .05, .25) == pytest.approx(.02)


def test_acceleration_and_deceleration_are_included():
    samples = [[0, 0, 0, 0, 0], [.1, 100, 100, 100, 100], [.2, 0, 0, 0, 0]]
    assert integrate_wheel_distance(samples, 0, .2) == pytest.approx(.01)


@pytest.mark.parametrize('samples', [
    [[0, 100, 100, 100, 100], [.2, 100, 100, 100, 100]],
    [[0, 100, 100, 100, 100], [0, 100, 100, 100, 100]],
    [[0, 100, 100, 100, 100], [.1, float('nan'), 100, 100, 100]],
    [[.01, 100, 100, 100, 100], [.1, 100, 100, 100, 100]],
])
def test_missing_or_invalid_feedback_cannot_produce_distance(samples):
    with pytest.raises(ValueError):
        integrate_wheel_distance(samples, 0, .1)


def test_stale_input_cannot_count_as_calibration():
    values = dict(command_age_sec='.05',costmap_age_sec='.05',odom_age_sec='.05',obstacle_age_sec='.05')
    assert fresh_gate(values)
    values['command_age_sec'] = '.26'
    assert not fresh_gate(values)
    values['command_age_sec'] = 'nan'
    assert not fresh_gate(values)
