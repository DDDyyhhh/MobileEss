import pytest

from mower_nav2.command_expiry_check import command_expiry_blocks


def sample():
    return dict(input_linear_x='.1', input_angular_z='0', output_linear_x='0',
                output_angular_z='0', command_age_sec='.31', costmap_age_sec='.05',
                odom_age_sec='.05', obstacle_age_sec='.05')


REASON = 'stale command, costmap, odometry, or obstacle cloud'


def test_command_expiry_with_fresh_sensors_and_zero_output():
    assert command_expiry_blocks(sample(), REASON)
    assert not command_expiry_blocks(sample(), 'predicted footprint cost 253 >= 253')
    assert not command_expiry_blocks(sample(), 'collision-free prediction')


@pytest.mark.parametrize('key,value', [
    ('command_age_sec', '.25'), ('command_age_sec', 'nan'),
    ('obstacle_age_sec', '.31'), ('odom_age_sec', '.31'),
    ('costmap_age_sec', '.31'), ('input_linear_x', '0'),
    ('output_linear_x', '.1'), ('input_angular_z', '.1'),
    ('output_angular_z', '.1'), ('obstacle_age_sec', None),
])
def test_other_failure_or_idle_input_cannot_pass_command_expiry(key, value):
    values = sample()
    values[key] = value
    assert not command_expiry_blocks(values, REASON)


def test_missing_diagnostic_cannot_pass():
    assert not command_expiry_blocks({}, REASON)
