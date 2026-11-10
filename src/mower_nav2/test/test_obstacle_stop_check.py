import pytest
from mower_nav2.obstacle_stop_check import obstacle_blocks, obstacle_profile

def values():
    return dict(input_linear_x='.1', input_angular_z='0',output_linear_x='0',output_angular_z='0',command_age_sec='.05',costmap_age_sec='.05',odom_age_sec='.05',obstacle_age_sec='.05')

def test_only_fresh_obstacle_block_of_forward_input_counts():
    v=values()
    assert obstacle_blocks(v,'predicted footprint cost 253 >= 253')
    assert not obstacle_blocks(v,'stale command, costmap, odometry, or obstacle cloud')
    v['obstacle_age_sec']='.31'
    assert not obstacle_blocks(v,'predicted footprint cost 253 >= 253')

def test_idle_or_nonzero_output_or_invalid_values_do_not_count():
    for key,value in [('input_linear_x','0'),('output_linear_x','.1'),('input_angular_z','.1'),('odom_age_sec','nan')]:
        v=values();v[key]=value
        assert not obstacle_blocks(v,'predicted footprint cost 253 >= 253')
    assert not obstacle_blocks({},'predicted footprint cost 253 >= 253')


def test_block_requires_the_actual_reviewed_input_speed():
    v=values()
    assert not obstacle_blocks(v,'predicted footprint cost 253 >= 253',.4)
    v['input_linear_x']='.400'
    assert obstacle_blocks(v,'predicted footprint cost 253 >= 253',.4)
    v['command_age_sec']='.31'
    assert not obstacle_blocks(v,'predicted footprint cost 253 >= 253',.4)


def test_04_profile_cannot_inherit_the_historical_twelve_second_duration():
    assert obstacle_profile({}) == (.1,12.,.9,250,1)
    p=dict(obstacle_profile='obstacle_04',max_linear_speed_mps=.4,
           max_drive_seconds=4,max_approach_distance_m=.9)
    assert obstacle_profile(p) == (.4,4.,.9,550,4)
    for change in (dict(max_drive_seconds=12),dict(max_approach_distance_m=1.5),
                   dict(max_linear_speed_mps=.5),dict(obstacle_profile='unbounded')):
        with pytest.raises(ValueError):
            obstacle_profile(dict(p,**change))


def test_startup_feedback_tolerance_requires_the_scoped_obstacle_profile():
    assert obstacle_profile(dict(obstacle_profile='obstacle_04',max_linear_speed_mps=.4,
        max_drive_seconds=4,max_approach_distance_m=.9,preflight_wheel_limit_mm_s=50)) == (.4,4.,.9,550,4)
    with pytest.raises(ValueError):
        obstacle_profile(dict(preflight_wheel_limit_mm_s=50))
    with pytest.raises(ValueError):
        obstacle_profile(dict(obstacle_profile='obstacle_04',max_linear_speed_mps=.4,
            max_drive_seconds=4,max_approach_distance_m=.9,preflight_wheel_limit_mm_s=51))
