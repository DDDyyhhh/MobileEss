import math

import pytest

from mower_nav2.forward_right_sequence import second_leg_proposal, waypoints


def example():
    pose = {'xy_m':[2.,1.], 'yaw_deg':0.}
    first, second, _ = waypoints(pose)
    review = dict(sequence_profile='forward_then_right_04', motion_approved=True,
                  pose=pose, second_target_xy_m=second,
                  leg_template={'navigation_profile':'short_04'})
    result = dict(completed=True, action_status=4,
                  final_pose=[first[0]-.1,first[1],0.],
                  final_api_mode=2, final_wheel_mm_s=[0,0,0,0])
    return review, result


def test_waypoints_advance_before_turning_right_in_the_robot_frame():
    first, second, yaw = waypoints({'xy_m':[0.,0.], 'yaw_deg':90.})
    assert first == pytest.approx([0.,.6])
    assert second[0] > first[0] and second[1] > first[1]
    assert yaw == pytest.approx(55.)


def test_second_goal_stays_fixed_while_its_start_tracks_the_verified_stop():
    review, stop = example()
    p = second_leg_proposal(review, stop)
    assert p['proposed_target_xy_m'] == review['second_target_xy_m']
    assert p['pose']['xy_m'] == stop['final_pose'][:2]
    assert .5 <= p['distance_m'] <= 1.
    assert p['require_initial_heading_alignment']
    assert not review['leg_template'].get('motion_approved')


@pytest.mark.parametrize('change', [
    {'completed':False}, {'action_status':6}, {'final_api_mode':0},
    {'final_wheel_mm_s':[0,0,11,0]}, {'final_pose':[2.2,1.,0.]},
    {'final_pose':[2.5,1.,.2]}, {'final_pose':[math.nan,1.,0.]},
])
def test_failed_moving_or_unreviewed_first_stop_never_advances(change):
    review, stop = example(); stop.update(change)
    with pytest.raises(ValueError):second_leg_proposal(review, stop)


def test_unapproved_or_changed_final_goal_never_advances():
    review, stop = example(); review['motion_approved'] = False
    with pytest.raises(ValueError):second_leg_proposal(review, stop)
    review['motion_approved'] = True; review['second_target_xy_m'][0] += .1
    with pytest.raises(ValueError):second_leg_proposal(review, stop)
