import math
import json
from types import SimpleNamespace as N

import pytest

from mower_nav2.obstacle_wait import (
    ObstacleWait, finish_controller_pause, goal_uuid, remaining_straight_path)


def advance(policy, begin, end, *, blocked, stationary=True):
    for i in range(round((end-begin)*10)+1):
        state = policy.update(round(begin+i*.1, 8), blocked=blocked, stationary=stationary)
    return state


def test_clear_path_does_not_create_a_wait_or_resume():
    p = ObstacleWait()
    assert advance(p, 0., 10., blocked=False) == 'following'
    assert p.resume_count == 0


def test_waits_beyond_nav2_progress_allowance_then_resumes_once():
    p = ObstacleWait()
    assert advance(p, 0., 20., blocked=True) == 'waiting'
    assert advance(p, 20.1, 22., blocked=False) == 'waiting'
    assert p.update(22.1, blocked=False, stationary=True) == 'resume'
    assert p.resume_count == 1
    assert p.update(22.2, blocked=False, stationary=True) == 'resume'
    p.acknowledge_resume()
    assert p.update(22.3, blocked=True, stationary=True) == 'fault'


def test_flickering_clear_observations_restart_clear_interval():
    p = ObstacleWait()
    advance(p, 0., 5., blocked=True)
    assert advance(p, 5.1, 7., blocked=False) == 'waiting'
    assert p.update(7.1, blocked=True, stationary=True) == 'waiting'
    assert advance(p, 7.2, 9.1, blocked=False) == 'waiting'
    assert p.update(9.3, blocked=False, stationary=True) == 'resume'


def test_wheels_must_be_continuously_stopped_before_resume():
    p = ObstacleWait()
    advance(p, 0., 5., blocked=True)
    assert advance(p, 5.1, 8., blocked=False, stationary=False) == 'waiting'
    assert advance(p, 8.1, 9., blocked=False) == 'waiting'
    assert p.update(9.1, blocked=False, stationary=True) == 'resume'


@pytest.mark.parametrize('fault', ['unhealthy', 'gap', 'backward', 'nan'])
def test_faults_cannot_auto_recover_when_data_returns(fault):
    p = ObstacleWait()
    p.update(1., blocked=True, stationary=True)
    t = {'unhealthy': 1.1, 'gap': 1.31, 'backward': .9, 'nan': math.nan}[fault]
    assert p.update(t, blocked=False, stationary=True, healthy=fault != 'unhealthy') == 'fault'
    assert p.update(1.2, blocked=False, stationary=True) == 'fault'
    assert p.resume_count == 0


def test_wait_expiry_is_terminal_even_if_obstacle_then_disappears():
    p = ObstacleWait()
    assert advance(p, 0., 30., blocked=True) == 'fault'
    assert 'expired' in p.error
    assert p.update(30.1, blocked=False, stationary=True) == 'fault'


def test_resume_retains_same_goal_without_replaying_passed_route():
    path = [[i/10., 0., 0.] for i in range(36)]
    remaining = remaining_straight_path(path, [1.23, .05, .01])
    assert remaining[0] == [1.23, .05, .01]
    assert remaining[1][0] == 1.3
    assert remaining[-1] == path[-1]
    assert all(x[0] > 1.23 for x in remaining[1:])
    for pose in ([1., .21, 0.], [1., 0., .16], [3.4, 0., 0.]):
        with pytest.raises(ValueError):
            remaining_straight_path(path, pose)


def test_clearance_loss_while_resume_pending_terminates():
    p = ObstacleWait()
    advance(p, 0., 5., blocked=True)
    assert advance(p, 5.1, 7.2, blocked=False) == 'resume'
    assert p.update(7.3, blocked=True, stationary=True) == 'fault'
    with pytest.raises(ValueError):
        p.acknowledge_resume()


def test_straight_review_rejects_changed_wait_budget_or_a_curve():
    from mower_nav2.reviewed_route import validate_review
    path = [[i/10., 0., 0.] for i in range(36)]
    proposal = dict(route_profile='static_obstacle_wait_straight_03',
        max_linear_speed_mps=.3, max_angular_speed_radps=.2,
        corridor_half_width_m=.2, max_run_seconds=75., max_travel_m=6.,
        review_radius_m=.98, physical_whole_route_confirmed=True,
        pose=dict(xy_m=[0., 0.], yaw_deg=0.), goal_xy_m=[3.5, 0.], goal_yaw_deg=0.,
        path_xy_yaw_rad=path, obstacle_wait_policy=dict(max_wait_seconds=30.,
            minimum_wait_seconds=5., clear_seconds=2., stationary_seconds=1., maximum_resumes=1))
    validate_review(proposal)
    proposal['obstacle_wait_policy']['maximum_resumes'] = 2
    with pytest.raises(ValueError, match='scope'):
        validate_review(proposal)
    proposal['obstacle_wait_policy']['maximum_resumes'] = 1
    path[10][1] = .03
    with pytest.raises(ValueError, match='straight'):
        validate_review(proposal)


@pytest.mark.parametrize('ack, status, valid', [(True, 5, True),
    (False, 5, False), (True, 6, False), (True, 4, False), (True, 2, False)])
def test_pause_requires_matching_ack_and_final_cancel_not_failure_recovery(ack, status, valid):
    uid = [7]*16
    handle = N(goal_id=N(uuid=uid), cancel_goal_async=lambda: 'cancel-request')
    pumped = []
    def pump(future, timeout):
        pumped.append(future)
        if future == 'cancel-request':
            return N(goals_canceling=[N(goal_id=N(uuid=uid if ack else [8]*16))])
        return N(status=status)
    if valid:
        finish_controller_pause(handle, 'finished', pump)
        assert pumped == ['cancel-request', 'finished']
    else:
        with pytest.raises(RuntimeError):
            finish_controller_pause(handle, 'finished', pump)


def test_humble_uint8_uuid_can_be_saved_as_complete_json():
    np = pytest.importorskip('numpy')
    handle = N(goal_id=N(uuid=np.arange(16, dtype=np.uint8)))
    assert json.loads(json.dumps(dict(uuid=goal_uuid(handle))))['uuid'] == list(range(16))
