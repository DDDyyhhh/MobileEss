import pytest
from mcu_can_bridge.bench_control import bench_payload


@pytest.mark.parametrize('mode', [0, 1])
def test_no_motion_before_run_and_no_auxiliary_change_or_unlock(mode):
    data = bench_payload(mode, 'forward', 3, 1)
    assert data == bytes((mode, 0, 0, 0, 0, 3, 1, 0))


def test_fixed_low_speed_and_positive_ros_left_with_blade_off():
    assert bench_payload(2, 'forward', 2, 0) == bytes((2, 1, 0, 0, 0, 2, 0, 0))
    assert bench_payload(2, 'left', 2, 0) == bytes((2, 0, 0, 2, 0, 2, 0, 0))
    with pytest.raises(ValueError):
        bench_payload(2, 'forward', 0, 0)

from mcu_can_bridge.bench_control import watchdog_evidence


def telemetry(t, can_id, values):
    if can_id == '0x507':
        payload = bytes((0, 0, 0, 0, values, 0, 0, 0))
    else:
        payload = b''.join(v.to_bytes(2, 'big', signed=True) for v in values)
    return {'t_s': t, 'direction': 'rx', 'can_id': can_id, 'payload': payload.hex()}


def stopped_sequence():
    return [telemetry(0.10, '0x507', 2), telemetry(0.53, '0x507', 0),
            telemetry(0.54, '0x50a', (35, 35, 35, 35)),
            telemetry(0.60, '0x50a', (0, 0, 0, 0)),
            telemetry(0.66, '0x50a', (0, 0, 0, 0)),
            telemetry(0.72, '0x50a', (0, 0, 0, 0)),
            telemetry(1.9, '0x507', 0)]


def test_watchdog_requires_mode_exit_and_sustained_real_wheel_stop():
    evidence = watchdog_evidence(stopped_sequence(), 0)
    assert evidence['watchdog_stopped']
    assert evidence['api_exit_after_last_tx_s'] == pytest.approx(0.53)
    assert evidence['settled_after_last_tx_s'] == pytest.approx(0.72)


@pytest.mark.parametrize('failure', ['no_mode_exit', 'late_exit', 'transient_zero', 'resume', 'tx'])
def test_watchdog_rejects_missing_late_or_unstable_stop_and_heartbeat(failure):
    events = stopped_sequence()
    if failure == 'no_mode_exit':
        events = [e for e in events if e['can_id'] != '0x507']
    elif failure == 'late_exit':
        events[1]['t_s'] = 1.2
    elif failure == 'transient_zero':
        events.append(telemetry(1.9, '0x50a', (60, 60, 60, 60)))
    elif failure == 'resume':
        events.append(telemetry(1.95, '0x507', 2))
    else:
        events.append({'t_s': 0.7, 'direction': 'tx', 'can_id': '0x5', 'payload': '0201000000010000'})
    assert not watchdog_evidence(events, 0)['watchdog_stopped']


@pytest.mark.parametrize('confirmation,passed', [(1.02, True), (1.5, True), (1.50001, False)])
def test_stop_confirmation_uses_approved_1_5s_deadline(confirmation, passed):
    events = [telemetry(0.53, '0x507', 0),
              telemetry(confirmation-0.12, '0x50a', (0, 0, 0, 0)),
              telemetry(confirmation-0.06, '0x50a', (0, 0, 0, 0)),
              telemetry(confirmation, '0x50a', (0, 0, 0, 0)),
              telemetry(1.9, '0x507', 0)]
    evidence = watchdog_evidence(events, 0)
    assert evidence['watchdog_stopped'] is passed
    assert evidence['stop_confirmation_deadline_s'] == 1.5
    assert evidence['api_exit_deadline_s'] == 1.0
def test_revolution_profile_is_bounded_and_preserves_normal_trials():
    from mcu_can_bridge.bench_control import bench_motion_profile, bench_payload
    assert bench_motion_profile('wheel-revolution') == ('forward', 5.0)
    assert bench_payload(2, bench_motion_profile('wheel-revolution')[0], 1, 0) == bytes((2, 1, 0, 0, 0, 1, 0, 0))
    assert bench_motion_profile('forward') == ('forward', 2.0)
    assert bench_motion_profile('left') == ('left', 2.0)
    assert bench_motion_profile('watchdog') == ('forward', 2.0)
    with pytest.raises(ValueError):
        bench_motion_profile('standby')


def test_medium_speed_probe_is_two_seconds_and_never_moves_in_standby_or_exit():
    from mcu_can_bridge.bench_control import bench_motion_profile
    assert bench_motion_profile('forward-02') == ('forward-02', 2.0)
    assert bench_payload(2, 'forward-02', 1, 0) == bytes((2, 2, 0, 0, 0, 1, 0, 0))
    for mode in (0, 1):
        assert bench_payload(mode, 'forward-02', 1, 0)[1] == 0


def test_remote_setting_comparison_is_bounded_and_never_unlocks():
    from mcu_can_bridge.bench_control import bench_motion_profile
    assert bench_motion_profile('forward-04') == ('forward-04', 2.0)
    assert bench_payload(2, 'forward-04', 1, 0) == bytes((2, 4, 0, 0, 0, 1, 0, 0))
    for mode in (0, 1):
        assert bench_payload(mode, 'forward-04', 1, 0)[1] == 0

