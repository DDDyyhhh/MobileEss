from mcu_can_bridge.protocol import (
    localization_is_accepted,
    quantize_command_speed,
    remote_takeover_detected,
    requested_api_state,
)


def test_localization_acceptance_uses_the_diagnostic_status_message():
    assert localization_is_accepted('accepted')
    assert not localization_is_accepted('heartbeat_stale')
    assert not localization_is_accepted('raw_icp_rejected')


def test_remote_takeover_requires_a_confirmed_api_run_then_state_drop():
    assert not remote_takeover_detected(False, 0)
    assert not remote_takeover_detected(False, 1)
    assert not remote_takeover_detected(True, 2)
    assert remote_takeover_detected(True, 1)
    assert remote_takeover_detected(True, 0)


def test_status_telemetry_gap_does_not_restart_api_handshake():
    assert requested_api_state(False, 0) == 1
    assert requested_api_state(False, 1) == 2
    assert requested_api_state(True, 2) == 2
    # A confirmed run persists through a status timeout; nonzero velocity is
    # still gated separately by status freshness.
    assert requested_api_state(True, 0) == 2


def test_raw_speed_clamps_to_documented_api_limit():
    assert quantize_command_speed(0.74, 0.7) == 7
    assert quantize_command_speed(-0.74, 0.7) == -7
    assert quantize_command_speed(0.51, 0.5) == 5
    assert quantize_command_speed(-0.51, 0.5) & 0xFF == 0xFB


def test_nonfinite_command_is_converted_to_zero_speed():
    assert quantize_command_speed(float('nan'), 0.7) == 0
    assert quantize_command_speed(float('inf'), 0.5) == 0
