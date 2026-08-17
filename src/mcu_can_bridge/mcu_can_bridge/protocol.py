"""Pure Python helpers for the STM32 0x50A wheel-speed frame."""

from __future__ import annotations

import math
from typing import Tuple


WHEEL_FEEDBACK_CAN_ID = 0x50A
WHEEL_FEEDBACK_DLC = 8


def localization_is_accepted(message: str) -> bool:
    """Return whether the fixed-map localizer reports an accepted alignment."""
    return message == 'accepted'


def remote_takeover_detected(api_was_running: bool, api_state: int) -> bool:
    """Detect an STM32 API-state drop after the bridge owned API run."""
    return api_was_running and api_state != 2


def requested_api_state(api_was_running: bool, reported_state: int) -> int:
    """Keep API run requested after status telemetry briefly goes stale."""
    if api_was_running:
        return 2
    return 1 if reported_state not in (1, 2) else 2


def quantize_command_speed(value: float, limit: float) -> int:
    """Quantize and clamp a velocity for STM32's signed 0.1-unit field."""
    if limit <= 0.0 or limit > 1.0:
        raise ValueError('limit must be within the STM32 1.0-unit API range')
    if not math.isfinite(value):
        return 0
    return max(-int(limit * 10), min(int(limit * 10), int(round(value * 10))))


def parse_wheel_feedback(data: bytes) -> Tuple[int, int, int, int]:
    """Decode four big-endian signed int16 speeds in mm/s."""
    if len(data) != WHEEL_FEEDBACK_DLC:
        raise ValueError(f'0x50A requires 8 data bytes, got {len(data)}')
    return tuple(
        int.from_bytes(data[offset:offset + 2], byteorder='big', signed=True)
        for offset in range(0, WHEEL_FEEDBACK_DLC, 2)
    )  # type: ignore[return-value]


def body_twist_from_wheels(
    speeds_mmps: Tuple[int, int, int, int], track_width_m: float
) -> Tuple[float, float]:
    """Return (linear.x m/s, angular.z rad/s) from FL, FR, RL, RR."""
    if track_width_m <= 0.0:
        raise ValueError('track_width_m must be positive')
    speed_fl, speed_fr, speed_rl, speed_rr = speeds_mmps
    left_mmps = speed_fl + speed_rl
    right_mmps = speed_fr + speed_rr
    linear_x = (left_mmps + right_mmps) / 4000.0
    angular_z = (right_mmps - left_mmps) / (2.0 * track_width_m * 1000.0)
    return linear_x, angular_z
