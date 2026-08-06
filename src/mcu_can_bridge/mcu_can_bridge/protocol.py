"""Pure Python helpers for the STM32 0x50A wheel-speed frame."""

from __future__ import annotations

from typing import Tuple


WHEEL_FEEDBACK_CAN_ID = 0x50A
WHEEL_FEEDBACK_DLC = 8


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
