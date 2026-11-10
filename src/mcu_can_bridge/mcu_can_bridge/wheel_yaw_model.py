"""Optional field-fit wheel yaw observation; never changes motor commands."""

from dataclasses import dataclass
import math

from .protocol import body_twist_from_wheels


@dataclass(frozen=True)
class WheelTwistEstimate:
    linear_x: float
    angular_z: float
    raw_angular_z: float
    yaw_scale: float
    regime: str
    calibrated: bool


def corrected_body_twist(wheels, track_width_m, enabled=False,
                         right_spin_scale=1., right_moving_scale=1.):
    if (len(wheels) != 4 or not all(math.isfinite(v) for v in wheels) or
            not math.isfinite(track_width_m) or track_width_m <= 0 or
            not all(math.isfinite(v) and .1 <= v <= 2.
                    for v in (right_spin_scale, right_moving_scale))):
        raise ValueError('Invalid wheel observation or yaw model parameters')
    linear, raw_yaw = body_twist_from_wheels(wheels, track_width_m)
    scale, regime, calibrated = 1., 'ideal', False
    if enabled:
        regime = 'uncalibrated'
        left = (wheels[0] + wheels[2]) / 2000.
        right = (wheels[1] + wheels[3]) / 2000.
        near_spin = abs(linear) <= .02 and left * right < 0.
        # Only the measured forward/right and stationary/right regimes are fit.
        # Tiny differential noise, left turns and reverse remain unmodified.
        if raw_yaw < -.02 and (linear >= 0. or near_spin):
            mean_absolute = (abs(left) + abs(right)) / 2.
            fraction = min(1., abs(linear) / mean_absolute)
            scale = right_spin_scale * (1.-fraction) + right_moving_scale * fraction
            regime = ('right_spin' if fraction < .1 else
                      'right_forward_arc' if fraction > .9 else 'right_transition')
            calibrated = True
    return WheelTwistEstimate(linear, raw_yaw * scale, raw_yaw,
                              scale, regime, calibrated)
