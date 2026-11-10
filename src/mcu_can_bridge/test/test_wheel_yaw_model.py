"""The observation model must preserve raw wheel data and uncalibrated regimes."""
import math
import pytest
from mcu_can_bridge.wheel_yaw_model import corrected_body_twist


def estimate(wheels,enabled=True):
 return corrected_body_twist(wheels,.640,enabled,1.21,.44)


def test_spin_and_forward_arc_need_different_right_turn_gains():
 spin=estimate((64,-64,64,-64));arc=estimate((364,236,364,236))
 assert spin.raw_angular_z==pytest.approx(-.2)
 assert spin.angular_z==pytest.approx(-.242)
 assert arc.angular_z==pytest.approx(-.088)
 assert arc.linear_x==pytest.approx(.3)
 assert spin.regime=='right_spin' and arc.regime=='right_forward_arc'


def test_disabled_model_preserves_the_existing_wheel_observation():
 r=estimate((364,236,364,236),False)
 assert r.angular_z==r.raw_angular_z==pytest.approx(-.2)
 assert r.yaw_scale==1 and not r.calibrated


@pytest.mark.parametrize('wheels',[(300,300,300,300),(0,0,0,0),(-64,64,-64,64),(-236,-364,-236,-364),(0,-1,0,-1)])
def test_straight_zero_left_reverse_and_near_zero_noise_are_not_fitted(wheels):
 r=estimate(wheels);assert r.angular_z==r.raw_angular_z and not r.calibrated


def test_intermediate_counter_rotation_blends_continuously_without_changing_translation():
 a=estimate((190,-1,190,-1));b=estimate((190,0,190,0));c=estimate((190,1,190,1))
 assert abs(a.yaw_scale-b.yaw_scale)<.02 and abs(b.yaw_scale-c.yaw_scale)<.02
 assert a.linear_x==pytest.approx(.0945)
 assert .44<=a.yaw_scale<=1.21


@pytest.mark.parametrize('args',[(.0,1.21,.44),(.64,0,.44),(.64,1.21,float('nan')),(float('nan'),1.21,.44),(.64,3.,.44)])
def test_invalid_track_or_calibration_is_rejected(args):
 track,spin,moving=args
 with pytest.raises(ValueError):corrected_body_twist((64,-64,64,-64),track,True,spin,moving)


def test_invalid_wheels_are_not_published():
 for wheels in [(1,2,3),(1,2,math.nan,4)]:
  with pytest.raises(ValueError):estimate(wheels)


def test_tiny_negative_mean_during_counter_rotation_is_still_stationary_turn():
 r=estimate((63,-65,63,-65))
 assert r.calibrated and r.regime=='right_spin'
 assert r.linear_x==pytest.approx(-.001)
