import pytest

from mcu_can_bridge.protocol import body_twist_from_wheels, parse_wheel_feedback


def test_parse_50a_big_endian_signed_wheel_speeds():
    assert parse_wheel_feedback(bytes.fromhex('048B 048A 048B 048C')) == (
        1163, 1162, 1163, 1164)
    assert parse_wheel_feedback(bytes.fromhex('FCCD FCD6 FCD3 FCD5')) == (
        -819, -810, -813, -811)


def test_four_wheel_average_and_yaw_rate_use_690_mm_track():
    linear_x, angular_z = body_twist_from_wheels((1159, 1158, 1159, 1160), 0.690)
    assert linear_x == pytest.approx(1.159)
    assert angular_z == pytest.approx(0.0)


def test_in_place_left_turn_has_negative_left_and_positive_right_rate():
    linear_x, angular_z = body_twist_from_wheels((-387, 399, -387, 399), 0.690)
    assert linear_x == pytest.approx(0.006)
    assert angular_z == pytest.approx(1.1391304)


def test_invalid_dlc_is_rejected():
    with pytest.raises(ValueError):
        parse_wheel_feedback(b'\x00' * 7)
