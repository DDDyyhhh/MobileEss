from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SOURCE = (ROOT / 'docs/hardware/STM32/Core/UserDev/chassis_control.c').read_text()


def test_api_turn_command_compensates_the_verified_oid_yaw_polarity():
    assert 'set_chassis_velocity(host_vx, host_vy, -host_omega);' in SOURCE
