from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]

def test_unvalidated_wheel_yaw_cannot_override_imu_lio_heading():
 p=yaml.safe_load((ROOT/'config/local_ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
 assert p['twist0_config'][6] is True and p['twist0_config'][11] is False
 assert p['imu0_config'][11] is True and p['odom0_config'][5] is True
 assert p['publish_tf'] is True and p['world_frame']=='odom'

def test_right_fit_is_explicit_and_has_conservative_uncertainty():
 p=yaml.safe_load((ROOT/'config/wheel_odometry.yaml').read_text())['mcu_can_bridge']['ros__parameters']
 assert p['right_yaw_model_enabled'] is True
 assert .1<p['right_moving_yaw_scale']<1<p['right_spin_yaw_scale']<2
 assert p['angular_z_variance']>=.04
 assert 'wheel_params_file' in (ROOT/'launch/localization.launch.py').read_text()
