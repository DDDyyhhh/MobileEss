from pathlib import Path


CONFIG = Path(__file__).parents[1] / 'config' / 'global_localization.yaml'


def _config_block() -> str:
    text = CONFIG.read_text()
    return text.split('/global/global_ekf_filter_node:', 1)[1]


def test_global_ekf_config_matches_namespaced_node():
    assert '/global/global_ekf_filter_node:' in CONFIG.read_text()


def test_dual_antenna_heading_is_required_for_map_to_odom_tf():
    assert 'publish_tf: false' in _config_block()
    assert 'imu0: /rtk/heading' in _config_block()


def test_global_ekf_uses_local_velocity_not_local_odom_pose():
    block = _config_block()
    assert 'odom0_config: [false, false, false,' in block
    assert 'false, false, false,\n                   true, false, false,' in block
    assert 'false, false, true,' in block
    assert 'odom0_relative: false' in block
