from pathlib import Path
import ast


ROOT = Path(__file__).resolve().parents[1]


def test_fixed_map_localizer_requires_a_pcd_and_defaults_to_observation_only():
    source = (ROOT / 'src' / 'fixed_map_localizer.cpp').read_text()
    config = (ROOT / 'config' / 'fixed_map_localization.yaml').read_text()
    assert 'map_file_path is required' in source
    assert 'unable to load non-empty PCD' in source
    assert 'publish_tf: false' in config
    assert 'max_fitness_score: 0.10' in config
    assert 'source_topic: /cloud_registered' in config


def test_localization_uses_measured_mid360_vehicle_extrinsic():
    launch = (ROOT / '..' / 'mcu_can_bridge' / 'launch' / 'localization.launch.py').read_text()
    assert "arguments=['0.17', '0.0', '0.48'" in launch
    assert "DeclareLaunchArgument('rtk_x_m', default_value='-0.17')" in launch
    assert "DeclareLaunchArgument('rtk_y_m', default_value='-0.325')" in launch
    assert "DeclareLaunchArgument('rtk_z_m', default_value='0.70')" in launch


def test_fixed_map_localizer_rejects_bad_icp_before_tf_publication():
    source = (ROOT / 'src' / 'fixed_map_localizer.cpp').read_text()
    assert 'if (!icp.hasConverged() || !std::isfinite(raw_fitness)' in source
    assert 'raw_fitness > get_parameter("max_fitness_score").as_double()' in source
    assert source.index('if (!icp.hasConverged()') < source.index('estimate_ = candidate')
    assert 'constrained_fitness > get_parameter("max_fitness_score").as_double()' in source
    assert 'if (!tf_broadcaster_)' in source


def test_fixed_map_localizer_constrains_ground_vehicle_updates():
    source = (ROOT / 'src' / 'fixed_map_localizer.cpp').read_text()
    config = (ROOT / 'config' / 'fixed_map_localization.yaml').read_text()
    assert 'declare_parameter<bool>("planar_mode", true)' in source
    assert 'planar_mode: true' in config
    assert 'get_parameter("planar_mode").as_bool() && has_accepted_alignment' in source
    assert 'candidate.translation().z() = reference.translation().z()' in source
    assert 'Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())' in source


def test_fixed_map_localizer_gates_updates_after_initial_alignment():
    source = (ROOT / 'src' / 'fixed_map_localizer.cpp').read_text()
    config = (ROOT / 'config' / 'fixed_map_localization.yaml').read_text()
    assert 'max_translation_step_m: 0.10' in config
    assert 'max_yaw_step_rad: 0.0872665' in config
    assert 'max_initial_translation_m: 1.0' in config
    assert 'max_initial_yaw_rad: 0.523599' in config
    assert 'has_accepted_alignment_' in source
    assert 'translation_step > get_parameter("max_translation_step_m").as_double()' in source
    assert 'yaw_step > get_parameter("max_yaw_step_rad").as_double()' in source
    assert 'translation_step > get_parameter("max_initial_translation_m").as_double()' in source
    assert source.index('if (has_accepted_alignment)') < source.index('estimate_ = candidate')


def test_fixed_map_localizer_heartbeats_last_accepted_tf_and_reports_rejections():
    source = (ROOT / 'src' / 'fixed_map_localizer.cpp').read_text()
    config = (ROOT / 'config' / 'fixed_map_localization.yaml').read_text()
    assert 'declare_parameter<double>("tf_heartbeat_rate_hz", 5.0)' in source
    assert 'tf_heartbeat_rate_hz: 5.0' in config
    assert 'max_tf_heartbeat_age_s: 2.5' in config
    assert 'std::bind(&FixedMapLocalizer::publish_heartbeat, this)' in source
    assert 'if (!has_accepted_alignment_)' in source
    assert 'publish_transform(get_clock()->now(), estimate_)' in source
    assert 'publish_estimate(msg->header.stamp, get_clock()->now(), candidate, constrained_fitness)' in source
    assert 'record_rejection(&raw_icp_rejections_, "raw_icp_rejected")' in source
    assert 'record_rejection(&constrained_fitness_rejections_, "constrained_fitness_rejected")' in source
    assert 'record_rejection(&motion_gate_rejections_, "motion_gate_rejected")' in source
    assert 'record_rejection(&initial_gate_rejections_, "initial_gate_rejected")' in source
    assert 'heartbeat_stale' in source
    assert 'awaiting_initial_alignment' in source
    assert 'MultiThreadedExecutor' in source
    assert '"/fixed_map_localization/diagnostics"' in source


def test_fixed_map_localization_launch_uses_typed_observation_switch():
    launch = ROOT / 'launch' / 'fixed_map_localization.launch.py'
    text = launch.read_text()
    ast.parse(text)
    assert 'ParameterValue(' in text
    assert "value_type=bool" in text
    assert "value_type=float" in text
    assert "DeclareLaunchArgument('odom_frame', default_value='odom')" in text
    assert "DeclareLaunchArgument('source_topic', default_value='/cloud_registered')" in text
    assert "'map_file_path': LaunchConfiguration('map_file_path')" in text
