from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_nav2_limits_and_livox_obstacle_layer_contract():
    params = yaml.safe_load((ROOT / 'config' / 'nav2_params.yaml').read_text())
    assert params['amcl']['ros__parameters']['tf_broadcast'] is False
    assert params['map_server']['ros__parameters']['yaml_filename'] == ''
    rpp = params['controller_server']['ros__parameters']['FollowPath']
    controller = params['controller_server']['ros__parameters']
    smoother = params['velocity_smoother']['ros__parameters']
    planner = params['planner_server']['ros__parameters']['GridBased']
    behaviors = params['behavior_server']['ros__parameters']
    timed_layer = params['local_costmap']['local_costmap']['ros__parameters']['timed_dynamic_layer']
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']
    assert rpp['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert rpp['time_steps'] == 13
    assert rpp['model_dt'] == 0.20
    assert rpp['time_steps'] * rpp['model_dt'] == 2.6
    assert rpp['batch_size'] == 300
    assert rpp['vx_max'] == 0.35
    assert rpp['ObstaclesCritic.consider_footprint'] is False
    assert rpp['ObstaclesCritic.collision_margin_distance'] == 0.20
    assert rpp['ObstaclesCritic.inflation_radius'] == 0.65
    assert rpp['ObstaclesCritic.cost_scaling_factor'] == 5.0
    assert rpp['ObstaclesCritic.repulsion_weight'] == 3.0
    assert 'ObstaclesCritic.near_collision_distance' not in rpp
    assert rpp['CostCritic.consider_footprint'] is False
    assert rpp['CostCritic.inflation_layer_name'] == 'inflation_layer'
    assert rpp['PathAlignCritic.cost_weight'] == 3.0
    assert rpp['PathFollowCritic.cost_weight'] == 3.0
    assert rpp['wz_max'] == 0.50
    assert rpp['az_max'] == 0.50
    assert rpp['wz_std'] == 0.40
    assert rpp['visualize'] is False
    assert rpp['vx_std'] == 0.20
    assert rpp['wz_std'] == 0.40
    assert rpp['TrajectoryVisualizer.trajectory_step'] == 50
    assert rpp['TrajectoryVisualizer.time_step'] == 5
    assert rpp['retry_attempt_limit'] == 0
    assert rpp['batch_size'] * rpp['time_steps'] * controller['controller_frequency'] == 19500
    assert controller['controller_frequency'] == 5.0
    assert controller['controller_frequency'] >= 1.0 / rpp['model_dt']
    assert controller['failure_tolerance'] == 0.0
    assert params['bt_navigator']['ros__parameters']['transform_tolerance'] == 0.5
    assert (params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
            == '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/'
            'navigate_to_pose_w_replanning_and_recovery.xml')
    assert params['behavior_server']['ros__parameters']['transform_tolerance'] == 0.5
    assert smoother['max_velocity'] == [0.7, 0.0, 0.5]
    assert planner['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert behaviors['spin']['plugin'] == 'nav2_behaviors/Spin'
    assert local_costmap['plugins'] == ['timed_dynamic_layer', 'inflation_layer']
    assert local_costmap['robot_radius'] == 0.45
    assert local_costmap['inflation_layer']['inflation_radius'] == 0.65
    assert timed_layer['plugin'] == 'timed_dynamic_obstacle_layer::TimedDynamicObstacleLayer'
    assert timed_layer['topic'] == '/nav2/mid360_obstacles'
    assert timed_layer['ttl_seconds'] == 0.80
    assert timed_layer['obstacle_cell_size'] == 0.05
    assert timed_layer['transform_tolerance'] == 0.5
    assert timed_layer['source_timeout'] == 0.6
    assert 'obstacle_layer' not in local_costmap
    assert local_costmap['width'] == 8
    assert local_costmap['height'] == 8
    assert local_costmap['transform_tolerance'] == 0.5
    assert global_costmap['transform_tolerance'] == 0.5
    assert global_costmap['plugins'] == ['static_layer', 'inflation_layer']
    assert 'obstacle_layer' not in global_costmap
    assert global_costmap['inflation_layer']['inflation_radius'] == 0.65
    assert global_costmap['inflation_layer']['cost_scaling_factor'] == 4.0
    assert local_costmap['inflation_layer']['inflation_radius'] >= local_costmap['robot_radius']
    assert local_costmap['inflation_layer']['cost_scaling_factor'] == 5.0



def test_dwb_comparison_profile_is_low_speed_and_position_only():
    params = yaml.safe_load((ROOT / 'config' / 'nav2_params_dwb_comparison.yaml').read_text())
    controller = params['controller_server']['ros__parameters']
    dwb = controller['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    assert controller['controller_frequency'] == 20.0
    assert controller['goal_checker_plugins'] == ['position_goal_checker']
    assert controller['position_goal_checker']['plugin'] == 'nav2_controller::PositionGoalChecker'
    assert dwb['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert dwb['max_vel_x'] == 0.20
    assert dwb['max_vel_theta'] == 0.50
    assert dwb['max_speed_xy'] == 0.20
    assert dwb['critics'] == [
        'RotateToGoal', 'Oscillation', 'ObstacleFootprint',
        'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist']
    assert smoother['max_velocity'] == [0.20, 0.0, 0.50]
    assert params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'].endswith(
        'navigate_to_pose_dwb_comparison.xml')


def test_dwb_box_lookahead_trial_changes_only_simulation_time():
    base = yaml.safe_load((ROOT / 'config' / 'nav2_params_dwb_box_0p15.yaml').read_text())
    trial = yaml.safe_load((ROOT / 'config' / 'nav2_params_dwb_box_0p15_lookahead_2s.yaml').read_text())
    base_dwb = base['controller_server']['ros__parameters']['FollowPath']
    trial_dwb = trial['controller_server']['ros__parameters']['FollowPath']
    deltas = {
        key: (base_dwb.get(key), trial_dwb.get(key))
        for key in set(base_dwb) | set(trial_dwb)
        if base_dwb.get(key) != trial_dwb.get(key)}
    assert deltas == {'sim_time': (1.0, 2.0)}
    assert trial_dwb['sim_time'] * trial_dwb['max_vel_x'] == 0.30
    assert trial_dwb['vtheta_samples'] == 11


def test_dwb_box_angular_resolution_trial_changes_only_vtheta_samples():
    base = yaml.safe_load((ROOT / 'config' / 'nav2_params_dwb_box_0p15_lookahead_2s.yaml').read_text())
    trial = yaml.safe_load(
        (ROOT / 'config' / 'nav2_params_dwb_box_0p15_lookahead_2s_vtheta21.yaml').read_text())
    base_dwb = base['controller_server']['ros__parameters']['FollowPath']
    trial_dwb = trial['controller_server']['ros__parameters']['FollowPath']
    deltas = {
        key: (base_dwb.get(key), trial_dwb.get(key))
        for key in set(base_dwb) | set(trial_dwb)
        if base_dwb.get(key) != trial_dwb.get(key)}
    assert deltas == {'vtheta_samples': (11, 21)}
    assert trial_dwb['sim_time'] == 2.0
    assert trial_dwb['max_vel_x'] == 0.15
    assert trial_dwb['max_vel_theta'] == 0.50


def test_dwb_comparison_bt_does_not_issue_automatic_spin_recovery():
    bt = (ROOT / 'behavior_trees' / 'navigate_to_pose_dwb_comparison.xml').read_text()
    assert '<FollowPath path="{path}" controller_id="FollowPath"/>' in bt
    assert '<Spin' not in bt
    assert '<BackUp' not in bt

    launch = (ROOT / 'launch' / 'mower_navigation.launch.py').read_text()
    assert "DeclareLaunchArgument('nav2_params_file'" in launch
    assert "'params_file': LaunchConfiguration('nav2_params_file')" in launch
    assert "DeclareLaunchArgument('enable_motion', default_value='false')" in launch
    assert "DeclareLaunchArgument(\n            'safety_map_approved', default_value='false'" in launch
    assert "'autostart': 'true'" in launch
    assert "command_topic: /cmd_vel_gated" in (ROOT / 'config' / 'cmd_vel_interface.yaml').read_text()
    assert "executable='collision_gate'" in launch
    assert "output_topic': '/cmd_vel_gated'" in launch
    assert "localization.launch.py" in launch
    assert "fast_lio_real.launch.py" in launch
    package_xml = (ROOT / 'package.xml').read_text()
    assert '<exec_depend>nav2_mppi_controller</exec_depend>' in package_xml
    assert '<exec_depend>diagnostic_msgs</exec_depend>' in package_xml
    assert "fast_lio_config_file" in launch
    converter = (ROOT / 'mower_nav2' / 'livox_obstacle_cloud.py').read_text()
    obstacle_params = yaml.safe_load(
        (ROOT / 'config' / 'livox_obstacle_cloud.yaml').read_text())
    assert "CustomMsg" in converter
    assert "PointCloud2" in converter
    assert 'self_filter_boxes' in converter
    assert "'livox_obstacle_params_file'" in launch
    assert "parameters=[LaunchConfiguration('livox_obstacle_params_file')]" in launch
    boxes = obstacle_params['livox_obstacle_cloud']['ros__parameters']['self_filter_boxes']
    assert boxes == [-0.55, -0.18, -0.42, 0.42, -0.15, 1.20]
    livox_params = obstacle_params['livox_obstacle_cloud']['ros__parameters']
    assert livox_params['point_stride'] == 4
    assert livox_params['output_topic'] == '/nav2/mid360_obstacles'
    assert 'clearing_topic' not in livox_params
    assert livox_params['stamp_with_receipt_time'] is True
    assert livox_params['max_range'] == 4.0
    assert livox_params['ground_band'] == 0.10
    assert livox_params['min_points_per_cell'] == 3
    assert 'filter_livox_costmap_evidence' in converter
    assert 'TimedDynamicObstacleLayer' not in converter
    assert 'self._clearing_publisher' not in converter
    assert 'self._cloud_callback, 1)' in converter
    capture = (ROOT / 'scripts' / 'capture_mppi_observation.sh').read_text()
    assert "node_list=\"$(ros2 node list)\"" in capture
    assert 'node_exists()' in capture
    assert "grep -Fxq \"$1\" <<< \"$node_list\"" in capture
    assert "grep -Fxq '/cmd_vel_can_bridge'" not in capture
    assert "pgrep -fo '(/|^)controller_server" in capture
    assert 'FollowPath.visualize true' in capture
    assert 'FollowPath.visualize false' in capture
    assert '/trajectories' in capture
    assert 'does not publish /local_plan' in capture
    assert '/cmd_vel_nav' in capture
    assert 'controller_top.txt' in capture
    assert 'mode must be \'performance\' or \'visualize\'' in capture
    assert 'duration must be below 15 s' in capture
    assert '/collision_gate/diagnostics' in capture
    assert '--include-hidden-topics' in capture
    assert 'first_cmd_vel_nav.yaml' in capture
    assert 'Waiting up to ${goal_timeout_sec}s for an active MPPI /cmd_vel_nav command' in capture
    assert '/follow_path/_action/status' in capture
