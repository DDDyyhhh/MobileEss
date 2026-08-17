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
    obstacle = params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['mid360']
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']
    global_obstacle = global_costmap['obstacle_layer']['mid360']
    assert rpp['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert rpp['max_vel_x'] <= 0.7
    assert rpp['max_vel_theta'] <= 0.5
    assert rpp['transform_tolerance'] == 0.5
    assert rpp['min_vel_y'] == 0.0
    assert rpp['max_vel_y'] == 0.0
    assert rpp['vy_samples'] == 1
    assert rpp['vx_samples'] == 12
    assert rpp['vtheta_samples'] == 15
    assert rpp['critics'] == [
        'RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
        'PathAlign', 'PathDist', 'GoalDist']
    assert controller['controller_frequency'] == 10.0
    assert params['bt_navigator']['ros__parameters']['transform_tolerance'] == 0.5
    assert (params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
            == '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/'
            'navigate_to_pose_w_replanning_and_recovery.xml')
    assert params['behavior_server']['ros__parameters']['transform_tolerance'] == 0.5
    assert smoother['max_velocity'] == [0.7, 0.0, 0.5]
    assert planner['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert behaviors['spin']['plugin'] == 'nav2_behaviors/Spin'
    assert obstacle['topic'] == '/nav2/mid360_obstacles'
    assert obstacle['data_type'] == 'PointCloud2'
    assert local_costmap['width'] == 8
    assert local_costmap['height'] == 8
    assert local_costmap['transform_tolerance'] == 0.5
    assert global_costmap['transform_tolerance'] == 0.5
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']
    assert global_obstacle['topic'] == '/nav2/mid360_obstacles'
    assert global_obstacle['data_type'] == 'PointCloud2'
    assert global_obstacle['marking'] is True
    assert global_obstacle['clearing'] is True
    assert global_costmap['inflation_layer']['inflation_radius'] == 0.90
    assert global_costmap['inflation_layer']['cost_scaling_factor'] == 1.8
    assert local_costmap['inflation_layer']['inflation_radius'] >= local_costmap['robot_radius']
    assert local_costmap['inflation_layer']['cost_scaling_factor'] == 5.0


def test_motion_is_opt_in_and_nav2_isolated_from_the_chassis_command_topic():
    launch = (ROOT / 'launch' / 'mower_navigation.launch.py').read_text()
    assert "DeclareLaunchArgument('nav2_params_file'" in launch
    assert "'params_file': LaunchConfiguration('nav2_params_file')" in launch
    assert "DeclareLaunchArgument('enable_motion', default_value='false')" in launch
    assert "DeclareLaunchArgument(\n            'safety_map_approved', default_value='false'" in launch
    assert "'autostart': 'true'" in launch
    assert "command_topic: /cmd_vel" in (ROOT / 'config' / 'cmd_vel_interface.yaml').read_text()
    assert "localization.launch.py" in launch
    assert "fast_lio_real.launch.py" in launch
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
    assert obstacle_params['livox_obstacle_cloud']['ros__parameters']['point_stride'] == 4
    assert "self._point_stride < 1" in converter
    assert 'self._cloud_callback, 1)' in converter
