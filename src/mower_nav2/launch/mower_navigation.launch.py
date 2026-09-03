"""Start fixed-map localization and the Nav2 stack for a supervised test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    nav2_share = FindPackageShare('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            FindPackageShare('mower_nav2'), 'maps', 'map.yaml'])),
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('mower_nav2'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('localizer_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'config', 'fixed_map_localization.yaml'])),
        DeclareLaunchArgument('map_pcd', default_value=(
            '/var/lib/mower/maps/parking_lot_2026-08-10_v2/map.pcd')),
        DeclareLaunchArgument('initial_x_m', default_value='0.0'),
        DeclareLaunchArgument('initial_y_m', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_rad', default_value='0.0'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('start_localization', default_value='true'),
        DeclareLaunchArgument('start_fast_lio', default_value='true'),
        DeclareLaunchArgument(
            'fast_lio_config_file',
            default_value='/etc/mower/fast_lio/mid360_real.yaml',
            description='Field-calibrated real-sensor FAST-LIO YAML.'),
        # Command CAN output is opt-in; bringup otherwise remains observation-only.
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument(
            'safety_map_approved', default_value='false',
            description='Set true only after the boundary, keep-out, and speed-zone maps are reviewed.'),
        DeclareLaunchArgument('cmd_vel_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('mower_nav2'), 'config', 'cmd_vel_interface.yaml'])),
        DeclareLaunchArgument('livox_obstacle_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('mower_nav2'), 'config', 'livox_obstacle_cloud.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mcu_can_bridge'), 'launch', 'localization.launch.py'])),
            launch_arguments={
                'can_interface': LaunchConfiguration('can_interface'),
                'enable_global_localization': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_localization')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'launch', 'fast_lio_real.launch.py'])),
            launch_arguments={
                'config_file': LaunchConfiguration('fast_lio_config_file'),
                'publish_tf': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_fast_lio')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'launch', 'fixed_map_localization.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('localizer_params_file'),
                'publish_tf': 'true',
                'map_file_path': LaunchConfiguration('map_pcd'),
                'initial_x_m': LaunchConfiguration('initial_x_m'),
                'initial_y_m': LaunchConfiguration('initial_y_m'),
                'initial_yaw_rad': LaunchConfiguration('initial_yaw_rad'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                nav2_share, 'launch', 'bringup_launch.py'])),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('nav2_params_file'),
                'autostart': 'true',
                'use_sim_time': 'false',
                'use_composition': 'False',
            }.items(),
        ),
        Node(
            package='mower_nav2',
            executable='livox_obstacle_cloud',
            name='livox_obstacle_cloud',
            parameters=[LaunchConfiguration('livox_obstacle_params_file')],
            output='screen',
        ),
        Node(
            package='mower_nav2',
            executable='collision_gate',
            name='collision_gate',
            parameters=[{
                'input_topic': '/cmd_vel',
                'output_topic': '/cmd_vel_gated',
                'costmap_topic': '/local_costmap/costmap_raw',
                'odom_topic': '/odometry/filtered',
                'robot_radius': 0.45,
                'prediction_horizon': 2.0,
                'prediction_dt': 0.05,
                'command_timeout': 0.25,
                'costmap_timeout': 0.30,
                'odom_timeout': 0.30,
                'collision_cost': 253,
            }],
            output='screen',
        ),
        Node(
            package='mcu_can_bridge',
            executable='cmd_vel_can_bridge',
            name='cmd_vel_can_bridge',
            parameters=[LaunchConfiguration('cmd_vel_params_file')],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_motion'), "' == 'true' and '",
                LaunchConfiguration('safety_map_approved'), "' == 'true'",
            ])),
            output='screen',
        ),
    ])
