"""Start fixed-PCD ICP localization without vehicle control nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'config', 'fixed_map_localization.yaml'
            ]),
        ),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument(
            'map_file_path',
            default_value='/var/lib/mower/maps/parking_lot_2026-08-10_v2/map.pcd',
        ),
        DeclareLaunchArgument('source_topic', default_value='/cloud_registered'),
        DeclareLaunchArgument('initial_x_m', default_value='0.0'),
        DeclareLaunchArgument('initial_y_m', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_rad', default_value='0.0'),
        Node(
            package='fast_lio',
            executable='fixed_map_localizer',
            name='fixed_map_localizer',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'publish_tf': ParameterValue(
                        LaunchConfiguration('publish_tf'), value_type=bool),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'map_file_path': LaunchConfiguration('map_file_path'),
                    'source_topic': LaunchConfiguration('source_topic'),
                    'initial_x_m': ParameterValue(
                        LaunchConfiguration('initial_x_m'), value_type=float),
                    'initial_y_m': ParameterValue(
                        LaunchConfiguration('initial_y_m'), value_type=float),
                    'initial_yaw_rad': ParameterValue(
                        LaunchConfiguration('initial_yaw_rad'), value_type=float),
                },
            ],
            output='screen',
        ),
    ])
