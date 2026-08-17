"""Run a one-time manual FAST-LIO capture; call /map_save before stopping it."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description='Field-calibrated real Mid-360 FAST-LIO YAML.',
        ),
        DeclareLaunchArgument(
            'map_file_path',
            description='Absolute destination for this one-time map PCD.',
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio_map_capture',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'common.odom_frame': 'map_capture_odom',
                    'common.body_frame': 'livox_frame',
                    'common.odometry_topic': '/fast_lio/map_capture/odometry',
                    'publish.tf_en': False,
                    'publish.path_en': True,
                    'publish.map_en': True,
                    'pcd_save.pcd_save_en': True,
                    'pcd_save.interval': -1,
                    'map_file_path': LaunchConfiguration('map_file_path'),
                },
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
