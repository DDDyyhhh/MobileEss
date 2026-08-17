from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mobileess_rtk'), 'config', 'um982.example.yaml'
            ]),
        ),
        Node(
            package='mobileess_rtk',
            executable='rtk_node',
            name='um982_rtk',
            output='screen',
            parameters=[params_file],
        ),
    ])
