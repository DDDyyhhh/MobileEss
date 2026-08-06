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
                FindPackageShare('mcu_can_bridge'), 'config', 'local_ekf.yaml'
            ]),
        ),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox_frame',
            arguments=['0.35', '0.0', '0.45', '0.0', '0.0', '0.0',
                       'base_link', 'livox_frame'],
            output='screen',
        ),
        Node(
            package='mcu_can_bridge',
            executable='mcu_can_bridge',
            name='mcu_can_bridge',
            output='screen',
            parameters=[{'interface': LaunchConfiguration('can_interface')}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
