from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    global_params_file = LaunchConfiguration('global_params_file')
    rtk_params_file = LaunchConfiguration('rtk_params_file')
    global_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('enable_global_localization'),
        "' == 'true' and '", LaunchConfiguration('rtk_extrinsics_configured'),
        "' == 'true'"
    ]))
    rtk_node_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('enable_global_localization'),
        "' == 'true' and '", LaunchConfiguration('rtk_extrinsics_configured'),
        "' == 'true' and '", LaunchConfiguration('start_rtk_node'),
        "' == 'true'"
    ]))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mcu_can_bridge'), 'config', 'local_ekf.yaml'
            ]),
        ),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('enable_can_feedback', default_value='true'),
        DeclareLaunchArgument('wheel_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('mcu_can_bridge'), 'config', 'wheel_odometry.yaml'])),
        DeclareLaunchArgument('enable_global_localization', default_value='false'),
        DeclareLaunchArgument('rtk_extrinsics_configured', default_value='false'),
        DeclareLaunchArgument('start_rtk_node', default_value='true'),
        DeclareLaunchArgument(
            'global_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mcu_can_bridge'), 'config', 'global_localization.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'rtk_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mobileess_rtk'), 'config', 'um982.example.yaml'
            ]),
        ),
        # Ant1 is the right-side primary antenna, measured relative to base_link.
        DeclareLaunchArgument('rtk_x_m', default_value='-0.17'),
        DeclareLaunchArgument('rtk_y_m', default_value='-0.325'),
        DeclareLaunchArgument('rtk_z_m', default_value='0.70'),
        DeclareLaunchArgument('rtk_roll_rad', default_value='0.0'),
        DeclareLaunchArgument('rtk_pitch_rad', default_value='0.0'),
        DeclareLaunchArgument('rtk_yaw_rad', default_value='0.0'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox_frame',
            # Measured Mid-360 optical/IMU frame relative to vehicle center.
            arguments=['0.17', '0.0', '0.48', '0.0', '0.0', '0.0',
                       'base_link', 'livox_frame'],
            output='screen',
        ),
        Node(
            package='mcu_can_bridge',
            executable='mcu_can_bridge',
            name='mcu_can_bridge',
            output='screen',
            parameters=[LaunchConfiguration('wheel_params_file'),
                        {'interface': LaunchConfiguration('can_interface')}],
            condition=IfCondition(LaunchConfiguration('enable_can_feedback')),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_rtk_link',
            arguments=[LaunchConfiguration('rtk_x_m'), LaunchConfiguration('rtk_y_m'),
                       LaunchConfiguration('rtk_z_m'), LaunchConfiguration('rtk_roll_rad'),
                       LaunchConfiguration('rtk_pitch_rad'), LaunchConfiguration('rtk_yaw_rad'),
                       'base_link', 'rtk_link'],
            condition=global_condition,
            output='screen',
        ),
        Node(
            package='mobileess_rtk',
            executable='rtk_node',
            name='um982_rtk',
            parameters=[rtk_params_file],
            condition=rtk_node_condition,
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[global_params_file],
            remappings=[('gps/fix', '/rtk/fix'), ('imu/data', '/rtk/heading'),
                        ('odometry/filtered', '/odometry/filtered')],
            condition=global_condition,
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_filter_node',
            namespace='global',
            parameters=[global_params_file],
            condition=global_condition,
            output='screen',
        ),
    ])
