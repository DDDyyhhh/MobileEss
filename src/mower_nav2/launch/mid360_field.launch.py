"""One driver, explicit field stage, observation-only by default."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def start(context):
    args = {key: context.launch_configurations[key] for key in (
        'stage', 'livox_config', 'fast_lio_config', 'session_dir', 'map_dir',
        'nav2_params', 'localizer_params', 'cmd_vel_params', 'can_interface', 'safety_map_approved', 'physical_ready')}
    stage = args['stage']
    if stage not in ('sensors', 'capture', 'observe', 'motion'):
        raise ValueError('stage must be sensors, capture, observe, or motion')
    for key in ('livox_config', 'fast_lio_config'):
        if not Path(args[key]).is_file():
            raise ValueError(f'{key} must name a real configuration file')
    config = json.loads(Path(args['livox_config']).read_text())
    if not config.get('lidar_configs'):
        raise ValueError('Livox configuration has no lidar_configs')
    nodes = [Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                  name='livox_lidar_publisher', output='screen', parameters=[{
                      'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                      'publish_freq': 10.0, 'output_data_type': 0,
                      'frame_id': 'livox_frame', 'user_config_path': args['livox_config'],
                      'cmdline_input_bd_code': 'livox0000000001',
                  }])]

    def include(package, file, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory(package)) / 'launch' / file)),
            launch_arguments=arguments.items())

    if stage == 'sensors':
        nodes.append(include('fast_lio', 'fast_lio_real.launch.py', {
            'config_file': args['fast_lio_config'], 'publish_tf': 'false'}))
    elif stage == 'capture':
        session = Path(args['session_dir'])
        if not session.is_dir() or (session / 'map.pcd').exists():
            raise ValueError('capture requires a new prepared session directory')
        nodes.append(include('fast_lio', 'map_capture.launch.py', {
            'config_file': args['fast_lio_config'],
            'map_file_path': str(session / 'map.pcd')}))
    else:
        map_dir = Path(args['map_dir'])
        for file in ('map.yaml', 'map.pcd'):
            if not (map_dir / file).is_file():
                raise ValueError(f'missing matching map asset: {map_dir / file}')
        moving = stage == 'motion'
        if moving and (args['safety_map_approved'] != 'true' or
                       args['physical_ready'] != 'true'):
            raise ValueError('motion requires reviewed maps and physical readiness')
        nodes.append(include('mower_nav2', 'mower_navigation.launch.py', {
            'map': str(map_dir / 'map.yaml'), 'map_pcd': str(map_dir / 'map.pcd'),
            'nav2_params_file': args['nav2_params'],
            'localizer_params_file': args['localizer_params'],
            'fast_lio_config_file': args['fast_lio_config'],
            'can_interface': args['can_interface'],
            'enable_can_feedback': str(moving).lower(),
            'collision_robot_radius': '0.70',
            'enable_motion': str(moving).lower(),
            'safety_map_approved': args['safety_map_approved'],
            'cmd_vel_params_file': args['cmd_vel_params'],
        }))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('mower_nav2'))
    defaults = {
        'stage': 'sensors', 'livox_config': '/home/cat/MobileEss_config/livox.json',
        'fast_lio_config': '/home/cat/MobileEss_config/mid360_real.yaml',
        'session_dir': '', 'map_dir': '', 'can_interface': 'can0',
        'nav2_params': str(share / 'config' / 'nav2_field.yaml'),
        'localizer_params': str(share / 'config' / 'localizer_field.yaml'),
        'cmd_vel_params': str(share / 'config' / 'cmd_vel_field.yaml'),
        'safety_map_approved': 'false', 'physical_ready': 'false',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        OpaqueFunction(function=start),
    ])
