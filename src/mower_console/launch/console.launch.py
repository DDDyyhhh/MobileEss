"""Start observation and the browser; CAN motion is owned by one task only."""
from pathlib import Path
import copy
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def start(context):
    share=Path(get_package_share_directory('mower_console'))
    nav_share=Path(get_package_share_directory('mower_nav2'))
    config=Path(context.launch_configurations['config'])
    observation_only=context.launch_configurations['observation_only']
    if observation_only not in ('true','false'):
        raise ValueError('observation_only must be true or false')
    cfg=yaml.safe_load(config.read_text())
    map_dir=Path(cfg['map_directory'])
    for name in ('map.yaml','map.pgm','map.pcd','livox.json','mid360_real.yaml'):
        if not (map_dir/name).is_file():
            raise ValueError('Missing fixed map/calibration asset: '+str(map_dir/name))
    params=yaml.safe_load((share/'config/nav2_console.yaml').read_text())
    for name in ('ForwardGrid','ReverseGrid'):
        params['planner_server']['ros__parameters'][name]['lattice_filepath']=str(share/'config/lattice_r4_10cm.json')
    # Humble NodeLattice has a process-global motion table cached by filename.
    # Different allow_reverse settings must not share a planner process.
    reverse_params=copy.deepcopy({key:params[key] for key in ('planner_server','global_costmap')})
    reverse_params['planner_server']['ros__parameters']['planner_plugins']=['ReverseGrid']
    reverse_params['planner_server']['ros__parameters'].pop('ForwardGrid')
    reverse_params['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic']='/map'
    params['planner_server']['ros__parameters']['planner_plugins']=['ForwardGrid']
    params['planner_server']['ros__parameters'].pop('ReverseGrid')
    with tempfile.NamedTemporaryFile(mode='w',prefix='mower-console-reverse-',suffix='.yaml',delete=False) as f:
        yaml.safe_dump({'reverse_planning':reverse_params},f,sort_keys=False)
        reverse_config=f.name
    with tempfile.NamedTemporaryFile(mode='w',prefix='mower-console-nav2-',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(params,f,sort_keys=False)
        nav_config=f.name
    def include(file,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share/'launch'/file)),launch_arguments=args.items())
    return [
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',str(share/'config/dds_console_udp.xml')),
        include('mid360_field.launch.py',{'stage':'sensors','livox_config':str(map_dir/'livox.json'),'fast_lio_config':str(map_dir/'mid360_real.yaml')}),
        include('mower_navigation.launch.py',{
            'map':str(map_dir/'map.yaml'),'map_pcd':str(map_dir/'map.pcd'),
            'start_fast_lio':'false','enable_motion':'false','enable_can_feedback':'true',
            'safety_map_approved':'false','collision_robot_radius':'0.70','console_safety_profile':'true',
            'nav2_params_file':nav_config,'localizer_params_file':str(share/'config/localizer_console.yaml'),
            'livox_obstacle_params_file':str(share/'config/livox_console.yaml')}),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',
             namespace='reverse_planning',parameters=[reverse_config],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager',
             namespace='reverse_planning',parameters=[{'autostart':True,'node_names':['planner_server'],
                                                       'use_sim_time':False}],output='screen'),
        ExecuteProcess(cmd=[str(Path(get_package_prefix('mower_console'))/'lib/mower_console/console_server'),
                            '--config',str(config)]+
                       (['--observe-only'] if observation_only=='true' else []),output='screen'),
    ]


def generate_launch_description():
    share=Path(get_package_share_directory('mower_console'))
    return LaunchDescription([DeclareLaunchArgument('config',default_value=str(share/'config/console.yaml')),
                              DeclareLaunchArgument('observation_only',default_value='true'),
                              OpaqueFunction(function=start)])
