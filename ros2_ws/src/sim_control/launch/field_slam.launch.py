"""Scan the simulation field with movable racks removed from the mapping run."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from mission_control.route import load_config
from sim_control.field_rviz import write_field_rviz
from sim_control.mission_world import render_world


def start(context):
    share = Path(get_package_share_directory('sim_control'))
    slam_share = Path(get_package_share_directory('slam_toolbox'))
    cfg = load_config(LaunchConfiguration('config').perform(context))
    directory = Path(tempfile.mkdtemp(prefix='dase7503-field-slam-'))
    world = render_world(share / 'worlds/dase7503_field.sdf', directory / 'world.sdf', cfg,
                         include_racks=False)
    rviz = write_field_rviz(share / 'rviz/simulation.rviz', directory / 'field.rviz',
                           cfg['scene']['field'], mission=False)
    spawn = cfg['start_pose']
    return [
        LogInfo(msg=f'Mapping static walls and columns from Gazebo LiDAR: {directory}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/sim_ideal.launch.py')),
            launch_arguments={'world': world, 'rviz_config': rviz, 'gui': LaunchConfiguration('gui'),
                              'rviz': LaunchConfiguration('rviz'), 'panel': LaunchConfiguration('panel'),
                              'spawn_x': str(spawn['x']), 'spawn_y': str(spawn['y']),
                              'spawn_yaw': str(spawn['yaw'])}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam_share / 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': str(share / 'config/field_slam.yaml')}.items()),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('mission_control')) / 'config/fixed_route_sim.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('panel', default_value='true'),
        DeclareLaunchArgument('ros_domain_id', default_value='77'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        OpaqueFunction(function=start),
    ])
