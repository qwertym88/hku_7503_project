"""One checked YAML drives world poses, spawn, cargo attachment and the mission."""

from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mission_control.field_map import rasterize
from mission_control.offline_plan import plan_route
from mission_control.route import load_config, validate_geometry
from mission_control.validate import write_svg
from sim_control.field_rviz import write_field_rviz
from sim_control.mission_world import render_world


def start(context):
    config = str(Path(LaunchConfiguration('config').perform(context)).expanduser().resolve())
    cfg = load_config(config)
    if not cfg['simulation_only']:
        raise ValueError('Simulation requires a simulation_only route')
    share = get_package_share_directory('sim_control')
    directory = Path(tempfile.mkdtemp(prefix='dase7503-mission-'))
    if IfCondition(LaunchConfiguration('plan_route')).evaluate(context):
        cfg = plan_route(cfg, float(LaunchConfiguration('clearance').perform(context)))
        planned = directory / 'planned_route.yaml'
        planned.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding='utf-8')
        config = str(planned)
    actions = validate_geometry(cfg)  # Reject invalid/obstructed routes before any motion.
    scene = cfg['scene']
    map_override = LaunchConfiguration('map').perform(context)
    map_file = directory / 'field_map.yaml'
    rasterize(scene).save(map_file)
    if map_override:
        map_file = Path(map_override).expanduser().resolve()
        metadata = yaml.safe_load(map_file.read_text(encoding='utf-8'))
        if not isinstance(metadata, dict) or not isinstance(metadata.get('image'), str) or not metadata['image']:
            raise ValueError(f'Occupancy map must reference an image: {map_file}')
        if not (map_file.parent / metadata['image']).is_file():
            raise FileNotFoundError(f'Map image missing: {map_file.parent / metadata["image"]}')
    spawn = cfg['start_pose']
    params = yaml.safe_load((Path(share) / 'config/sim_localization.yaml').read_text(encoding='utf-8'))
    params['map_server']['ros__parameters']['yaml_filename'] = str(map_file)
    params['amcl']['ros__parameters']['initial_pose'] = {**spawn, 'z': 0.0}
    localization_params = directory / 'localization.yaml'
    localization_params.write_text(yaml.safe_dump(params), encoding='utf-8')
    rviz_config = write_field_rviz(Path(share)/'rviz/simulation.rviz', directory/'field.rviz',
                                   scene['field'], mission=True)
    world = render_world(Path(share)/'worlds/dase7503_field.sdf', directory/'world.sdf', cfg)
    write_svg(cfg, actions, directory/'route.svg')
    output = LaunchConfiguration('output_dir').perform(context)
    return [
        LogInfo(msg=f'Checked {len(actions)} actions; generated world and route review: {directory}'),
        LogInfo(msg=f'Localizing on {"supplied" if map_override else "generated geometry"} map: {map_file}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share+'/launch/sim_ideal.launch.py'),
            launch_arguments=dict(world=world, gui=LaunchConfiguration('gui'),
                rviz=LaunchConfiguration('rviz'), rviz_config=rviz_config, panel='false',
                spawn_x=str(spawn['x']),spawn_y=str(spawn['y']),spawn_yaw=str(spawn['yaw'])).items()),
        Node(package='sim_control', executable='cargo_simulator', output='screen',
             parameters=[{'config':config,'use_sim_time':True}]),
        GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('nav2_bringup') + '/launch/localization_launch.py'),
            launch_arguments={'map': str(map_file), 'params_file': str(localization_params),
                              'use_sim_time': 'true', 'autostart': 'true',
                              'use_composition': 'False'}.items())]),
        Node(package='mission_control', executable='fixed_route', output='screen',
             parameters=[{'config':config,'use_sim_time':True,'output_dir':output,
                          'require_localization':True,
                          'autostart':LaunchConfiguration('autostart')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=get_package_share_directory('mission_control')+'/config/fixed_route_sim.yaml'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/fixed_route_runs'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('plan_route', default_value='false', description='Plan in XY and heading before launching Gazebo'),
        DeclareLaunchArgument('clearance', default_value='0.10', description='Planner minimum clearance in metres'),
        DeclareLaunchArgument('map', default_value='',
                              description='Saved occupancy YAML in field coordinates; empty generates the map from scene geometry'),
        DeclareLaunchArgument('ros_domain_id', default_value='77', description='Isolated simulation ROS domain'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        OpaqueFunction(function=start),
    ])
