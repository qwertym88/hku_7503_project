"""One checked YAML drives world poses, spawn, cargo attachment and the mission."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mission_control.route import load_config, validate_geometry
from mission_control.validate import write_svg
from sim_control.mission_world import render_world


def start(context):
    config = str(Path(LaunchConfiguration('config').perform(context)).expanduser().resolve())
    cfg = load_config(config)
    actions = validate_geometry(cfg)  # Reject invalid/obstructed routes before any motion.
    share = get_package_share_directory('sim_control')
    directory = Path(tempfile.mkdtemp(prefix='dase7503-mission-'))
    world = render_world(Path(share)/'worlds/dase7503_field.sdf', directory/'world.sdf', cfg)
    write_svg(cfg, actions, directory/'route.svg')
    output = LaunchConfiguration('output_dir').perform(context)
    spawn = cfg['start_pose']
    return [
        LogInfo(msg=f'Checked {len(actions)} actions; generated world and route review: {directory}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share+'/launch/sim_ideal.launch.py'),
            launch_arguments=dict(world=world, gui=LaunchConfiguration('gui'),
                rviz=LaunchConfiguration('rviz'), panel='false',
                spawn_x=str(spawn['x']),spawn_y=str(spawn['y']),spawn_yaw=str(spawn['yaw'])).items()),
        Node(package='sim_control', executable='cargo_simulator', output='screen',
             parameters=[{'config':config,'use_sim_time':True}]),
        Node(package='mission_control', executable='fixed_route', output='screen',
             parameters=[{'config':config,'use_sim_time':True,'output_dir':output,
                          'autostart':LaunchConfiguration('autostart')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=get_package_share_directory('mission_control')+'/config/fixed_route_sim.yaml'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/fixed_route_runs'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('ros_domain_id', default_value='77', description='Isolated simulation ROS domain'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        OpaqueFunction(function=start),
    ])
