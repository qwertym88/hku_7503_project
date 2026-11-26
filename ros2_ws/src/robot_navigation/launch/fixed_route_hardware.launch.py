"""Start the fixed mission on the standard field, using the real chassis feedback."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from mission_control.offline_plan import plan_route
from mission_control.route import load_config, validate_config, validate_geometry
from mission_control.validate import write_svg


def prepare_route(base_path, overrides_path, *, preplanned=False):
    cfg = load_config(base_path)
    overrides = yaml.safe_load(Path(overrides_path).read_text(encoding='utf-8'))
    cfg['control'].update(overrides.pop('control', {}))
    for key, fields in overrides.pop('racks', {}).items():
        cfg['racks'][key].update(fields)
    cfg.update(overrides)
    if cfg['simulation_only'] or not cfg.get('qr_faces_negative_y'):
        raise ValueError('Hardware requires a real route with a default QR approach side')
    if preplanned:
        clearance = cfg.get('planning', {}).get('clearance_m', 0.)
        if clearance < 0.10:
            raise ValueError('Encoder mode requires a route preplanned at 0.10 m clearance or more')
        cfg['version'] += '-encoder'
        return validate_config(cfg)
    return plan_route(validate_config(cfg), clearance=0.10)


def start(context):
    share = Path(get_package_share_directory('robot_navigation'))
    encoder = LaunchConfiguration('localization_mode').perform(context) == 'encoder'
    route_argument = 'encoder_route' if encoder else 'layout'
    directory = Path(tempfile.mkdtemp(prefix='dase7503-hardware-'))
    cfg = prepare_route(LaunchConfiguration(route_argument).perform(context),
                        LaunchConfiguration('hardware_config').perform(context), preplanned=encoder)
    route_file = directory / 'route.yaml'
    route_file.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding='utf-8')
    write_svg(cfg, validate_geometry(cfg), directory / 'route.svg')
    rviz_file = share / 'rviz' / 'fixed_route.rviz'

    output_dir = LaunchConfiguration('output_dir').perform(context)
    if not output_dir:
        suffix = '_encoder' if encoder else ''
        output_dir = f'runtime_logs/fixed_route_hardware{suffix}'
    mission_params = {
        'config': str(route_file), 'output_dir': output_dir, 'use_sim_time': False,
        'feedback_topic': '/odom' if encoder else '/odometry/filtered',
        'require_localization': not encoder,
        'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
    }
    if encoder:
        localization = [Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro "', LaunchConfiguration('robot_urdf'), '"']), value_type=str)}],
        )]
        localization_info = 'Wheel odometry (/odom)'
    else:
        localization, localization_info = amcl_nodes(share, directory, cfg, mission_params)

    qr_share = Path(get_package_share_directory('qr_perception'))
    return [
        LogInfo(msg=f'{localization_info} | Offline route: {route_file} | Preview: {directory / "route.svg"} | RViz: {rviz_file}'),
        *localization,
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(qr_share / 'launch' / 'qr_perception.launch.py')),
            launch_arguments={'source': LaunchConfiguration('camera_source'),
                              'width': LaunchConfiguration('camera_width'),
                              'height': LaunchConfiguration('camera_height'),
                              'camera_framerate': LaunchConfiguration('camera_framerate'),
                              'focus_diopters': LaunchConfiguration('focus_diopters')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        )]),
        Node(package='mission_control', executable='fixed_route_encoder' if encoder else 'fixed_route_hardware',
             output='screen', parameters=[mission_params]),
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', str(rviz_file)],
             parameters=[{'use_sim_time': False}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def amcl_nodes(share, directory, cfg, mission_params):
    map_file = share / 'maps' / 'field_geometry.yaml'
    params = yaml.safe_load((share / 'config' / 'fixed_route_amcl.yaml').read_text(encoding='utf-8'))
    params['map_server'] = {'ros__parameters': {'yaml_filename': str(map_file)}}
    seed = cfg['start_pose']
    params['amcl']['ros__parameters']['initial_pose'] = {
        'x': seed['x'], 'y': seed['y'], 'z': 0.0, 'yaw': seed['yaw'],
    }
    amcl_file = directory / 'amcl.yaml'
    amcl_file.write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf-8')
    mission_params['amcl_transform_tolerance_s'] = params['amcl']['ros__parameters']['transform_tolerance']

    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    return [
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'lidar.launch.py')),
            launch_arguments={'port': LaunchConfiguration('lidar_port'), 'urdf': LaunchConfiguration('robot_urdf')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_lidar')),
        )]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[str(share / 'config' / 'fixed_route_ekf.yaml')]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'localization_launch.py')),
            launch_arguments={'map': str(map_file), 'params_file': str(amcl_file), 'use_sim_time': 'false',
                              'autostart': 'true', 'use_composition': 'False'}.items(),
        )]),
    ], f'Standard field: {map_file}'


def generate_launch_description():
    share = Path(get_package_share_directory('robot_navigation'))
    mission = Path(get_package_share_directory('mission_control'))
    description = Path(get_package_share_directory('robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value=str(mission / 'config' / 'fixed_route_sim.yaml')),
        DeclareLaunchArgument('localization_mode', default_value='amcl', choices=['amcl', 'encoder']),
        DeclareLaunchArgument('encoder_route', default_value=str(share / 'routes' / 'encoder_a_offset.yaml')),
        DeclareLaunchArgument('hardware_config', default_value=str(share / 'config' / 'fixed_route_hardware.yaml')),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('robot_urdf', default_value=str(description / 'urdf' / 'dase7503_nav.urdf')),
        DeclareLaunchArgument('camera_source', default_value='rpicam'),
        DeclareLaunchArgument('camera_width', default_value='512'),
        DeclareLaunchArgument('camera_height', default_value='384'),
        DeclareLaunchArgument('camera_framerate', default_value='15'),
        DeclareLaunchArgument('focus_diopters', default_value='0.0'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        OpaqueFunction(function=start),
    ])
