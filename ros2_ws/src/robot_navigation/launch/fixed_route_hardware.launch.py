"""Start the fixed mission on the standard field, using the real chassis feedback."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from mission_control.offline_plan import plan_route
from mission_control.route import load_config, validate_config, validate_geometry
from mission_control.validate import write_svg


def prepare_route(base_path, overrides_path):
    cfg = load_config(base_path)
    overrides = yaml.safe_load(Path(overrides_path).read_text(encoding='utf-8'))
    cfg['control'].update(overrides.pop('control', {}))
    for key, fields in overrides.pop('racks', {}).items():
        cfg['racks'][key].update(fields)
    cfg.update(overrides)
    if cfg['simulation_only'] or not cfg.get('qr_faces_negative_y'):
        raise ValueError('Hardware requires a real route with −Y-facing QR codes')
    return plan_route(validate_config(cfg), clearance=0.10)


def write_rviz(template, target, field):
    cfg = yaml.safe_load(template.read_text(encoding='utf-8'))
    manager = cfg['Visualization Manager']
    manager['Tools'] = [tool for tool in manager['Tools'] if tool['Class'] != 'rviz_default_plugins/SetGoal']
    manager['Tools'].append({'Class': 'rviz_default_plugins/SetInitialPose', 'Topic': {'Value': '/initialpose'}})
    manager['Views']['Current'].update(Scale=145., X=(field['x_min'] + field['x_max'])/2,
                                       Y=(field['y_min'] + field['y_max'])/2)
    for display in manager['Displays']:
        if display['Name'] == 'AMCLParticles':
            display.update(Class='rviz_default_plugins/PoseWithCovariance', Name='AMCL pose', Shape='Arrow')
            display['Topic'].update(Value='/amcl_pose', **{'Durability Policy': 'Transient Local'})
            display.pop('Color', None)
        if display['Name'] not in ('GlobalPath', 'LocalPath'):
            continue
        active = display['Name'] == 'LocalPath'
        display.update(Name='Current route segment' if active else 'Planned mission route',
                       Color='0; 218; 185' if active else '94; 91; 183',
                       **{'Line Style': 'Billboards', 'Line Width': 0.038 if active else 0.020})
        display['Topic'].update(Value='/mission/active_path' if active else '/mission/planned_path',
                                **{'Durability Policy': 'Transient Local'})
    target.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding='utf-8')


def start(context):
    share = Path(get_package_share_directory('robot_navigation'))
    directory = Path(tempfile.mkdtemp(prefix='dase7503-hardware-'))
    cfg = prepare_route(LaunchConfiguration('layout').perform(context),
                        LaunchConfiguration('hardware_config').perform(context))
    route_file = directory / 'route.yaml'
    route_file.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding='utf-8')
    write_svg(cfg, validate_geometry(cfg), directory / 'route.svg')

    map_file = share / 'maps' / 'field_geometry.yaml'
    params = yaml.safe_load((share / 'config' / 'fixed_route_amcl.yaml').read_text(encoding='utf-8'))
    params['map_server'] = {'ros__parameters': {'yaml_filename': str(map_file)}}
    seed = cfg['start_pose']
    params['amcl']['ros__parameters']['initial_pose'] = {
        'x': seed['x'], 'y': seed['y'], 'z': 0.0, 'yaw': seed['yaw'],
    }
    amcl_file = directory / 'amcl.yaml'
    amcl_file.write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf-8')
    rviz_file = directory / 'fixed_route.rviz'
    write_rviz(share / 'rviz' / 'nav.rviz', rviz_file, cfg['scene']['field'])

    qr_share = Path(get_package_share_directory('qr_perception'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    return [
        LogInfo(msg=f'Standard field: {map_file} | Offline route: {route_file} | Preview: {directory / "route.svg"} | RViz: {rviz_file}'),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'lidar.launch.py')),
            launch_arguments={'port': LaunchConfiguration('lidar_port'), 'urdf': LaunchConfiguration('robot_urdf')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_lidar')),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(qr_share / 'launch' / 'qr_perception.launch.py')),
            launch_arguments={'source': LaunchConfiguration('camera_source'),
                              'focus_diopters': LaunchConfiguration('focus_diopters')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        )]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[str(share / 'config' / 'ekf.yaml'), {'use_sim_time': False, 'odom0': '/odom'}]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'localization_launch.py')),
            launch_arguments={'map': str(map_file), 'params_file': str(amcl_file), 'use_sim_time': 'false',
                              'autostart': 'true', 'use_composition': 'False'}.items(),
        )]),
        Node(package='mission_control', executable='fixed_route_hardware', output='screen', parameters=[{
            'config': str(route_file), 'output_dir': LaunchConfiguration('output_dir'), 'use_sim_time': False,
            'feedback_topic': '/odometry/filtered', 'require_localization': True, 'autostart': False,
        }]),
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', str(rviz_file)],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('robot_navigation'))
    mission = Path(get_package_share_directory('mission_control'))
    description = Path(get_package_share_directory('robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value=str(mission / 'config' / 'fixed_route_sim.yaml')),
        DeclareLaunchArgument('hardware_config', default_value=str(share / 'config' / 'fixed_route_hardware.yaml')),
        DeclareLaunchArgument('output_dir', default_value='runtime_logs/fixed_route_hardware'),
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('robot_urdf', default_value=str(description / 'urdf' / 'dase7503_nav.urdf')),
        DeclareLaunchArgument('camera_source', default_value='rpicam'),
        DeclareLaunchArgument('focus_diopters', default_value='0.0'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        OpaqueFunction(function=start),
    ])
