#!/usr/bin/env python3
"""Launch Gazebo, ideal kinematic robot, camera/LiDAR bridges, RViz and teleop."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    panel = LaunchConfiguration('panel')
    x = LaunchConfiguration('spawn_x')
    y = LaunchConfiguration('spawn_y')
    yaw = LaunchConfiguration('spawn_yaw')

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    description_dir = get_package_share_directory('robot_description')
    sim_share = get_package_share_directory('sim_control')
    with open(description_dir + '/urdf/dase7503_sim.urdf', encoding='utf-8') as f:
        robot_description = f.read()
    bridge_config = sim_share + '/config/bridge.yaml'
    rviz_config = sim_share + '/rviz/simulation.rviz'
    model = sim_share + '/models/ideal_robot/model.sdf'
    # Resolve CAD meshes against the installed ROS package before spawning.
    with open(model, encoding='utf-8') as f:
        model_xml = f.read().replace(
            'package://robot_description/', 'file://' + description_dir + '/')
    default_world = sim_share + '/worlds/dase7503_field.sdf'

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world SDF'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Show RViz'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('panel', default_value='true', description='Show /cmd_vel panel'),
        DeclareLaunchArgument('spawn_x', default_value='1.0', description='Initial X in metres'),
        DeclareLaunchArgument('spawn_y', default_value='0.3', description='Initial Y in metres'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.57079632679',
                               description='Initial yaw; pi/2 points along +Y'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=IfCondition(gui),
            launch_arguments={'gz_args': ['-r -v 3 ', world]}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=UnlessCondition(gui),
            launch_arguments={'gz_args': ['-r -s -v 3 ', world]}.items()),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='simulation_bridge', output='screen',
             parameters=[{'config_file': bridge_config}]),

        # Let the world server advertise create/set_pose before spawning.
        TimerAction(period=2.0, actions=[Node(
            package='ros_gz_sim', executable='create', name='spawn_ideal_robot',
            output='screen', arguments=['-string', model_xml, '-name', 'ideal_robot',
                                        '-world', 'dase7503_field',
                                        '-x', x, '-y', y, '-z', '0', '-Y', yaw])]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}]),

        Node(package='sim_control', executable='ideal_base', name='ideal_base',
             output='screen', parameters=[{
                 'use_sim_time': True,
                 'world_name': 'dase7503_field',
                 'entity_name': 'ideal_robot',
                 'start_x': ParameterValue(x, value_type=float),
                 'start_y': ParameterValue(y, value_type=float),
                 'start_yaw': ParameterValue(yaw, value_type=float),
             }]),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[{'use_sim_time': True}],
             condition=IfCondition(rviz)),

        Node(package='sim_control', executable='control_panel', name='control_panel',
             output='screen', condition=IfCondition(panel)),
    ])
