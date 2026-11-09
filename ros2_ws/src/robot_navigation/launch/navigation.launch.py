#!/usr/bin/env python3
"""Nav2 点到点导航（M4-5/M4-6）：雷达 + EKF + AMCL + Nav2 全栈。

用法：
  # M2 固件未就绪（现在）：mock 底盘，验证"发目标 → 出 /cmd_vel"全链路
  ros2 launch robot_navigation navigation.launch.py mock_base:=true rviz:=true

  # 真机：
  ros2 launch robot_navigation navigation.launch.py map:=maps/dase7503_field.yaml

发目标点（RViz 里用 2D Goal Pose，或命令行）：
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.20, y: 1.33}, \
    orientation: {z: 0.7071, w: 0.7071}}}}"

AMCL 初始位姿默认 = Start 区中心（navigation.yaml 里 set_initial_pose），
比赛时车摆到 Start 区即可，无需手动初始化。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

PKG = 'robot_navigation'
# 仓库根目录的 maps/（本包用 --symlink-install 安装，launch 文件即源码文件，可向上定位）
HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
DEFAULT_MAP = os.path.join(REPO_ROOT, 'maps', 'dase7503_field.yaml')


def generate_launch_description():
    mock_base = LaunchConfiguration('mock_base')
    rviz = LaunchConfiguration('rviz')
    map_yaml = LaunchConfiguration('map')

    ekf_config = os.path.join(
        get_package_share_directory(PKG), 'config', 'ekf.yaml')
    nav_config = os.path.join(
        get_package_share_directory(PKG), 'config', 'navigation.yaml')
    rviz_config = os.path.join(
        get_package_share_directory(PKG), 'rviz', 'nav.rviz')
    lidar_launch = os.path.join(
        get_package_share_directory(PKG), 'launch', 'lidar.launch.py')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=DEFAULT_MAP,
            description='地图 yaml 路径（建图产物，见 slam.launch.py 的保存命令）'),

        DeclareLaunchArgument(
            'mock_base', default_value='false',
            description='M2 固件未就绪时置 true：mock 节点发零速 odom/unfiltered（真机务必置 false）'),

        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='启动 RViz'),

        # 雷达 + robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)),

        # mock 底盘（T0，决策 D9）
        Node(
            package=PKG,
            executable='mock_odom',
            name='mock_odom',
            output='screen',
            condition=IfCondition(mock_base),
        ),

        # EKF：odom/unfiltered + imu/data → /odom + odom→base_footprint TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odom')],
        ),

        # Nav2 全栈：map_server + AMCL + costmaps + planner + controller + 行为树
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': 'false',
                'params_file': nav_config,
                'autostart': 'true',
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
            parameters=[{'use_sim_time': False}],
        ),
    ])
