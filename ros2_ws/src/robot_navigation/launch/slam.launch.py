#!/usr/bin/env python3
"""SLAM 建图（M4-2/M4-4）：雷达 + EKF + slam_toolbox。

用法：
  # M2 固件还没就绪时（现在）：用 mock 底盘发零速 odom，链路全通，可以手提雷达走动建图
  ros2 launch robot_navigation slam.launch.py mock_base:=true rviz:=true

  # M2 固件就绪后（真机）：固件发 /odom（remap 到 odom/unfiltered）+ /imu/data
  ros2 launch robot_navigation slam.launch.py rviz:=true

建完图保存（在仓库根目录执行）：
  ros2 run nav2_map_server map_saver_cli -f maps/dase7503_field

话题链路：
  MS200 → /scan → slam_toolbox → /map（+ map → odom TF）
  固件/mock → odom/unfiltered + /imu/data → EKF → /odom + odom → base_footprint TF
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


def generate_launch_description():
    mock_base = LaunchConfiguration('mock_base')
    rviz = LaunchConfiguration('rviz')

    ekf_config = os.path.join(
        get_package_share_directory(PKG), 'config', 'ekf.yaml')
    slam_config = os.path.join(
        get_package_share_directory(PKG), 'config', 'slam.yaml')
    rviz_config = os.path.join(
        get_package_share_directory(PKG), 'rviz', 'nav.rviz')
    lidar_launch = os.path.join(
        get_package_share_directory(PKG), 'launch', 'lidar.launch.py')

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mock_base', default_value='false',
            description='M2 固件未就绪时置 true：启动 mock 节点发零速 odom/unfiltered，'
                        '让 EKF/SLAM 全链路可跑（真机务必置 false）'),

        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='启动 RViz'),

        # 雷达 + robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)),

        # mock 底盘（T0，决策 D9）：发 odom/unfiltered 零速里程计
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

        # slam_toolbox 在线异步建图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_config,
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
