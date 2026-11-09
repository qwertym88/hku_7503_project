#!/usr/bin/env python3
"""MS200 雷达 → /scan（M4-1）+ robot_state_publisher（发布 base_footprint→laser 等 TF）。

用法：
  ros2 launch robot_navigation lidar.launch.py
  ros2 launch robot_navigation lidar.launch.py port:=/dev/ttyACM0

验证（M4-1 验收标准）：
  ros2 topic hz /scan          # 期望 ~10 Hz（电机 10 转/秒，每转一帧完整扫描）
  ros2 topic echo /scan --once # 检查 frame_id == "laser"、单位 rad/m
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os

URDF_PATH = os.path.join(
    get_package_share_directory('robot_description'), 'urdf', 'dase7503_nav.urdf')


def generate_launch_description():
    port = LaunchConfiguration('port')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyACM0',
            description='MS200 串口设备（CH343 芯片，插上一般是 ttyACM0）'),

        DeclareLaunchArgument(
            'frame_id', default_value='laser',
            description='/scan 的 frame_id（接口契约固定为 laser）'),

        # MS200 驱动（Oradar 官方 ROS 2 驱动，230400 波特率，47 字节定长包协议）
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='ms200',
            output='screen',
            parameters=[{
                'device_model': 'MS200',
                'frame_id': frame_id,
                'scan_topic': '/scan',
                'port_name': port,
                'baudrate': 230400,
                'angle_min': 0.0,
                'angle_max': 360.0,
                'range_min': 0.05,
                'range_max': 20.0,
                'clockwise': False,
                'motor_speed': 10,
            }],
        ),

        # TF：base_footprint → base_link → laser / imu_link / cam_link
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', URDF_PATH]), value_type=str),
                'use_sim_time': False,
            }],
        ),
    ])
