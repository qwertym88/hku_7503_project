#!/usr/bin/env python3
"""M3 感知节点：相机取流 → QR 解码 → /qr（M3-1/2/3 一键启动）。

用法：
  # 树莓派真机（picamera2 取流，固定对焦 2.5 屈光度 ≈ 0.4 m）
  ros2 launch qr_perception qr_perception.launch.py
  ros2 launch qr_perception qr_perception.launch.py focus_diopters:=2.5

  # 本机无相机：用图片目录回放（整条链路含 ROS 发布都能测）
  ros2 launch qr_perception qr_perception.launch.py source:=file \
      file_path:=/home/you/7139/tests/qr_dataset/RACKA_4X6M

验证（M3-3 验收标准）：
  ros2 topic echo /qr            # 举码到相机前，应看到完整字符串，且同一码只发一次
  ros2 topic hz /camera/image_raw  # 内部图像流（默认 640x480）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('qr_perception'), 'config', 'qr_perception.yaml')

    source = LaunchConfiguration('source')
    file_path = LaunchConfiguration('file_path')
    focus_diopters = LaunchConfiguration('focus_diopters')
    exposure_time_us = LaunchConfiguration('exposure_time_us')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    confirm_frames = LaunchConfiguration('confirm_frames')

    return LaunchDescription([
        DeclareLaunchArgument(
            'source', default_value='picamera2',
            description='帧源: picamera2 | rpicam | file | cv（Ubuntu 24.04 无 python3-picamera2 时用 rpicam）'),
        DeclareLaunchArgument(
            'file_path', default_value='',
            description='source:=file 时的图片目录/视频/单图路径'),
        DeclareLaunchArgument(
            'focus_diopters', default_value='0.0',
            description='固定对焦屈光度(1/米)，0=自动对焦。例: 0.4m 停靠距离 → 2.5'),
        DeclareLaunchArgument(
            'exposure_time_us', default_value='0',
            description='锁曝光时间(µs)，0=自动曝光。防运动模糊建议 ≤10000'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument(
            'confirm_frames', default_value='2',
            description='多帧投票：连续 N 帧一致才上报 /qr'),

        Node(
            package='qr_perception',
            executable='qr_perception',
            name='qr_perception',
            output='screen',
            parameters=[config, {
                'source': source,
                'file_path': file_path,
                'focus_diopters': focus_diopters,
                'exposure_time_us': exposure_time_us,
                'width': width,
                'height': height,
                'confirm_frames': confirm_frames,
            }],
        ),
    ])
