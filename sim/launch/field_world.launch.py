#!/usr/bin/env python3
"""只启动 Gazebo 并加载 DASE7503 场地世界（不生成机器人）。

用途：肉眼核对场地布局 / 量尺寸 / 给 PPT 截图。

用法：
  ros2 launch sim/launch/field_world.launch.py
  ros2 launch sim/launch/field_world.launch.py headless:=true   # 无界面
  # 不装 ROS 也行：
  gz sim -r sim/worlds/dase7503_field.sdf
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD = os.path.abspath(os.path.join(HERE, "..", "worlds", "dase7503_field.sdf"))


def _start_gz(context, *args, **kwargs):
    headless = context.launch_configurations.get("headless", "false").lower() == "true"
    cmd = ["gz", "sim", "-r", "-v", "3"]
    if headless:
        cmd.append("-s")          # 只跑服务器，不开图形界面
    cmd.append(WORLD)
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="true = 不开 Gazebo 图形界面"),
        OpaqueFunction(function=_start_gz),
    ])
