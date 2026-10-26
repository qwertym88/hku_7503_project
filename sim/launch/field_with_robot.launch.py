#!/usr/bin/env python3
"""DASE7503 场地 + linorobot2 麦轮机器人 —— 一条命令进入可导航的仿真。

它做的事：把【我们的场地世界】塞给 linorobot2 现成的 gazebo.launch.py。
linorobot2 那边会自动完成：生成机器人、起 EKF、桥接全部话题、加载 Nav2 所需 TF。

前置：
  1. 装了 Gazebo Harmonic 与 ros_gz：
       sudo apt install ros-jazzy-ros-gz
  2. linorobot2 已放进工作区并编译：
       git clone -b jazzy https://github.com/linorobot/linorobot2.git ros2_ws/src/linorobot2
       cd ros2_ws && colcon build && source install/setup.bash

用法：
  ros2 launch sim/launch/field_with_robot.launch.py
  ros2 launch sim/launch/field_with_robot.launch.py gui:=false     # 无界面（跑回归时用）

出生点默认在【Start 区】：中心 (2.72, 1.00)，朝向 -X（yaw = π）。
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD = os.path.abspath(os.path.join(HERE, "..", "worlds", "dase7503_field.sdf"))

# Start 区：x 1.21~1.81, y 0.00~0.80 → 中心 (1.51, 0.40)
SPAWN_X = "1.51"
SPAWN_Y = "0.40"
SPAWN_Z = "0.0"        # 机器人陷地/悬空时改这里（每次 ±0.02 试）
SPAWN_YAW = "1.5708"   # π/2 → 车头朝 +Y（从底部朝场地内部）


def generate_launch_description():
    base = LaunchConfiguration("base")
    gui = LaunchConfiguration("gui")
    odom_topic = LaunchConfiguration("odom_topic")

    linorobot2_gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "launch", "gazebo.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("base", default_value="mecanum",
                              description="linorobot2 的底盘型号（用它的 mecanum 模板）"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="是否启动 Gazebo 图形界面"),
        DeclareLaunchArgument("odom_topic", default_value="/odom",
                              description="EKF 输出的里程计话题（与我们的契约一致）"),

        # linorobot2 的 launch 用环境变量挑 URDF 文件
        SetEnvironmentVariable("LINOROBOT2_BASE", base),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(linorobot2_gazebo_launch),
            launch_arguments={
                "world_path": WORLD,          # ← 关键：覆盖它的默认 world
                "spawn_x": SPAWN_X,
                "spawn_y": SPAWN_Y,
                "spawn_z": SPAWN_Z,
                "spawn_yaw": SPAWN_YAW,
                "gui": gui,
                "odom_topic": odom_topic,
            }.items(),
        ),
    ])
