#!/usr/bin/env python3
"""只启动 Gazebo 并加载 DASE7503 场地世界（不生成机器人）。

用途：肉眼核对场地布局 / 量尺寸 / 给 PPT 截图。

用法：
  # 目标环境：Gazebo Harmonic（Ubuntu 24.04 / ROS 2 Jazzy）
  ros2 launch sim/launch/field_world.launch.py

  # 本机若是 Ubuntu 22.04 + Gazebo Fortress（Humble）：
  ros2 launch sim/launch/field_world.launch.py backend:=fortress

  # 无界面
  ros2 launch sim/launch/field_world.launch.py headless:=true

  # 不装 ROS 也行（直接命令行）：
  gz sim -r sim/worlds/dase7503_field.sdf             # Harmonic
  ign gazebo sim/worlds/dase7503_field_fortress.sdf   # Fortress
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.abspath(os.path.join(HERE, "..", "worlds"))

# 两个变体：几何一样，只有 SDF 版本与插件命名不同
BACKENDS = {
    # Harmonic / gz-sim8 —— 命令是 `gz sim`
    "harmonic": {"exe": "gz", "sub": ["sim"], "world": "dase7503_field.sdf"},
    # Fortress / ign-gazebo6 —— 命令是 `ign gazebo`
    "fortress": {"exe": "ign", "sub": ["gazebo"], "world": "dase7503_field_fortress.sdf"},
}


def _start_gz(context, *args, **kwargs):
    backend = context.launch_configurations.get("backend", "harmonic")
    headless = context.launch_configurations.get("headless", "false").lower() == "true"

    if backend not in BACKENDS:
        raise RuntimeError(f"backend 只能是 {list(BACKENDS)}，收到 '{backend}'")
    cfg = BACKENDS[backend]

    cmd = [cfg["exe"], *cfg["sub"], "-r", "-v", "3"]
    if headless:
        cmd.append("-s")          # 只跑服务器，不开图形界面
    cmd.append(os.path.join(WORLDS, cfg["world"]))

    print(f"[field_world] backend={backend}  cmd={' '.join(cmd)}")
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="harmonic",
                              description="harmonic (Jazzy/24.04) 或 fortress (Humble/22.04)"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true = 不开 Gazebo 图形界面"),
        OpaqueFunction(function=_start_gz),
    ])
