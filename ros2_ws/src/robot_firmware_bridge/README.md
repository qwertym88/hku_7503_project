> **2026-11-11当前任务v2.1**：M2交付agent安装、确切串口/波特率、断连看门狗证据；M5集成建图/定位执行两模式。原始/odom不改名，EKF输出/odometry/filtered与唯一局部TF，不relay、不映射回/odom。详见[接口](../../../docs/给队友看/模块接口说明.md)与[运行说明](../../../docs/给队友看/运行说明.md)。

# robot_firmware_bridge —— micro-ROS 桥接（M2 + M5）

## 职责
在 Pi5 上运行 `micro_ros_agent`，把 ESP32 的 micro-ROS 话题接到 ROS 2 网络：

```text
ESP32 ──USB 串口── micro_ros_agent ── /cmd_vel /odom /imu/data ── ROS 2
```

## 待办
- [ ] 固定串口设备名（udev 规则），避免 `/dev/ttyUSB*` 漂移
- [ ] 断线重连 + 心跳检测
- [ ] 启动脚本与 bringup 集成

当前仍无 ROS 包/启动脚本实现；本机已用 Docker agent 连通正常固件，`/lifter` 尚未在固件实现。串口 UART0、921600、domain42，同串口只运行一个 agent；操作和容器停止/重建步骤见[固件说明](../../../firmware/chassis/README.md)。USB 重连可能需重建容器与板上 RESET，尚未实现自动恢复。Pi 上部署/udev/统一启动仍待完成，见[M2交付清单](../../../docs/给队友看/M2底盘进度与交付清单.md)。
