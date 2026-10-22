# robot_firmware_bridge —— micro-ROS 桥接（M2 + M5）

## 职责
在 Pi5 上运行 `micro_ros_agent`，把 ESP32 的 micro-ROS 话题接到 ROS 2 网络：

```text
ESP32 ──USB 串口── micro_ros_agent ── /cmd_vel /odom /lifter ── ROS 2
```

## 待办
- [ ] 固定串口设备名（udev 规则），避免 `/dev/ttyUSB*` 漂移
- [ ] 断线重连 + 心跳检测
- [ ] 启动脚本与 bringup 集成

当前为空壳。固件本体在仓库根目录 `firmware/`。
