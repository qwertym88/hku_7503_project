# ESP32 micro-ROS 固件（M2）

实现 3 件事：

| 方向 | 接口 | 内容 |
|---|---|---|
| 订阅 | `/cmd_vel` (`Twist`) | 麦轮逆运动学 → 4 轮目标转速 → 编码器 PID |
| 发布 | `/odom` (`Odometry`) | 编码器推算位置 + yaw |
| 订阅 | `/lifter` (`UInt8`) | `1` 升起 / `0` 放下，带限位与堵转保护 |

## ⚠️ 先读：别自己推逆运动学

麦轮逆/正运动学**已有现成实现**，直接移植：

1. **首选**：Yahboom 官方例程 `twist_subscriber` + `odom_publisher`（ESP-IDF，同板同电机，源码在官方资料包/虚拟机 `~/esp/Samples/microros_samples/`）。
2. **备选**：`linorobot2_hardware` 的 `lib/kinematics/`（Arduino，`MECANUM` 枚举现成，含 PID/编码器/odom）。

完整对比与代码引用见 **《[底盘参考实现](../docs/给队友看/底盘参考实现.md)》**。

**两条硬规矩**：运动学**只放 ESP32 一侧**；**保底阶段先不开横向平移**（`linear.y = 0`）。

## 必须做到
- **速度看门狗**：断流 0.5s 内停车。
- **上电双人复核**：电机线白接头→控制板、黑接头→电机，接反会烧板。
- 麦轮左右旋配对正确，否则平移方向相反。

## 引脚 / 轮序 / 参数
- PPR 13，减速比 1:20（来自需求材料）。
- 引脚表、轮序、串口波特率：**待 M2 填写后替换本段。**

## 构建与烧录
- 框架：**ESP-IDF**（官方例程同款）或 Arduino/PlatformIO（linorobot2 路线）——**二选一，别混用**。
- 官方参考：https://github.com/YahboomTechnology/MicroROS-Board
- agent 用 `micro-ROS/micro-ROS-Agent` 的 **`jazzy` 分支**，走**串口**（满足需求文档）。
- 烧录步骤：**待 M2 补充。**

> 当前目录为空壳，未烧录任何程序。
