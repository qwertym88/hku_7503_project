# ESP32 micro-ROS 固件（M2）

实现 3 件事：

| 方向 | 接口 | 内容 |
|---|---|---|
| 订阅 | `/cmd_vel` (`Twist`) | 麦轮逆运动学 → 4 轮目标转速 → 编码器 PID |
| 发布 | `/odom` (`Odometry`) | 编码器推算位置 + yaw |
| 订阅 | `/lifter` (`UInt8`) | `1` 升起 / `0` 放下，带限位与堵转保护 |
| 发布 | `/imu/data` (`Imu`) | 板载 **ICM42670P**，EKF 只需 `angular_velocity.z` |

## ⚠️ 先读：别自己推逆运动学

运动学**有现成实现**，别自己推。但先分清两件事：

1. **首选**：Yahboom 官方例程 `twist_subscriber` + `odom_publisher`（ESP-IDF，同板同电机，源码在官方资料包/虚拟机 `~/esp/Samples/microros_samples/`）。
   ⚠️ **官方参考车是"普通轮四轮差速"，不是麦轮**：`Motion_Ctrl(vx, 0, wz)` 把 `linear.y` 写死为 0，模型是左右两组差速。**照搬它 = 麦轮当四驱用（无横移），正好是保底路线。**
2. **要麦轮横移才用**：`linorobot2_hardware` 的 `lib/kinematics/`（Arduino，`MECANUM` 枚举现成，含 PID/编码器/odom）。

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
