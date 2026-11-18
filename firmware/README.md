> **当前状态（2026-11-18）**：正常底盘固件已烧录并包含板载 IMU，静态 odom 约20Hz、IMU约50Hz。用户确认 M1～M3 电机/编码器正常、M4 编码器待换。零偏补偿、完整安装外参、地面标定、升降和安全/供电验收尚未完成。详见[M2进度与交付清单](../docs/给队友看/M2底盘进度与交付清单.md)与[烧录及静态证据](../tests/firmware/2026-11-18_imu_flash_verification.md)。

# ESP32 micro-ROS 固件（M2）

目标接口与当前实现：

| 方向 | 接口 | 内容 / 状态 |
|---|---|---|
| 订阅 | `/cmd_vel` (`Twist`) | 已实现：左右轮组模型 → 4 轮目标转速 → 编码器 PID，禁止横移 |
| 发布 | `/odom` (`Odometry`) | 已实现：编码器推算位置 + yaw，不发 TF；地面精度待标定 |
| 订阅 | `/lifter` (`UInt8`) | **待实现**：`1` 升起 / `0` 放下，行程/异常保护需与 M1 确认 |
| 发布 | `/imu/data` (`Imu`) | 已实现六轴发布：板载 **ICM42670P**；零偏未补偿，无姿态估计，M4 经校准验证后使用 |

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
- 减速比 **1:20**，编码器 **13 线 AB 相**。
- **`MOTOR_ENCODER_CIRCLE = 1040`** 为当前配置（13 × 4倍频 × 20）；官方 PCNT 高低电平同时触发，当前电机每转计数仍需逐轮实测确认。
  详见 **《[里程计标定](../docs/给队友看/里程计标定.md)》**（含 65mm 轮速查、三步标定、检查清单）。
- 轮序：左前=Motor1、左后=Motor2、右前=Motor3、右后=Motor4。
- 首次底盘工程使用UART0 TX=GPIO43、RX=GPIO44、micro-ROS波特率921600、domain ID=42；完整引脚见`chassis/components/encoder/encoder.h`和`chassis/components/pwm_motor/pwm_motor.h`。

## 构建与烧录
- 框架：沿用官方 **ESP-IDF** 例程；本轮不切换 Arduino/PlatformIO。
- 官方参考：https://github.com/YahboomTechnology/MicroROS-Board
- agent 用 `micro-ROS/micro-ROS-Agent` 的 **`jazzy` 分支**，走**串口**（满足需求文档）。
- 首次底盘构建/烧录与agent命令见[chassis操作说明](chassis/README.md)。

> `chassis/`已有首次底盘源码，完整M2功能仍待补齐；实际烧录和运动结果必须由M2实测记录。
