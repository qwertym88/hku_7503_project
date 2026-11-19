# M2 180°位置舵机升降控制

2026-11-19：用户纠正硬件属性，S1/GPIO8上的MG996R实际是180°位置舵机，1500µs对应90°。手动可转360°不能证明是连续旋转型。旧版“1500µs停转、按时间控制角度”不适用；本版改为持续发送目标位置脉宽，无需arm。新版已完成主机检查，且在本次[IMU校准记录](../../tests/firmware/2026-11-19_imu_calibration.md)中确认板端运行（实际下0°/上75°、等待各1200ms，无升降命令）；编译历史见[位置控制验证记录](../../tests/firmware/2026-11-19_lifter_position.md)；实物端点、载荷和20循环验收仍待完成。

## 正常操作

启用正常chassis固件、启动唯一agent、source ROS并设置domain42，先停止所有底盘速度发布者并等车轮停稳。直接发送一次上升：

```bash
ros2 topic pub /lifter std_msgs/msg/UInt8 '{data: 1}' \
  --once --wait-matching-subscriptions 1 --max-wait-time-secs 5
```

等实际停稳后下降：

```bash
ros2 topic pub /lifter std_msgs/msg/UInt8 '{data: 0}' \
  --once --wait-matching-subscriptions 1 --max-wait-time-secs 5
```

默认0对应下降0°，1对应抬升80°。无需调用 `/lifter/arm`；旧arm/jog服务已移除。不要同时启动多个 `/lifter` 发布者。`0`是下降，不是停止。

- 首次命令前不输出舵机PWM、不假定实际在最低点；首次0或1均会执行。
- 指令生效后直接输出目标角度脉宽，等待稳定时间结束后保持同一脉宽；不会自动返回90°，等待时间不决定角度或速度。
- 同目标重复命令不重启计时；稳定等待期间反向目标返回busy，应等停稳后发送。
- 轮电机关闭、反馈速度连续200ms接近0后才能开始升降；升降等待期间底盘非零速度拒绝。结束后保留约200ms互锁余量，旧速度不会自动恢复。
- `lifter_position`仅表示目标等待结束，`lifter_position_confirmed`恒为false；舵机内部的位置闭环没有向控制板提供测得角度/到位信号。

## menuconfig配置

在项目根目录运行：

```bash
bash firmware/chassis/idf.sh menuconfig
```

进入 `Chassis lifter (180-degree position servo)`：

| 参数 | 默认 | 含义 |
|---|---|---|
| CHASSIS_LIFTER_ENABLE | y | 启用升降；首次命令前仍不发PWM |
| CHASSIS_LIFTER_PORT | 1 | S1/GPIO8；2为S2/GPIO21 |
| CHASSIS_LIFTER_LOWER_DEG | 0 | `/lifter=0` 的目标角度 |
| CHASSIS_LIFTER_UPPER_DEG | 80 | `/lifter=1` 的目标角度，须大于最低角度 |
| CHASSIS_LIFTER_PULSE_0_DEG_US | 500 | 实物0°标定脉宽 |
| CHASSIS_LIFTER_PULSE_180_DEG_US | 2500 | 实物180°标定脉宽 |
| CHASSIS_LIFTER_UP_MS | 600 | 上升等待稳定时间，不是PWM持续截止时间 |
| CHASSIS_LIFTER_DOWN_MS | 600 | 下降等待稳定时间，不是角度/速度参数 |
| CHASSIS_LIFTER_TIMEOUT_MS | 5000 | 任务截止检查，必须大于两种等待时间 |

0～180°线性映射到两个端点脉宽，四舍五入到1µs；默认90°=1500µs、80°约1389µs、0°=500µs，50Hz持续输出。500/2500沿用Yahboom官方PWM示例映射，是待实物验证的端点，不能仅由“1500=90°”推断实际端点一定准确。先支撑机构、脱开连杆/负载校准端点，避免让0°机械干涉；实际方向反了可交换两个端点脉宽。180°可转范围不能替代机构净空检查。

原有UP_MS/DOWN_MS数值保留，但含义改为稳定等待；旧NEUTRAL_US/UP_US/DOWN_US/TRAVEL_CALIBRATED/JOG_MS配置已移除。改变角度或脉宽后重新编译烧录；不要只修改sdkconfig.defaults期待覆盖现有配置。

```bash
bash firmware/chassis/idf.sh build
```

停止所有速度/升降发布者和串口agent，再烧录：

```bash
docker stop hku-chassis-agent
bash firmware/chassis/idf.sh -p /dev/ttyUSB0 flash
```

如果agent已停止，跳过docker stop。烧录复位本身不发舵机位置命令。按[底盘说明](README.md)重新启动agent。

## 停止、故障与保持边界

如需撤销当前舵机输出：

```bash
ros2 service call /lifter/stop std_srvs/srv/Trigger '{}'
```

stop把PWM信号强制拉低，不再发送某个位置目标；机构位置标为未知并锁住底盘。没有实际角度反馈，不能实现“读取当前位置并闭环保持”，也不能保证舵机收到撤销信号后立刻停止或保持载荷。支撑载荷后再测试此行为，物理急停另行实现。`success=True`仅表示软件撤销请求被处理。

人工stop后，下一个合法0/1命令在车轮停稳且通信正常时直接恢复，无需arm。通信失联/主循环心跳超过480ms、PWM错误或错过任务截止时撤销PWM并锁存，需要排查后RESET；人工stop不能覆盖原硬故障。保持目标期间也监控通信，断线撤销后是否回落必须实测。不要把强制信号低等同于切断舵机电源。

## 诊断与M5接入

```bash
ros2 topic echo /chassis/diagnostics --qos-reliability best_effort --full-length
```

| 字段 | 含义 |
|---|---|
| lifter_state | disabled/unconfigured/idle/raising/lowering/fault |
| lifter_fault | none/operator_stop/timeout/communication/pwm_error |
| lifter_target_deg | 当前目标角度；首次命令前或撤销后为-1 |
| lifter_pulse_us | 当前软件请求脉宽；0表示撤销输出，不是0° |
| lifter_lower_deg / lifter_upper_deg | 配置的最低/最高角度 |
| lifter_pulse_0_deg_us / lifter_pulse_180_deg_us | 脉宽标定端点 |
| lifter_position | -1未知；0/1目标等待完成，无实测到位保证 |
| lifter_position_confirmed | 恒为false |
| lifter_up_ms / lifter_down_ms / lifter_timeout_ms | 稳定等待与截止参数 |
| lifter_rx / lifter_rejected / lifter_completed | 命令/拒绝/等待完成计数 |
| lifter_last_result | accepted/duplicate/invalid_value/not_ready/busy/wheels_moving |

M5先停止底盘并等待停稳，直接单次发0/1，再根据实测稳定时间加余量等待。故障或取消时不推进取放流程；不构造虚假到位消息。正常应用含2订阅、3发布、1服务，executor共5句柄；micro-ROS预留服务池与MTU2048可继续复用。

## 实机验收

1. 脱载确认上电/RESET不自行移动、首次0/1有效，校准最低/最高角度与方向，等待结束保持目标，不回90°。
2. 重复同值不重新冲击、等待中反向拒绝；底盘/升降互锁、人工stop和直接恢复按预期。
3. 支撑载荷后测通信断线、撤销PWM、复位/断电行为，检查实际停车/保持与供电电流、温升。
4. 空载/真实载荷20循环逐次记录实际角度、稳定时间、离地高度、保持/净空与视频，使用[循环记录表](../../tests/firmware/lifter_cycle_template.csv)。未做实测不得标为实机通过。
