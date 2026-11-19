# 2026-11-19 按最近提交恢复电机反馈方向

用户查明更换轮布局后没有按固件轮ID连接；纠正接线后四轮向前滚动时 `/chassis/diagnostics.counts` 均增加。因此之前依据误接线观察引入的四轮负向转换不适用。

参考最近提交 `4943022`（feat(m2): 集成底盘固件与板载IMU，归档实测并更新交付进度）：原始编码器增量直接进入PID，前进计数增量/速度为正。现已移除encoder_sign数组、MOTOR_ENCODER_SIGN_M1～M4及乘-1处理；保留后续无符号差分对计数回绕的处理。

## 与提交基线的关系

- 编码器encoder.c/h、电机PWM驱动pwm_motor.c/h、运动学car_motion.c/h六个文件与4943022逐字节相同，不重新猜测AB相或PWM方向。
- motor.c恢复原始count增量方向，保留后续增加的“先查四轮再写PWM”、无反馈/反向故障锁存、480ms看门狗与升降互锁。未把整个motor文件退回旧版以免丢失这些接口。
- 已启用的IMU零偏补偿头文件指纹保持不变；位置舵机当前0°/75°及上下1200ms配置保持不变，不恢复旧连续旋转模式。
- counts仍为原始计数，target为每周期目标计数，speed为m/s；现在前进均为正向。转向符号见[手动测试表](../../docs/给队友看/M2手动测试步骤.md)。

## 软件验证

```bash
python3 tests/firmware/check_motor_control.py
python3 tests/firmware/check_lifter_driver.py
python3 tests/firmware/check_lifter_ros.py
python3 tests/firmware/check_imu_publication.py
bash firmware/chassis/idf.sh build
```

实际motor.c/car_motion.c注入验证前进原始正增量、后退负增量、左右转向、PID纠偏、诊断和速度/odom符号，以及回绕、无反馈/反向锁存、看门狗和互锁，通过。升降驱动/ROS回调、IMU补偿发布及独立保护回归通过。主机替身不验证真实电机功率与实物运动。

ESP-IDF5.1.2完整构建通过：应用376096字节，SHA256 `64b70b007054dbcff6128afefba0b003d82d90c2d4c848c71a665313147a7087`。详见[构建摘要](2026-11-19_encoder_baseline_restore_build.json)。

## 上板状态

本次没有烧录或发送动作命令。停止全部动作发布者和串口agent后，用正常chassis工程flash；复位会清除故障锁存，但不得带着非零发布者复位。恢复唯一agent，架空先手转确认count增量与speed同向，再逐次验证前/后/左/右实际运动。此前missing_mask=10是历史失败，不能仅凭纠正接线和编译通过便宣称供电、闭环或物理停车已验收。
