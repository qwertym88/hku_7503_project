> **已作废的历史假设**：用户随后查明轮ID接错，纠正后四轮向前原始计数均增加。下文的四轮-1处理已移除，不作为当前操作说明。当前状态见[按4943022恢复方向记录](2026-11-19_encoder_baseline_restore.md)。

# 2026-11-19 四轮编码器方向适配

用户确认四个编码器都有反馈，新轮安装使“向车头滚动时原始计数减小”。这不等于编码器损坏；正常控制需把硬件方向转换为车体前进为正。

## 修改

- `firmware/chassis/components/motor/motor.h`新增M1～M4各自的 `MOTOR_ENCODER_SIGN` 常量，当前全为-1，编译期检查只能为±1。
- 电机任务先对原始累计count求每周期差（保留计数回绕处理），再乘方向系数；PID误差、无反馈/反向反馈保护、Motor_Get_Speed及Motion_Get_Speed统一使用转换后的增量。
- `/chassis/diagnostics.counts`仍报告原始PCNT计数，向前递减；target/speed则使用车体方向，前进为正。odom通过同一速度通路获得正确符号。
- PWM输出方向没有修改。先独立确认正驱动是否实际向前；若不符需另行修正驱动方向，不能靠反转目标速度掩盖。
- 300ms无反馈窗口、480ms看门狗、保护锁存与升降互锁保留。此次不能据此确认此前missing_mask=10的启动/供电根因已消失。

## 验证

编译实际motor.c和car_motion.c，注入向前raw-negative、后退raw-positive、正反转向，检查实际PID误差/输出纠偏方向、速度与Vx/Wz符号，计数回绕、原始诊断计数、M4无反馈/M2反向锁存、看门狗及升降互锁均通过。升降驱动/ROS回调、IMU发布和独立保护逻辑回归通过。

```bash
python3 tests/firmware/check_motor_control.py
python3 tests/firmware/check_lifter_driver.py
python3 tests/firmware/check_lifter_ros.py
python3 tests/firmware/check_imu_publication.py
bash firmware/chassis/idf.sh build
```

ESP-IDF5.1.2构建通过，应用376160字节，SHA256 `c57937ac231764eca0c60188106a036fd0d2e685cd0bafd0070581b160fc7f28`，详见[构建摘要](2026-11-19_encoder_direction_build.json)。保留已启用的IMU零偏补偿和用户当前升降0°/75°、各1200ms参数。本轮没有烧录、没有向实机发送轮电机或舵机动作命令，软件通过不等于实际驱动方向或供电验收。

## 烧录后架空复核

停止所有速度/升降发布者及串口agent，按正常方式flash，恢复唯一agent。先手转，确认前进时raw counts减小、对应speed为正；再每次一个短时前/后/左右转动作，观察实际轮向和故障位。若正命令使轮子后退，立即停止并核对PWM方向；若仍missing_mask非0，记录首次启动到锁存的过程、供电与实际轮子动作，不能仅修改编码器方向就认定修复。
