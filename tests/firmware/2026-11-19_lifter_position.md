# 2026-11-19 升降改为180°位置控制

用户纠正：S1上的MG996R实际是180°位置舵机，1500µs对应90°；原连续旋转假设错误，旧版上升结束后恢复1500µs会让机构返回90°。本记录替代旧连续旋转升降实现的当前操作说明，不改写历史实测结果。

## 修订行为

- menuconfig提供最低/最高角度（默认0°/80°），0°/180°脉宽端点（默认500/2500µs，可反向标定）。90°默认1500µs，80°四舍五入为1389µs。
- `/lifter` UInt8 1/0直接选最高/最低目标；删除arm/jog服务。首次合法指令前不输出位置PWM，也不假定实际在最低点。
- UP_MS/DOWN_MS保留用户既有数值，改为稳定等待；角度由目标脉宽决定，等待结束保持目标，不能回1500µs。并未实现可配置运动速度。
- 相同目标幂等且不延长计时，稳定等待期间反向拒绝；首次0也有效。
- 保留轮静止200ms前置条件、升降时底盘互锁、额外稳定余量、通信/轮故障/IO故障锁存。
- stop撤销PWM，不能在没有实测角度时承诺保持中间位置；下一条合法0/1可恢复人工stop，硬故障须排查RESET。
- 诊断改为目标/上下角度、端点脉宽及等待状态；`lifter_position_confirmed=false`。
- 2订阅+2定时器+1服务，executor 5句柄；保留原micro-ROS资源池上限和2048字节MTU，无需缩减缓存。

## 主机验证（通过）

```bash
cc -std=c11 -Wall -Wextra -Werror \
  -I firmware/chassis/components/lifter \
  tests/firmware/lifter_control_test.c \
  firmware/chassis/components/lifter/lifter_control.c \
  -o /tmp/hku-lifter-state-check
/tmp/hku-lifter-state-check
python3 tests/firmware/check_lifter_driver.py
python3 tests/firmware/check_lifter_ros.py
python3 tests/firmware/check_motor_control.py
python3 tests/firmware/check_imu_publication.py
```

直接运行实际状态机、驱动和ROS回调：覆盖免arm首次上/下、重复与反向命令、无启动PWM、上下等待完成后长期保持目标、非默认角度/反向端点、人工撤销/直接恢复、故障锁存与JSON/MTU容量。轮控制和IMU发布回归通过。主机PWM替身不验证真实波形、角度、载荷和供电。

## 编译与实机边界

完整ESP-IDF 5.1.2构建通过，应用375984字节，SHA-256 `d0c9a1c0373a361aaf653c936536ef217e3871f1fadd9dd757e25e3c1778ce7d`。详细配置见[构建记录](2026-11-19_lifter_position_build.json)。本次保留用户当前上下稳定等待各1200ms（menuconfig缺省600ms），上下角度为0°/80°。Kconfig新增项通过 `bash firmware/chassis/idf.sh reconfigure build` 刷新配置后编译，不能继续用旧生成头文件。

本轮没有烧录、没有向实物发送升降或底盘运动命令。500/2500µs是Yahboom本地官方 `02. ESP32 basic course/06. Drive PWM servo.pdf` 示例的端点范围，尚未对这颗舵机标定。用户确认1500µs=90°不能单独证明这两个端点准确；需脱载验证机构净空、方向和0°/80°位置，再做带载保持、断线/撤销PWM以及20循环验收。
