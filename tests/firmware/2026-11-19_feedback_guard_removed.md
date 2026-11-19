# 2026-11-19 移除无反馈和反向反馈停车锁存

用户反馈A/B：`4943022`构建烧录后可前后/左右行驶，`15aed21`不行，随后明确要求移除“300ms无反馈四轮停车并锁存”和反向保护。本次按该要求移除，不保留默认启用/隐藏开关；刚才未提交的1秒启动宽限也一并撤销。

## 源码与接口

- 删除motor_feedback_guard.h、相关独立测试和Motor_Fault_Latched接口；电机任务不再根据编码器位移/方向锁住四轮，速度入口不再因这些故障拒绝新命令。
- 编码器仍用于PID和原始odom，前进原始counts增量为正；PWM引脚/轮序、增益、速度限幅不变。
- 保留480ms速度命令超时、显式零速停车、通信故障处理、升降时轮电机互锁与停止/截止行为。超时后新的有效速度命令可恢复，不需清除轮反馈锁存。
- 移除诊断feedback_missing_mask/feedback_reversed_mask；新增feedback_guard_enabled=false和lift_inhibited，明确前两项已不存在。台架工具兼容新字段结构，遇到历史固件非零mask仍停止。
- 升降不再订阅/传播已删除的轮编码器故障；仍要求轮控制关闭且反馈速度连续200ms接近0，并保留自身通信、IO、等待截止故障。
- IMU校准文件指纹保持不变，当前补偿开启；本轮未主动修改menuconfig；当前sdkconfig/构建中的位置舵机为0°/80°、上下等待各600ms，与早先校准记录中的0°/75°、1200ms不同，烧录前按实际机构核对。

## 软件验证

```bash
python3 tests/firmware/check_motor_control.py
python3 tests/firmware/check_motor_startup.py
python3 tests/firmware/check_lifter_driver.py
python3 tests/firmware/check_lifter_ros.py
python3 tests/firmware/test_chassis_bench_tool.py
python3 tests/firmware/check_imu_publication.py
bash firmware/chassis/idf.sh build
```

实际motor.c/car_motion.c注入持续缺失反馈、反向反馈并持续刷新命令，确认不再锁存；前后/左右转、PID反馈符号、累计计数回绕、互锁和命令超时/恢复通过。实际升降状态机和驱动/ROS、7项台架工具、IMU发布/补偿回归通过。

新增启动测试使用仓库实际增量PID源码与确定性的模拟起转阈值，与4943022对比：两者均在模拟610ms起转；持续无反馈+有效命令仍保持控制使能；停止刷新命令后均在480ms停用。610ms是模拟场景，不是实测当前电机启动时间。先前300ms保护在同场景会提前停止，但不能仅凭该模拟断言现场全部根因。

完整ESP-IDF5.1.2构建通过，应用375424字节，SHA256 `5065335f216c650798dcfd01ea8fcd4d566ee681681dc7971d74f261b377d3cb`，见[构建摘要](2026-11-19_feedback_guard_removed_build.json)。

## 上板验证

本次未烧录、未发送实物动作命令。停止全部动作发布者、串口agent后正常flash并恢复唯一agent，确认diagnostics中feedback_guard_enabled=false。先架空短时前/后/左右转，再测停止发布时的物理停车。源码保留480ms超时不等于物理停轮已验收。

当前持续速度命令下，丢失/反向编码器反馈不会自动停车；PID仍会对误差做调整。异常动作按操作说明停止发布、发零或切断执行器电源。本次移除不声称已经解决所有供电/机械或任务级反馈监控问题。
