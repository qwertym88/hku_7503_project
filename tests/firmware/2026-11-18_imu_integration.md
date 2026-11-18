# 板载IMU接入与软件验证

2026-11-18，Asia/Shanghai。用户要求复用官方组件，为现有底盘固件加入板载IMU与/imu/data。

## 实现

- 从`~/Desktop/Samples.zip`中的`esp32_samples/read_imu/components`复用i2c_master/icm42670p/inv_imu。inv_imu的16个文件保持字节一致，原始版权文字保留；来源逐文件SHA256见`firmware/chassis/components/icm42670p/official_sources.json`。
- 使用官方GPIO39/40、I2C0、0x68与WHO_AM_I=0x67、自检、4g/2000dps量程、400Hz ODR及FIFO。独立core0任务5ms轮询；短I2C超时与无长重试，读取不占用ROS回调和core1电机任务。
- 临界区快照包含六轴SI数据、FIFO接收时间、序号、有效状态与初始化/读取错误诊断。FIFO只接收同时有效的accel/gyro，不接受INT16_MIN无效帧。
- `/imu/data`为sensor_msgs/Imu、imu_link、best effort、目标50Hz，单位m/s²及rad/s。官方_g/_dps getter已经是SI，未二次转换；加速度含重力。
- 使用micro-ROS同步时钟回推FIFO接收时刻。启动未就绪、初始化失败、读取失败、重复或超过100ms样本不发布；未虚构姿态（orientation_covariance[0]=-1），其余协方差未知为0。
- 原始odom仍来自编码器，未融合IMU。安装外参、传感器轴方向及偏置待硬件测量；IMU故障仅使IMU数据停止发布，当前不额外改变底盘指令处理，任务级必要传感器失效停车由后续集成验证。
- 扩展micro-ROS池到3个publisher/4个topic，executor到3个handle。修正共享缓存容量检查与vendor头文件拷贝路径，核对库安装头、应用头、构建头均为3/4/1配置。

## 验证

1. `bash firmware/chassis/idf.sh build`成功，应用链接并通过分区大小检查；已有micro-ROS依赖的include_next/未使用变量告警，不是本次IMU编译错误。
2. 应用大小356432字节，SHA256 `035bb902db397cdcfb27ee69410e4820d3e117eb0574a841ad629905f6d0b65d`；机器可读记录见[2026-11-18_imu_build.json](2026-11-18_imu_build.json)。
3. `gcc -std=c11 -Wall -Wextra -Werror -Ifirmware/chassis/components/icm42670p tests/firmware/imu_sample_test.c -lm -o /tmp/imu_sample_test`，运行通过：1g尺度、1000dps到rad/s、符号、FIFO无效值、过期/重复/未来/读取失败样本拒绝、恢复。
4. `python3 tests/firmware/check_imu_publication.py`通过：提取实际main.c的IMU回调，以主机桩注入样本/时钟/发布返回值，验证SI载荷、接收时间戳、各无效状态不发布、恢复及发布失败设置fault。
5. 组件链接清单、micro-ROS三个头文件池容量、脚本bash语法及git diff --check通过。

未烧录、未操作设备或发送任何运动命令；控制板仍为之前不含IMU的正常底盘固件。50Hz为配置目标，不能写成板上实测频率。WHOAMI、自检、静态重力/零偏、旋转符号、数据频率/最大间隔/时间戳及与odom并发的表现仍待烧录后验证。

用户报告M1–M3电机及编码器测试正常，M4编码器损坏待换；该外部报告不替代完整四轮交付验收。IMU静态与手动转动检查无需驱动底盘。
