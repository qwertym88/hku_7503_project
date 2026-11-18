# 底盘固件：USB 串口、四轮 PID、原始里程计与板载 IMU

2026-11-18，根据用户重新测量更新：**左右轮组中心间距200mm、前后轮组中心间距170mm**，标称轮径65mm。前后间距由首次提供的165mm修正为170mm；旋转几何系数由0.1825m更新为0.185m。测量应以轮组中心为基准。保留官方不横移模型，`linear.y`必须为0。

这是首次底盘bring-up固件，不是完整M2交付：包含`/cmd_vel`订阅、四轮编码器PID、20Hz目标频率的原始`/odom`、50Hz目标频率的板载IMU `/imu/data`发布与电机任务超时停车，另有诊断ADC估算；**IMU已烧录并完成静态检查，零偏补偿与完整安装外参待验证；升降、物理急停输入及自动断线恢复尚未实现**。通信失败后保持停车，需要按板上RESET重新连接。实际方向、频率、精度、停车延时必须实测。

## 参数在哪里

| 参数 | 文件 | 当前初值 |
|---|---|---|
| 左右间距 | `components/car_motion/car_motion.h`，`ROBOT_WIDTH` | 0.200m |
| 前后间距 | 同上，`ROBOT_LENGTH` | 0.170m |
| 旋转几何系数 | 同上，`ROBOT_APB` | 自动计算(W+L)/2 = 0.185m |
| 轮周长 | `components/motor/motor.h`，`MOTOR_WHEEL_CIRCLE` | π×65 = 204.2035mm |
| 编码器每输出轴转计数 | 同上，`MOTOR_ENCODER_CIRCLE` | 1040，假设官方310电机、13线×20减速×4倍频，须转轮实测 |
| PID周期 | 同上，`MOTOR_PID_PERIOD` | 10ms |
| 速度命令有效期 | 同上，`MOTOR_COMMAND_TIMEOUT_US` | 480ms，由独立电机任务每10ms检查；<0.5s仍待实测 |
| 首次调试车速限值 | `main/main.c`，`command_callback` | ±0.10m/s，±0.40rad/s |
| 通信/domain ID | `main/Kconfig.projbuild`，可用menuconfig修改 | UART0，921600，ROS_DOMAIN_ID=42 |

修改几何常量后重新build、flash。这里的旋转系数沿用官方模型，空/带载和实际地面仍需转角标定；不是精度保证。四轮方向沿用官方板卡：M1左前、M2左后、M3右前、M4右后。编码器和电机反向取决于实际装配，首次架空验证。

## 环境与编译

本机ESP-IDF在`~/esp/esp-idf`，版本5.1.2。micro-ROS采用Jazzy旧版提交`89fa3fef4cf3a58cad313eef0fb79445b8db08b2`；最新版要求IDF>=5.2，不能直接替换依赖。外部依赖和Python包位于仓库忽略的`.ref/`。`idf.sh`自动应用`microros-idf51.patch`，防止交叉编译误用主机`/opt/ros/jazzy`库。首次获取的ROS源码提交记录于`micro_ros_sources.lock.json`。

从仓库根目录运行：

```bash
bash firmware/chassis/idf.sh build
```

`idf.sh`自动加载ESP-IDF、隔离ROS构建环境并设置项目内缓存；不必在当前shell先source ROS。如需改domain ID或波特率：

```bash
bash firmware/chassis/idf.sh menuconfig
```

选择`Project chassis`。`sdkconfig.defaults`只对首次生成配置生效，已有`sdkconfig`应通过menuconfig改。

新机器需要先按官方教程安装ESP-IDF5.1.2，再准备以下依赖。**下列是恢复环境步骤，不是烧录命令**：

```bash
mkdir -p .ref
git clone --branch jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git .ref/micro_ros_espidf_component
git -C .ref/micro_ros_espidf_component checkout 89fa3fef4cf3a58cad313eef0fb79445b8db08b2
source ~/esp/esp-idf/export.sh
python -m pip install --target .ref/microros-python -r firmware/chassis/requirements-build.txt
```

首次构建会进一步下载micro-ROS的ROS源码，需要网络。不要把ESP-IDF与ROS工作区的colcon构建目录混用。

## 烧录

1. 车轮架空，先核对电机线白接头接控制板、黑接头接电机。初次烧录断开举升舵机，避免把保留出厂举升控制当作本固件功能。
2. 使用**Type-C数据串口**连接电脑；给Pi供电的Type-C接口不能用于烧录。检查`ls /dev/serial/by-id/`，也可能显示`/dev/ttyUSB0`。
3. 关闭占用该串口的agent、monitor或配置脚本。把下面的串口替换为实测设备名：

```bash
bash firmware/chassis/idf.sh -p /dev/ttyUSB0 flash
```

`flash`使用工程生成的bootloader、partition table和应用地址，不需要手工猜地址，也不要用`erase-flash`。连接失败时按官方流程按住BOOT、按一下RESET、松开BOOT后再试。

固件会覆盖出厂应用。需要恢复出厂固件时，使用官方`microROS_Robot_V2.0.0.bin`完整镜像和相应烧录说明；工程的`build/chassis.bin`是应用镜像，不能当出厂合并镜像写入0x0。

本固件UART0专用于micro-ROS，关闭运行期普通日志；不要通过同一串口同时运行`idf.py monitor`和agent，也不使用出厂`config_robot.py`修改此固件。

## 架空验收

以下为待执行步骤；先更换当前损坏的 M4 编码器并确认四轮反馈正常。既有单轮检查不等于四轮闭环验收。

Pi/电脑先安装Jazzy的micro_ros_agent。在已经source ROS2 Jazzy的另一个终端运行（与编译终端分开）：

```bash
export ROS_DOMAIN_ID=42
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

启动agent后按RESET，等待节点出现。先只观察：

```bash
ros2 node list
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 topic info /cmd_vel -v
```

确认`/odom`是本固件唯一发布，frame为`odom`/`base_footprint`。本固件不发TF；首次架空测试不需要启动EKF或导航。后续按当前接口文档接EKF，不直接套用旧的将滤波结果重映射回`/odom`的launch。

车轮保持架空，唯一速度发布者用以下命令持续发送0.03m/s：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.03}, angular: {z: 0.0}}'
```

按Ctrl+C断流，检查四轮在0.5s内停下。再测后退（x=-0.03）、原地左转（x=0,z=0.2）和USB拔线；按零速停车可用：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

确认轮序、方向、断流停车后再落地；有误先断电检查，不能靠增大PID掩盖电机/编码器方向错误。1m、0.10m和90°标定由M2/M4联合按`docs/给队友看/分工与任务拆解.md`验收，不能仅用速度乘时间证明精度。

## 来源

- Yahboom官方`Samples.zip`，本机`~/Desktop/Samples.zip`：`microros_samples/odom_publisher/components/{car_motion,encoder,motor,pwm_motor}`。本项目更改几何、轮周长、电机超时、线程互斥与PID停机清零，使用新的串口ROS主程序。
- `components/espressif__bdc_motor`与`components/espressif__pid_ctrl`来自该官方示例依赖；保留各自Apache-2.0 LICENSE及版权。PID组件增加停机清零函数。
- micro-ROS ESP-IDF组件：https://github.com/micro-ROS/micro_ros_espidf_component 。UART0 GPIO43/44与Yahboom官方串口例程/板卡定义一致。

构建结果和实车结果分别记录；无连接设备时只能报告编译通过，不能报告烧录或运动通过。

2026-11-12本机编译通过：`chassis.bin`为321856字节，SHA256为`09a9748d4a544ef8a82389f55e9b2478d156b0f64f41b4fb90d28b8061280388`。当时未检测到串口开发板，烧录与架空/落地测试均未执行。

2026-11-12后续实测：已通过`/dev/ttyUSB0`烧录，全部写入段校验成功并复位；Jazzy agent连接成功。静止100条`/odom`平均19.998Hz，最大间隔51.1ms；未发送运动指令。详见`tests/firmware/2026-11-12_flash.md`。本机使用已有Docker镜像，无需另外安装agent：

```bash
docker run --rm --name hku-chassis-agent --network host --device /dev/ttyUSB0 \
  -e ROS_DOMAIN_ID=42 microros/micro-ros-agent:jazzy \
  serial --dev /dev/ttyUSB0 -b 921600 -v4
```

如果容器已经运行，不重复启动；`docker logs -f --tail 30 hku-chassis-agent`查看，`docker stop hku-chassis-agent`停止。退出日志窗口不等于停止后台容器；`--rm` 容器停止后被删除，重新连接用 `docker run` 重建。复连异常先 `docker ps -a` 排除其他 agent/串口占用，再重建 agent 并按板上 RESET；仅换名字会让多个进程争抢同一串口。Pi 固定设备名/一键启动尚待集成。

## 当前硬件状态与历史排查

2026-11-18：板上为含 IMU 的正常 `/chassis` 固件；用户修正电机线束后已能驱动，确认 M1～M3 电机/编码器正常，M4 编码器损坏待更换。**正常闭环尚无编码器失效自动停车；无反馈可能使 PID 持续增加输出。M4 更换前保持断开，不执行四轮闭环或地面精度验收。** 分项交付状态与测试顺序见[M2 清单](../../docs/给队友看/M2底盘进度与交付清单.md)。

此前的 M1 无动作、M3 单轮复测、临时测试版烧录，以及 165mm→170mm 几何修正均保留在[测试总日志](../../tests/TEST_LOG.md)。[独立单轮 PWM/编码器工程](../motor_test/README.md)用于绕过 PID 排查，需临时另行烧录；正常固件不订阅 `/chassis/motor_pwm`。最近正常应用烧录证据见[IMU 板端验证](../../tests/firmware/2026-11-18_imu_flash_verification.md)。

只读话题 `/chassis/diagnostics`（`std_msgs/msg/String`、JSON、best effort、约2Hz）提供命令接收/拒绝数、使能、命令年龄、控制循环数、四轮累计计数/目标计数/速度/PID 输出及电池 ADC 估算；它不证明 PWM 电平、驱动电源或实际停车已经测量。电池电压需万用表和负载测试复核。

```bash
ros2 topic echo /chassis/diagnostics --qos-reliability best_effort --full-length
```

## 板载IMU接入（已烧录，静态检查通过）

2026-11-18新增：从官方Samples.zip的`esp32_samples/read_imu/components`复用`i2c_master`、`icm42670p`、`inv_imu`，驱动初始化、WHO_AM_I与自检后读取FIFO。InvenSense原始驱动和版权声明保留；改动与来源摘要见[驱动说明](components/icm42670p/README.md)。

- ICM42670P：I2C0，SCL=GPIO39、SDA=GPIO40，400kHz，地址0x68，WHO_AM_I预期0x67（诊断十进制103）。
- 保留官方±4g/±2000°/s量程、400Hz传感器ODR；独立core0任务每5ms轮询FIFO，避免在ROS回调中执行I2C。电机任务仍在core1，优先级更高。
- 发布`/imu/data`，`sensor_msgs/msg/Imu`，best effort，目标50Hz，`frame_id=imu_link`。角速度rad/s，加速度m/s²（包含重力）；官方函数的_g/_dps名称不代表真实输出单位，不二次转换。
- 只发布六轴测量，不解算姿态：orientation.w=1作为占位，orientation_covariance[0]=-1表示姿态不可用。角速度/加速度协方差暂为全0，表示未知，待静态实测或消费者明确配置；不伪造校准精度。
- 时间戳使用micro-ROS同步时钟回推到FIFO样本被MCU接收时刻；这是读取时间，不是芯片硬件采样时间，正常还包含FIFO/调度延迟。
- 保留传感器原生轴方向、零偏。`base_link→imu_link`安装外参及静态偏置须实测，当前URDF中的identity不能当作已确认结果；未将IMU混入固件原始odom。
- 启动未就绪、WHOAMI/自检失败、I2C读取失败、样本过期100ms或重复序号时不发布假数据。IMU故障本身不改变当前底盘指令接受逻辑；必要IMU断流的任务级停车仍需M4/M5按接口实现。通信失败仍按现有固件停车并等待RESET。
- `/chassis/diagnostics`新增`imu_state`（0初始化/1已配置/-1初始化失败）、`imu_whoami`、`imu_valid`、`imu_age_ms`（无样本为-1）、累计`imu_errors`、`imu_published`；`imu_state=1`不等于一定有新鲜数据，要结合valid/age/计数判断。
- micro-ROS池容量扩大到4 topics、3 publishers、1 subscription；`idf.sh`检查共享缓存容量并同步头文件，避免motor_test构建留下较小容量。两个工程不要并发构建。

该版本已烧录并通过静态通信及六轴读取检查；陀螺零偏已测量但未补偿，安装外参仍需完整验证。再次烧录沿用本文命令；M4编码器已由用户确认损坏，更换前断开M4，IMU检查不发送运动命令，可手动转动整块板/车体。

烧录并启动agent后，在已source ROS、domain42的终端：

```bash
ros2 topic echo /chassis/diagnostics --qos-reliability best_effort --full-length
ros2 topic echo /imu/data --qos-reliability best_effort --once
ros2 topic hz /imu/data
```

需实测：静置时陀螺零偏、加速度模长约9.81m/s²、绕各轴旋转的符号与安装方向、发布频率/最大间隔/时间戳年龄；确认odom约20Hz与电机看门狗未受影响。拔掉IMU/注入总线故障须在台架条件进行，验证没有旧数据被重打时间戳。当前已完成烧录与静态读取检查，不计完整安装方向、零偏补偿或整车验收。

2026-11-18静态实测：IMU约49.992Hz、odom约19.989Hz，WHOAMI=0x67，imu_errors=0。陀螺均值XYZ为[-0.013751,-0.005066,+0.024554]rad/s，尚未扣除；用户确认X前/Y左，Z朝上与静态+9.82m/s²相符。完整限制及数据见[IMU烧录验证](../../tests/firmware/2026-11-18_imu_flash_verification.md)。
