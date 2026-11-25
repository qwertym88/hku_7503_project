# 底盘固件：USB 串口、四轮 PID、原始里程计与板载 IMU

2026-11-18，根据用户重新测量更新：**左右轮组中心间距200mm、前后轮组中心间距170mm**，标称轮径65mm。前后间距由首次提供的165mm修正为170mm；旋转几何系数由0.1825m更新为0.185m。测量应以轮组中心为基准。保留官方不横移模型，`linear.y`必须为0。

当前为底盘bring-up固件，包含`/cmd_vel`订阅、四轮编码器PID、20Hz目标频率的原始`/odom`、50Hz目标频率的板载IMU `/imu/data`发布与电机任务超时停车，另有诊断ADC估算。2026-11-25版新增通信断开后软件复位和带锁重连。旧版通信故障后仍需按板上RESET。方向、频率、精度和停车延时以实测为准。

## 参数在哪里

| 参数 | 文件 | 当前初值 |
|---|---|---|
| 左右间距 | `components/car_motion/car_motion.h`，`ROBOT_WIDTH` | 0.200m |
| 前后间距 | 同上，`ROBOT_LENGTH` | 0.170m |
| 旋转几何系数 | 同上，`ROBOT_APB` | 自动计算(W+L)/2 = 0.185m |
| 轮周长 | `components/motor/motor.h`，`MOTOR_WHEEL_CIRCLE` | π×63.5 ≈ 199.4911mm，按实测直径64mm试用63.5mm有效滚动直径 |
| 编码器每输出轴转计数 | 同上，`MOTOR_ENCODER_CIRCLE` | 1040，假设官方310电机、13线×20减速×4倍频，须转轮实测 |
| PID周期 | 同上，`MOTOR_PID_PERIOD` | 10ms |
| 速度命令有效期 | 同上，`MOTOR_COMMAND_TIMEOUT_US` | 480ms，由独立电机任务每10ms检查；<0.5s仍待实测 |
| 车身命令限值（本地试验版） | `components/car_motion/car_motion.h` | ±0.45m/s，±1.50rad/s；`main/main.c`按这两个常量截断 |
| 单轮目标速度限值（本地试验版） | `components/motor/motor.h`，`MOTOR_MAX_SPEED` | ±0.75m/s；不是实测可达速度 |
| 通信/domain ID | `main/Kconfig.projbuild`，可用menuconfig修改 | UART0，921600，ROS_DOMAIN_ID=42 |

修改几何常量后重新build、flash。这里的旋转系数沿用官方模型，空/带载和实际地面仍需转角标定；不是精度保证。四轮方向沿用官方板卡：M1左前、M2左后、M3右前、M4右后。编码器和电机反向取决于实际装配，首次架空验证。

## 2026-11-23 路线速度预算试验版

按用户要求提高软件限值，以覆盖当前仿真的0.45m/s空载、0.40m/s带载、0.18m/s接近与1.50rad/s转向。旧±0.10m/s、±0.40rad/s为此前烧录版本的限值；本次仅本地修改和编译，小车已关机，未连接、未烧录、未进行高速实测。发送低速命令仍按低速执行，提高上限不改变上层命令值。

当前路线含29段直行（总16.6384m）、23段转向（绝对角度总32.4489rad）和6.4s显式等待。按每段配置速度并考虑固件限幅，旧版匀速理想预算253.91s，新版78.97s（直行50.94s、转向21.63s、等待6.40s）。这满足90s**匀速预算**，未包含加减速、减速到位、停稳、照片和升降调用的额外耗时；同一仿真配置既有整轮实测约144s，故不承诺实车90s完成，也没有修改180s任务截止时间或仿真路线。

单轮限值一起提高，因为轮目标为`vx ± 0.185*wz`。车身两项同时到上限时最大轮速0.7275m/s，需要低于单轮0.75m/s限值；若只提高车身限值，旧0.20m/s单轮截断会继续限制运动。电机额定/负载下的实际能力仍待测，不把软件上限作为硬件能力证据。

离线复算（仅依赖Python及PyYAML，不启动ROS）从仓库根目录执行：

```bash
python3 tools/chassis_route_budget.py
python3 tools/chassis_route_budget.py --linear-limit 0.10 --angular-limit 0.40 --wheel-limit 0.20
```

脚本读取当前固件常量和路线配置，输出逐段有效速度、时间分项与90s预算判断。详见[本次构建与验证记录](../../tests/firmware/2026-11-23_speed_budget.md)。

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

完整手动测试顺序、命令与通过标准见[M2手动测试步骤](../../docs/给队友看/M2手动测试步骤.md)。

以下为待执行步骤；用户2026-11-19已换新电机，先确认新 M4 及四轮反馈正常。既有单轮检查不等于四轮闭环验收。

Pi/电脑先安装Jazzy的micro_ros_agent。在已经source ROS2 Jazzy的另一个终端运行（与编译终端分开）：

```bash
export ROS_DOMAIN_ID=42
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

启动agent，等待节点出现。先只观察：

```bash
ros2 node list
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 topic info /cmd_vel -v
```

确认`/odom`是本固件唯一发布，frame为`odom`/`base_footprint`。本固件不发TF；首次架空测试不需要启动EKF或导航。后续按当前接口文档接EKF，不直接套用旧的将滤波结果重映射回`/odom`的launch。

### Agent 重启后的恢复

检测到通信故障后，固件切断驱动并释放舵机 PWM，随后软件复位、等待 Agent 并重建 micro-ROS 会话。恢复后 `/odom` 从零开始，升降位置为未知，诊断中的 `command_locked=true` 表示 `/cmd_vel` 和 `/lifter` 仍被屏蔽。通信恢复期间会保留已锁存的升降超时和 PWM 故障。

退出固定路线、遥控和所有动作测试程序，确认轮子停稳、机构及周围安全，至少 1 秒没有收到上述两类指令后，调用：

```bash
ros2 service call /chassis/unlock std_srvs/srv/Trigger '{}'
```

解锁本身不驱动车轮或舵机。确认诊断变为 `command_locked=false` 后，再进行新动作。使用固定路线前，重新初始化定位，在机构可安全下降时发送一次 `/lifter=0`，确认机构实际低位且诊断为 `lifter_position=0`。若 `lifter_fault` 为 `timeout` 或 `pwm_error`，先排查固件和硬件故障，通信解锁不会清除这些故障。

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

如果容器已经运行，不重复启动；`docker logs -f --tail 30 hku-chassis-agent`查看，`docker stop hku-chassis-agent`停止。退出日志窗口不等于停止后台容器；`--rm` 容器停止后被删除，重新连接用 `docker run` 重建。复连异常先 `docker ps -a` 排除其他 agent/串口占用，再按本页恢复流程处理；仅换名字会让多个进程争抢同一串口。

## 当前硬件状态与历史排查

2026-11-19历史记录：用户已更换新电机；当时已烧录含反馈保护及 IMU 的正常 `/chassis` 固件。M1～M3 正常由用户此前确认，新 M4 反馈、四轮闭环和停车需复测。只读检查约50Hz IMU、20Hz odom，新故障位为0，未发送速度命令。版本与烧录见[本轮记录](../../tests/firmware/2026-11-19_feedback_guard.md)，分项交付见[M2清单](../../docs/给队友看/M2底盘进度与交付清单.md)。

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
- 保留传感器原生轴方向；当前陀螺零偏已补偿，加速度保留重力；重新校准流程见下文。`base_link→imu_link`安装外参及静态偏置须实测，当前URDF中的identity不能当作已确认结果；未将IMU混入固件原始odom。
- 启动未就绪、WHOAMI/自检失败、I2C读取失败、样本过期100ms或重复序号时不发布假数据。IMU故障本身不改变当前底盘指令接受逻辑；必要IMU断流由任务层停车处理。
- `/chassis/diagnostics`新增`imu_state`（0初始化/1已配置/-1初始化失败）、`imu_whoami`、`imu_valid`、`imu_age_ms`（无样本为-1）、累计`imu_errors`、`imu_published`；`imu_state=1`不等于一定有新鲜数据，要结合valid/age/计数判断。
- micro-ROS池容量现为5 topics、3 publishers、2 subscriptions、3 services，自定义MTU为2048；`idf.sh`检查共享缓存容量并同步头文件，避免motor_test构建留下较小容量。两个工程不要并发构建。

正常固件已烧录；2026-11-19用户完成陀螺静态校准、板端补偿与静态复测，见[完成记录](../../tests/firmware/2026-11-19_imu_calibration.md)。安装外参/旋转符号、重启与温度重复性仍需验证。再次烧录沿用本文命令；M4 已更换但反馈待复测，IMU检查不发送运动命令，可手动转动整块板/车体。

烧录并启动agent后，在已source ROS、domain42的终端：

```bash
ros2 topic echo /chassis/diagnostics --qos-reliability best_effort --full-length
ros2 topic echo /imu/data --qos-reliability best_effort --once
ros2 topic hz /imu/data
```

需实测：静置时陀螺零偏、加速度模长约9.81m/s²、绕各轴旋转的符号与安装方向、发布频率/最大间隔/时间戳年龄；确认odom约20Hz与电机看门狗未受影响。拔掉IMU/注入总线故障须在台架条件进行，验证没有旧数据被重打时间戳。当前已完成静态陀螺零偏补偿复测，不计完整安装方向、加速度六面标定或整车验收。

2026-11-18静态实测：IMU约49.992Hz、odom约19.989Hz，WHOAMI=0x67，imu_errors=0。陀螺均值XYZ为[-0.013751,-0.005066,+0.024554]rad/s，当时尚未扣除；用户确认X前/Y左，Z朝上与静态+9.82m/s²相符。完整限制及数据见[IMU烧录验证](../../tests/firmware/2026-11-18_imu_flash_verification.md)。


## 轮子安装方向与编码器反馈（2026-11-19修订）

用户已查明换轮布局后未按固件轮ID接线，纠正接线后四轮向车头滚动时原始计数均增加。已参考最近提交 `4943022` 移除后来增加的四轮反馈取反，恢复每周期原始count增量直接用于PID与速度/里程计；诊断 `counts`、`target`和`speed`的前进方向均为正。累计count本身可为负，判断方向应看增量。

编码器、PWM驱动和运动学文件与 `4943022` 逐字节一致；保留看门狗、升降互锁、已校准IMU和位置舵机。用户已确认当前基本移动及简单定时前进/右转初步有效，精度和物理停车仍待量测，详见[方向恢复记录](../../tests/firmware/2026-11-19_encoder_baseline_restore.md)。此前基于误接线观察所做的-1取反修订已作废；供电与物理停车仍需独立验收。

## 电机反馈与停车行为（2026-11-19移除反馈锁存）

按用户要求，当前源码已移除“300ms无反馈停车锁存”和“反馈反向停车锁存”，此前未提交的1秒启动宽限也一并移除。轮编码器仍用于PID和原始odom；无计数/反向计数本身不再让固件锁住四轮，也不再通过轮反馈故障触发升降故障。

保留480ms速度命令超时、显式零速停车、通信异常停车及升降互锁。单独的速度命令超时后，新有效命令可恢复。通信异常后的恢复见本页 Agent 重启流程。没有独立编码器丢失/反向自动停车或堵转检测，不将持续速度发布视为异常情况下的停止保证。

诊断新增 `feedback_guard_enabled=false` 明示移除状态，保留 `counts`、`target`、`speed`、`pid_output`；`lift_inhibited`表示当前升降互锁。原 `feedback_missing_mask` / `feedback_reversed_mask`字段已移除。此前带mask的日志是历史版本，不能用于判断当前功能存在。

1. 架空核对M1左前/M2左后/M3右前/M4右后，手转确认前进count增加。
2. 停止其他速度发布者，source ROS并设置domain42，每次仅一个短动作：

```bash
/usr/bin/python3 tools/chassis_bench_test.py forward --confirm-wheels-raised \
  --duration 1 --output "/tmp/forward_$(date +%Y%m%d_%H%M%S).json"
```

`reverse`、`left`、`right`分别测试后退、原地左/右转，单独观察再执行。工具检查新鲜诊断和唯一速度发布者/单一订阅者，兼容已移除mask的新固件；历史固件非零mask仍会拒绝。结束尽力持续发零，但外部物理方向/停止仍需观察。

3. 用户确认移除版基本移动及简单定时前进/右转有效，详见[现场确认](../../tests/firmware/2026-11-19_motion_recovered.md)；诊断 `feedback_guard_enabled=false` 可核对版本。实现与软件检查见[移除记录](../../tests/firmware/2026-11-19_feedback_guard_removed.md)。

## 陀螺零偏的标定与启用

`main/imu_calibration.h` 是补偿配置，当前 `CHASSIS_IMU_BIAS_ENABLED=1`。2026-11-19静态标定及板端复测完成，XYZ零偏为[-0.0119512194,-0.0046553588,+0.0230571526]rad/s，Z残余约0.03825°/s，详见[校准记录](../../tests/firmware/2026-11-19_imu_calibration.md)。以下流程只供将来重新标定：先将开关设0、重编译烧录，再采集新的原始数据。不要用历史值或已补偿数据重新估计原始零偏；加速度保留重力，不用静态XYZ均值直接归零。诊断 `imu_bias_enabled` / `imu_bias` 可核对当前开关和三轴rad/s配置。

仅在需要重新标定时，保持车体绝对静止、无速度发布者，未补偿固件RESET后确认imu_bias_enabled=0、rx=0、enabled=0，再在仓库根目录采集：

```bash
python3 tools/chassis_imu_probe.py --duration 35 --warmup 5 \
  --label stationary --output /tmp/imu-raw-20261119.json
python3 tools/chassis_imu_calibrate.py /tmp/imu-raw-20261119.json \
  --confirm-stationary --header /tmp/imu-calibration-candidate.h
```

校准器要求至少1000样本/20秒、健康IMU、无速度发布者/电机命令/轮计数变化，并检查陀螺噪声/扰动、加速度稳定和时间戳间隔。候选头文件只能写到新路径；数值条件无法识别持续极慢旋转，仍须独立静止确认。生成成功后先检查结果与环境，把候选内容复制到 `firmware/chassis/main/imu_calibration.h`，重新build/flash，静置复测残余零偏，再做重启/温漂与受控转向。不要把已补偿数据重新当作原始偏置。本次已经完成该流程，后续优先做重启/方向验证。


## S1 180°位置舵机升降

`/lifter` UInt8（1升/0降）直接控制位置舵机，默认0°/80°，角度可在menuconfig配置，无需arm。首次指令前不输出PWM；等待稳定后持续保持目标脉宽，不返回1500µs。保留底盘互锁、stop撤销PWM和故障锁存，无外部到位反馈。

用户纠正实际为180°位置舵机，旧连续旋转配置不再适用。新版已在IMU补偿后记录中确认运行（当前下0°/上75°），端点和实际载荷仍待验证；UP_MS/DOWN_MS仅为稳定等待。配置、正常升降、停止/恢复及20循环表见[升降控制说明](LIFTER.md)。
