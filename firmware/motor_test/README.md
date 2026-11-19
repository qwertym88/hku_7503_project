> **当前状态（2026-11-19）**：板上已恢复含 IMU 的正常 `/chassis` 固件。用户修正线束后确认 M1～M3 电机/编码器正常、M4 编码器已换待复测。本工程保留为单轮排查工具；再次使用需临时烧录并记录版本，结束后恢复正常固件。详见[M2交付清单](../../docs/给队友看/M2底盘进度与交付清单.md)。

# M2 单轮 PWM 与编码器排查

2026-11-18。用于定位“ROS 能通信，但电机不转”。这是基于官方底层驱动的独立测试固件，**不是完整底盘固件，也不是未经修改的官方样例**。它不调用 `Motor_Init()`、不运行 PID、不订阅 `/cmd_vel`、不发布 `/odom`；只保留 USB micro-ROS、单轮 PWM 和原始编码器。构建与烧录都和 `firmware/chassis/` 分开。

## 官方依据

本机官方资料在 `~/Desktop/Samples.zip`，解压目录为 `~/Desktop/Samples./`：

- `esp32_samples/pwm_motor/main/main.c`：`PwmMotor_Init()` 后直接调用 `PwmMotor_Set_Speed_All()`，按前进、刹车、后退、滑行各 1 秒循环，输入为 `PWM_MOTOR_MAX_VALUE`（200）。
- `esp32_samples/encoder/main/main.c`：只初始化编码器，每 200ms 读取四路累计计数，不驱动电机。
- 官方教程：[07.Drive motor](https://github.com/YahboomTechnology/MicroROS-Board/blob/main/02.%20ESP32%20basic%20course/07.Drive%20motor.pdf)、[08.Read motor encoder data](https://github.com/YahboomTechnology/MicroROS-Board/blob/main/02.%20ESP32%20basic%20course/08.%20Read%20motor%20encoder%20data.pdf)。

已逐字节比对：`chassis/components/pwm_motor/{pwm_motor.c,pwm_motor.h}`、`encoder/{encoder.c,encoder.h}` 与上述官方样例一致，`espressif__bdc_motor/src/bdc_motor_mcpwm_impl.c` 也与官方样例的 managed component 一致。本工程直接引用这些现有组件，不修改其引脚或死区。

不能在现有 `chassis/main/main.c` 中仅插入 `PwmMotor_Set_Speed()`：`Motor_Init()` 创建的 PID 任务每 10ms 都会重新设置 PWM／停车，直接测试会被覆盖。

## 测试接口与限幅

沿用 UART0 GPIO43/44、921600、domain 42，现有 Docker agent 无需更换镜像。

| 接口 | 内容 |
|---|---|
| 节点 `/chassis_motor_test` | 与正常固件 `/chassis` 区分 |
| 订阅 `/chassis/motor_pwm` | `std_msgs/msg/Int32MultiArray`，固定 `[M1,M2,M3,M4]`，layout 为空；每次只允许一路非零 |
| 发布 `/chassis/diagnostics` | 10Hz、best effort、JSON；`mode=open_loop_test`、counts、enabled、latched、rx、rejected、fault、pwm_input、pwm_ticks |

- 上电不驱动；先收到全零数组才解锁。停止状态为滑行，方便轻转轮轴测编码器；滑行不等于机械刹停。
- 输入限制为每路 ±100；超范围或多轮同时非零则停止并锁定。
- 保留官方 **200 tick 死区补偿**，周期为 400 tick：输入 `40` 对应软件 PWM 命令 `240/400=60%`，**不是 40%**。输入 `-40` 是同幅反向，0 是停止。测试上限 100 对应 75%；官方原样例的 200 对应 100%。
- 非零指令断流 480ms 后关闭驱动，持续刷新也最多驱动 1 秒；由独立 10ms 任务检查，之后必须先发送零才能再次运行。
- 运行期间改变通道、方向或幅值也会停车并锁定，须先发零。ROS 错误／agent 掉线后保持停止，需要 RESET 恢复。
- `pwm_ticks` 是软件下发值，不是 GPIO 波形、电机端电压或转动成功的证明。硬件未运动时不能宣称停车时间验收通过。

## 构建、烧录与恢复

在仓库根目录构建，依赖沿用 `chassis/README.md` 的本机 ESP-IDF5.1.2 和 micro-ROS 环境：

两个工程共用本机 micro-ROS 构建缓存，请顺序构建，不要同时运行 build。

```bash
bash firmware/motor_test/idf.sh build
```

烧录前确认车轮全部架空、升降负载隔离，核对电池、主电源及电机插头。四路轮序为 M1左前／M2左后／M3右前／M4右后；按官方线束白头接控制板、黑头接电机。改接线先断电。

**烧录会替换板上的正常底盘应用。** 先关闭串口占用者；如当前容器名为 `hku-chassis-agent`：

```bash
docker stop hku-chassis-agent
bash firmware/motor_test/idf.sh -p /dev/ttyUSB0 flash
```

串口名以本机实测为准。不要 `erase-flash`，不要将 `motor_test.bin` 应用镜像作为合并镜像写入 0x0。重新用原来的 Docker 命令启动 agent；如果容器使用 `--rm`，停止后需要重建，不能 `docker start`：

```bash
docker run --rm --name hku-chassis-agent --network host --device /dev/ttyUSB0 \
  -e ROS_DOMAIN_ID=42 microros/micro-ros-agent:jazzy \
  serial --dev /dev/ttyUSB0 -b 921600 -v4
```

agent 启动后按 RESET。UART0 专用于 micro-ROS，不同时开 `idf.py monitor`。另一个终端：

```bash
source /opt/ros/jazzy/setup.bash  # zsh 终端改为 setup.zsh
export ROS_DOMAIN_ID=42
ros2 node list
ros2 topic echo /chassis/diagnostics --qos-reliability best_effort
```

必须看到 `/chassis_motor_test` 与 `mode: open_loop_test`。如果仍是 `/chassis` 或旧的 PID 诊断，说明尚未运行测试固件；当前 `chassis` 固件不会接收下面的 PWM 话题。

恢复正常固件时，同样先停止 agent，再执行：

```bash
bash firmware/chassis/idf.sh -p /dev/ttyUSB0 flash
```

之后重建 agent 并 RESET。测试工程没有修改 `chassis/main/main.c` 或正常固件的控制逻辑。

## 先测编码器，再测单个电机

1. 不发布任何 PWM，确认 `enabled=0`、`pwm_input=[0,0,0,0]`。给编码器供电，轻转**轮轴／轮毂**，不要只拨麦轮小辊子；观察对应一路 `counts` 是否连续变化，反向转动时变化方向是否反转。不要强拧减速箱。
2. 若手转有计数，说明对应编码器供电、信号和 PCNT 采集在本次测试中有效，仍需独立验证电机驱动。
3. 先测 M1，保持其他轮悬空；在发布命令的终端执行：

```bash
# 停止并解锁；在诊断终端确认 latched=0、enabled=0。
ros2 topic pub --times 5 -r 20 /chassis/motor_pwm \
  std_msgs/msg/Int32MultiArray '{data: [0, 0, 0, 0]}'

# M1 正向：10 条、20Hz，约半秒；同时观察轮子和 counts。
ros2 topic pub --times 10 -r 20 /chassis/motor_pwm \
  std_msgs/msg/Int32MultiArray '{data: [40, 0, 0, 0]}'

# 主动关闭驱动；不依赖超时等待。
ros2 topic pub --times 5 -r 20 /chassis/motor_pwm \
  std_msgs/msg/Int32MultiArray '{data: [0, 0, 0, 0]}'
```

确认 `rx` 增长、`rejected` 未增长、运行时 `enabled=1`、`pwm_ticks=[240,0,0,0]`，结束后回到全零。测试期间不要运行其他 PWM 发布者。原来的 `/cmd_vel` 在这个固件中不控制电机。

4. 只有 M1 正常后，再用 `[-40,0,0,0]` 测反向；每次先全零。其余电机分别为 `[0,40,0,0]`、`[0,0,40,0]`、`[0,0,0,40]`。逐个测试，不自动循环加速。

## 根据结果定位

| 观察 | 下一步 |
|---|---|
| 手转有计数，PWM 能转 | 单轮电机与编码器的基本链路成立，恢复正常固件后检查闭环与方向 |
| 手转有计数，PWM 不转 | 用万用表核对电机电源输入、开关、线束和驱动输出；软件下发值不能排除驱动故障 |
| PWM 能转，计数不变 | 检查编码器电源、AB 线、插头和引脚，不要继续闭环提高 PID |
| 四路都不转，手转也无计数 | 先检查共同供电／地和连接，不据此断言四个电机都损坏 |
| 诊断 rx 不增加或 mode 不对 | 先查固件、domain、话题类型、agent 与订阅者 |
| 当前正常固件可手转计数，但低速闭环不转 | 记录实际 PID／PWM；用本测试排除闭环影响，不能只凭 0.03m/s 目标判断电机坏了 |

断电后才能换接电机与通道做交叉排查。将通道、方向、counts 前后值、轮子观察、电池／电机端实测电压分别记录；不要用软件推算电压替代万用表。

构建通过仅证明可编译；烧录、PWM 波形、轮子运动与物理停止效果均需单独记录实测结果。

2026-11-18已完成本机交叉编译、控制逻辑主机检查及正常底盘固件构建回归，证据见[测试准备记录](../../tests/firmware/2026-11-18_motor_test_preparation.md)。后续经用户确认架空供电，已烧录测试版并执行一次M1约0.5秒测试：板端确认单轮PWM软件输出，用户确认完全无动作，四路counts仍为0；结束后软件输出全零。用户随后报告可正反手转轮毂，本次17秒只读窗口仍未采到编码器变化。当时保留测试固件并待检查线束与接口供电；此为历史失败记录，后续状态以上方说明为准，详见[M1实测记录](../../tests/firmware/2026-11-18_m1_pwm_test.md)。

2026-11-18 13:57历史复测：用户重新检查线路后，M3单向约0.5秒测试采到编码器从0增加至574，其他三路保持0，结束后输出全零且计数稳定；用户确认右前轮转动并已停下，本次M3单向开环与反馈基本检查通过。反向、PID闭环、其他轮和停车时延未验收。详见[电机M3复测记录](../../tests/firmware/2026-11-18_m3_pwm_test.md)。
