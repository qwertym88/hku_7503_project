# mission_control：固定顺序取放（M5）

2026-11-21：已实现**仿真版**状态机、固定路线反馈控制、YAML校验、到位拍照、运动学搬运和日志。完整构建、开始/取消/复位命令与参数说明见[固定路线仿真使用说明](../../../docs/给队友看/固定路线仿真使用说明.md)，统一配置为[fixed_route_sim.yaml](config/fixed_route_sim.yaml)。当前只允许仿真运行，下面真实EKF/AMCL与硬件联合交付仍待完成。

M5负责按A→B→C→D提交M4的固定路段，管理停靠等待、进入0.10m、举升、离架、运输、放下、退出0.10m；统一180s计时、每步超时和失败锁存。不等待/qr、不调用Nav2规划、不自动重排/跳架。真雷达、EKF局部反馈与AMCL实时定位在执行模式中必需同跑；不要求厘米级定位，定位真实可用性与取放结果分别验收。

M4提供非阻塞执行组件，使用/odometry/filtered执行短动作、检查定位链路健康；M5在一个mission进程导入；M4唯一发/cmd_vel，M5只调度并发/lifter。失败/取消/截止即stop，保持载荷支撑；人工复摆复位后才重启。定义见[接口](../../../docs/给队友看/模块接口说明.md)。

交付：配置校验、dry-run、实车运行/取消/复位命令、统一日志、单A架和四架证据。实现后把真实命令补到[运行说明](../../../docs/给队友看/运行说明.md)，不能把不存在的ros2 run命令当可用入口。

职责/日期见[任务安排](../../../docs/给队友看/分工与任务拆解.md)，完整流程见[任务流程](../../../docs/给队友看/任务流程说明.md)，验收见[证据协议](../../../tests/保守方案验收.md)。

M2已改为180°位置舵机升降（已见板端诊断，实物端点/载荷待验收）。无需arm；升降前停稳，直接单次发 `/lifter` UInt8 1/0，默认目标80°/0°（menuconfig可设），等待实测稳定时间加余量，结束保持目标。任务取消应保持载荷支撑；人工维护接口 `/lifter/stop` 会撤销PWM，不能保证载荷保持，不作为任务取消的默认动作。不要用0作为停止，也不把等待结束当作传感器到位。完整接口/恢复限制见[升降说明](../../../firmware/chassis/LIFTER.md)。

仿真实现：`route.py`校验并将固定航点展开为独立drive/turn；`core.py`提供非阻塞状态机和反馈控制；`node.py`适配ROS、相机与仿真货架服务；`validate.py`执行无ROS演练并输出SVG/动作JSON。仿真读取原始`/odom`，不会冒充真实EKF/AMCL已通过。

### 真机固定距离/转角闭环测试

需要验证 sim 的 `drive` / `turn` 动作能否在真机完成时，使用 `chassis_segment`。
它读取同一份 YAML 的控制参数，复用 `motion_request` / `ramp_command`，以每次启动时的
车体位姿为原点，20 Hz 根据实时 `/odom` 更新命令，接近目标减速，到位后连续发零速并确认停稳。
距离不是通过“速度 × 发布条数”决定。当前配置的位置容差为 0.006 m、转角容差 0.012 rad；
这些是**里程计坐标中的完成条件**，不代表实车已有同等精度。`--speed` 是速度上限，
不是整段恒定速度；其余加速度、减速控制律、航向纠偏和容差保持与 sim 相同。

这是空旷地面的独立底盘测试，不启动完整货架任务、不执行升降，也没有接入雷达避障。
运行时操作员应在车旁，车前后留出测试距离以外的停车余量。先测 0.20 m/s，再测 0.40 m/s。
出现超调、横向/航向偏差超限、反馈过期、进展停滞、指令发布冲突或超时会停车并报告失败。
执行期间不启动其他 `/cmd_vel` 发布程序。

在本机仓库根目录，把测试模块和配置复制到现有 Pi 测试目录（不需要重刷固件）：

```bash
scp ros2_ws/src/mission_control/mission_control/{chassis_segment,motion,route}.py \
  ubuntu-pi@10.133.206.59:hku_7503_project/chassis_test/mission_control/
scp ros2_ws/src/mission_control/config/fixed_route_sim.yaml \
  ubuntu-pi@10.133.206.59:hku_7503_project/chassis_test/fixed_route_segment.yaml
ssh ubuntu-pi@10.133.206.59
```

Pi 上加载环境，确认 Agent 为 active；操作员摆好车并准备测量后执行：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd ~/hku_7503_project/chassis_test
mkdir -p logs
systemctl --user is-active hku-chassis-test-agent

# 先预览：省略 --run，不连接 ROS，不发速度。
python3 -m mission_control.chassis_segment drive 1.0 --speed 0.20 \
  --config fixed_route_segment.yaml --output logs/preview_1m.json

# 地面闭环前进 1 m，先测低一档速度。
python3 -m mission_control.chassis_segment drive 1.0 --speed 0.20 \
  --config fixed_route_segment.yaml --run hardware --output logs/drive_1m_020_01.json

# 测量并重新标记起点后，再测 0.40 m/s 上限。
python3 -m mission_control.chassis_segment drive 1.0 --speed 0.40 \
  --config fixed_route_segment.yaml --run hardware --output logs/drive_1m_040_01.json
```

每次运行都以本次静止姿态为原点，不要求重启或清零 `/odom`。
已有输出文件会被拒绝覆盖；重复试验将末尾 `01` 换为 `02`、`03` 等。
`drive -0.35 --speed 0.18` 测倒退 0.35 m；
`turn 90 --speed 0.60` 测左转 90°（角度参数单位为度，转速上限单位为 rad/s），
同样补上 `--config`、`--run hardware` 和新的 `--output`。
若在理想仿真验证同一动作，加载仿真工作区、使用仿真 domain、关闭控制面板及任务发布器，
将 `--run hardware` 改为 `--run sim`。构建后也可用 `ros2 run mission_control chassis_segment`。

地面用同一个车体参考点标记起点和终点，并量前向距离、横向偏移和车头转角。
每档速度至少重复 3 次，比较平均偏差、最差偏差和重复性。
报告的 `odom_end` 是停车后复测位姿，`odom_minus_goal` 是其相对目标的差（m/m/rad）；
`odom_goal_reached` 仅表示里程计闭环完成。`commands` 和 `odometry` 保存全过程，
`physical_measurement` 保持待人工测量，不能当作实车成功。
`goal_reached_s` 是动作到位并停稳的耗时，`elapsed_s` 还包括随后约 1 s 的持续零速复测。
例如目标 1 m、里程计约 1 m、卷尺约 0.96 m，应归为定位比例/滑移误差；
若里程计也明显越过目标，则先查控制、减速和实际停车响应。
1 m 测试通过后再测路线实际使用的 0.16、0.20、0.25、0.35 m 短段，
以及带载和地面变化；短段常常达不到设置的最高速度，应同时记录动作耗时。

### 底盘同速度序列响应对照（不是定距闭环验收）

`chassis_compare` 用任务的转向和直行控制律，在理想底盘上预生成 20 Hz 的速度序列。测试速度上限为 0.05 m/s 和 0.20 rad/s。两端回放同一序列，分别记录速度命令、发送时刻和里程计。回放期间里程计用于测量与异常停车。这样可以先量出相同输入的响应差异，再用真实反馈做闭环。

两个目标都以每次启动时的车体为参考，前为正 X，左为正 Y。`forward_1m` 直行 1 m，`diagonal_1m` 先左转 45° 再走向 (1 m, 1 m)。命令在离目标 2 cm、离目标转角 0.02 rad 的范围内结束，所以理想参考终点也会有少量误差。报告分别保存理想终点与目标的差、实测里程计与理想终点的差。

仿真使用独立 ROS domain。启动 `sim_ideal.launch.py` 时设 `panel:=false`，保持仿真实时运行，选择有净空的起点。另一个已加载工作区环境的终端使用同一 domain 运行：

```bash
ros2 run mission_control chassis_compare forward_1m --run sim --output /tmp/sim_forward.json
```

在已启动 Agent 的树莓派上，当地面测试路径有净空、操作员在车旁时，运行：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd ~/hku_7503_project/chassis_test
python3 -m mission_control.chassis_compare forward_1m --run hardware --output "logs/forward_$(date +%Y%m%d_%H%M%S).json"
```

对角测试把目标名换成 `diagonal_1m`，并换一个输出文件名。省略 `--run` 时只生成速度序列和理想参考，不连接 ROS。比对报告的 `profile_id` 确认速度序列一致，再查看 `odom_minus_ideal`。实车报告基于 ESP32 的轮式里程计。请同时在地面标记起点、车头方向和停车点，测量前向、横向及转角误差，以识别打滑和里程计比例误差。
