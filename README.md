# DASE7503 智能轮式物流机器人（第 X 队）

一台 30×30×18 cm 内的麦克纳姆轮机器人：**扫描起点二维码自启 → 自动导航到货架 → 扫码确认 → 舵机举升 → 送到目的区放下**，全程自动，比赛限时 3 分钟。

本仓库是团队唯一的代码/文档/图纸仓库。目标只有一个：**保底跑通 A 档自动流程**。

```text
Pi5（ROS 2 Jazzy）                         ESP32（micro-ROS）
 相机 → /qr   ┐                            4 电机 + 编码器 → /odom
 雷达 → /scan ├→ mission 状态机 → /cmd_vel ─►  舵机 ← /lifter
 定位 + Nav2  ┘
```

当前状态：**框架与文档已就绪，硬件与算法待 5 位队友按《分工与任务拆解》接入。**

---

## 第一次来，先看什么？

1. **想知道机器人怎么完成任务** → 《[任务流程说明](docs/给队友看/任务流程说明.md)》
2. **要接自己的模块** → 先看《[模块接口说明](docs/给队友看/模块接口说明.md)》，再找下表对应的目录
3. **想知道自己做什么** → 《[分工与任务拆解](docs/给队友看/分工与任务拆解.md)》
4. **想跑一遍 / 单独测自己的模块** → 《[运行说明](docs/给队友看/运行说明.md)》、《[单模块测试说明](docs/给队友看/单模块测试说明.md)》
5. **做底盘/固件（M2）** → 动手前先看《[底盘参考实现](docs/给队友看/底盘参考实现.md)》，**不要自己推麦轮逆运动学**

队友按自己那部分读上面 1–2 份即可；文档统一放在 `docs/给队友看/`。

---

## 你做哪部分，就从哪里开始

| 角色 | 先看这些文件 | 需要完成什么 |
|---|---|---|
| **机械 M1** | `cad/`、《[分工与任务拆解](docs/给队友看/分工与任务拆解.md)》第 2 节 | 底盘 + 举升机构 + 装配；SolidWorks pack-and-go |
| **硬件/嵌入式 M2** | `firmware/`、《[底盘参考实现](docs/给队友看/底盘参考实现.md)》、《[里程计标定](docs/给队友看/里程计标定.md)》、《[模块接口说明](docs/给队友看/模块接口说明.md)》 | 线束供电；ESP32 固件：`/cmd_vel`→电机、编码器→`/odom`、`/imu/data`、`/lifter`→舵机 |
| **感知 M3** | `ros2_ws/src/qr_perception/`、《[QR 感知方案](docs/给队友看/QR感知方案.md)》、《[模块接口说明](docs/给队友看/模块接口说明.md)》 | picamera2 取流 + 纯 Python 解码 + 薄封装 → `/qr`（**不用 camera_ros**） |
| **导航 M4** | `ros2_ws/src/robot_navigation/`、`maps/`、《[导航栈方案对比](docs/给队友看/导航栈方案对比.md)》、《[仿真与离线测试方案](docs/给队友看/仿真与离线测试方案.md)》 | 雷达 → `/scan`；建图定位；点到点导航 → `/cmd_vel`（复用 linorobot2 的 Nav2/SLAM/EKF/Gazebo 栈） |
| **集成/PM M5** | `ros2_ws/src/mission_control/`、`tests/` | mission 状态机、联调、测试、文档、PPT |

> `ros2_ws/src/mission_control` 是**状态机**；`robot_bringup` 是**统一启动**；`robot_description` 是 **URDF/模型**；`robot_firmware_bridge` 是 **micro-ROS 桥接**。**不要把电机、导航、解码算法塞进状态机。**

---

## 仓库结构

```text
.
├─ README.md                  ← 你正在看的入口
├─ docs/
│  ├─ PROJECT_FRAMEWORK.md    需求解读与完整框架（背景备查）
│  ├─ PROJECT_PLAN_SLIM.md    精简执行计划（保底 A 档）
│  ├─ 给队友看/               上手文档（4 份 + 分工拆解）
│  └─ 自己的记录/             维护者的测试/构建记录（队友无需读）
├─ requirements/              原始需求 PDF（勿改）
├─ cad/                       SolidWorks 图纸与导出
├─ firmware/                  ESP32 micro-ROS 固件
├─ ros2_ws/src/               ROS 2 工作区
│  ├─ robot_bringup/          统一启动与配置
│  ├─ robot_description/      URDF / 模型
│  ├─ robot_firmware_bridge/  micro-ROS agent 桥接
│  ├─ qr_perception/          相机 + QR 解码
│  ├─ robot_navigation/       雷达 / 定位 / 导航
│  └─ mission_control/        任务状态机
├─ maps/                      建图结果（pgm/yaml）
└─ tests/                     测试用例与记录
```

---

## 快速开始（ROS 2 Jazzy）

```bash
# 在安装了 ROS 2 Jazzy 的 Ubuntu 24.04 或树莓派上
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py
```

详细操作、通信域、实机步骤见《[运行说明](docs/给队友看/运行说明.md)》。

---

## 提交与协作

- **接口冻结**：只保留 6 个话题（见《模块接口说明》），加话题要先在站会说一声。
- **分支**：`main` 稳定，`feat/<模块名>` 开发；合并前至少一人看过。
- **不要提交**：`build/`、`install/`、`log/`、`__pycache__`、运行日志（已在 `.gitignore`）。
- **贡献日志**：每周更新 `docs/CONTRIBUTION_LOG.md`（用于 5% 同伴互评）。

---

## 诚实声明（当前未完成的部分）

- 真实电机通信、二维码解码、举升固件、定位导航**尚未接入**，接口为空壳。
- 场地尺寸与货架几何**以实测为准**，文档中的数值来自需求图纸 OCR，仅供参考。
- 模拟/离线通过 ≠ 实物验收通过。

*组内项目，未指定开源许可证。*
