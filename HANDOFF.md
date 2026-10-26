# 开发交接与环境衔接（换设备先看这一页）

> 唯一目的：**在一台全新机器上，半小时内恢复到"能继续开发"的状态。**
> 配套：一键环境重建脚本 [`tools/bootstrap_jazzy.sh`](tools/bootstrap_jazzy.sh)。

---

## 0. 当前真实状态（诚实版）

**仿真场地已完成并验证；6 个 ROS 包仍是只有 README 的空壳，底盘/感知/导航算法尚未落地。**

| 模块 | 责任人 | 状态 | 位置 | 备注 |
|---|---|---|---|---|
| 机械 CAD | M1 | ⬜ 未开始 | `cad/` | 只有 README + `exports/.gitkeep` |
| 固件 / 底盘 | M2 | ⬜ 未开始 | `firmware/` | 只有 README；`COUNTS_PER_REV = 1040` 已记录 |
| 感知 QR | M3 | ⬜ 未开始 | `ros2_ws/src/qr_perception/` | 只有 README（方案已定：picamera2 + 纯 Python，不用 `camera_ros`） |
| 导航 | M4 | 🟡 **仿真场地 ✅，导航栈未开始** | `sim/` ✅、`ros2_ws/src/robot_navigation/` ⬜ | 场地已验证；Nav2/SLAM/EKF 待接 |
| 集成 / 状态机 | M5 | ⬜ 未开始 | `ros2_ws/src/mission_control/` | 只有 README |
| 联调测试 | M5 | ⬜ 未开始 | `tests/` | 只有 README + 两个空目录 |

> ⚠️ **因为包都还是空壳，`colcon build` 现在不会构建任何东西**（不报错，但也什么都没编出来）。这是正常的，不是环境装错了。

**场地世界的验证程度**（如实）：

| 项 | 状态 |
|---|---|
| 几何自检（出界 / 重叠） | ✅ 通过，退出码 0 |
| 脚本 ↔ SDF 一致性 | ✅ 逐字节一致（没有手改 SDF） |
| Fortress 版实跑 | ✅ 无界面跑 25 s，0 error / 0 warning |
| **Harmonic 版实跑** | ❌ **未实测**（此前开发机装不了 Harmonic）→ **换到 Jazzy 机器后第一件事就是跑一次** |

---

## 1. 换设备：照抄这 6 步

```bash
# 1) 拿到代码（见 §2：现在没有 remote，必须先建立！）
git clone <远端地址> 7139 && cd 7139

# 2) 一键装环境（Ubuntu 24.04 + ROS 2 Jazzy）
bash tools/bootstrap_jazzy.sh

# 3) 验证仿真场地能起（应看到 3.02×2.00 的横向场地）
source /opt/ros/jazzy/setup.bash
ros2 launch sim/launch/field_world.launch.py

# 4) 构建工作区（当前无包可编，属正常）
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# 5) 场地自检随时可跑（不需要 Gazebo）
python3 sim/scripts/generate_field_world.py --check

# 6) 读 §3 的不变量再动手，能省你半天
```

---

## 2. ⚠️ 第一优先：本仓库目前没有 git remote

实测 `git remote -v` 为空 —— **代码带不走**。换设备前必须二选一：

**A. 建远端（推荐）**

```bash
git remote add origin git@github.com:<你的账号>/<仓库名>.git
git push -u origin main
```

**B. 不能上网 / 不想上远端 → 打一个 bundle 单文件（含全部历史）**

```bash
git bundle create 7139.bundle --all
# 新机器上：
git clone 7139.bundle 7139
```

> ❌ **只拷目录是不够的**：会丢掉未提交的改动和 `.git` 历史。
> ✅ 用 bundle 或远端，两者都能完整还原 `main` 的 41 个文件与全部提交。

---

## 3. 不变量（换设备后也别动）

| # | 不变量 | 为什么 |
|---|---|---|
| 1 | **只做 ROS 2 Jazzy（Ubuntu 24.04）**。Humble / Fortress 相关文件保留但**不再维护、不再投入** | 需求 PDF 指定 Jazzy；linorobot2 的 `humble` 分支自 2025-02 已冻结 |
| 2 | 接口只有 **6 个话题**，全是标准消息：`/cmd_vel`(Twist) `/odom`(Odometry) `/scan`(LaserScan) `/imu/data`(Imu) `/qr`(String) `/lifter`(UInt8) | 冻结即可；零自定义消息 |
| 3 | 场地坐标**硬编码**在 `sim/scripts/generate_field_world.py` 的 `ZONES` / `RACKS` 两张表 | 已按图纸修正过两次，**禁止再用公式推导** |
| 4 | **不要手改 `sim/worlds/*.sdf`** | 它们是生成物，改脚本后重新生成 |
| 5 | 场地是**横向**：X = 3.02（长/横向）、Y = 2.00（宽/纵向），原点在**左下角** | 曾被改成竖向一次，别再翻 |
| 6 | 保底方案**不使用横向移动**（`linear.y = 0`） | 麦轮横移是加分项，不是保底 |
| 7 | **不引入 `ros2_control`** | 版本间硬件接口 API 大改；我们用 micro-ROS 直接吃 `/cmd_vel`，本来不需要 |
| 8 | Nav2 配置要**显式写死 `enable_stamped_cmd_vel: false`** | 否则新发行版 Nav2 默认发 `TwistStamped`，与冻结的 `/cmd_vel`（`Twist`）不符 → **车不动且不报错** |

---

## 4. 场地几何速查（M4 / M5 直接要用）

场地 `3.02 × 2.00 m`，原点左下角。四角：`(0,0)` `(3.02,0)` `(0,2.00)` `(3.02,2.00)`。

| 对象 | 中心 `(x, y)` | 边界 |
|---|---|---|
| Destination（绿，仅视觉） | `(0.40, 1.00)` | `x 0.00~0.80`，`y 0.30~1.70` |
| Start（蓝，仅视觉） | `(2.72, 1.00)` | `x 2.42~3.02`，`y 0.60~1.40` |
| Rack A（左上） | `(1.20, 1.58)` | `x 1.13~1.27`，`y 1.51~1.65` |
| Rack B（右下） | `(1.85, 0.51)` | `x 1.78~1.92`，`y 0.44~0.58` |
| Rack C（右上） | `(1.85, 1.72)` | `x 1.78~1.92`，`y 1.65~1.79` |
| Rack D（左下） | `(1.35, 0.37)` | `x 1.28~1.42`，`y 0.30~0.44` |

**通道净宽**（货架与围墙是实体，区域只是地面涂装可压过）：

| 通道 | 位置 | 净宽 | 机器人 0.30 m 能否过 |
|---|---|---|---|
| **中间东西向**（两排货架之间） | `y 0.58 ~ 1.51` | **0.93 m** | ✅ 宽裕 |
| 右侧南北向 | `x 1.92 ~ 3.02` | **1.10 m** | ✅ |
| 左侧南北向 | `x 0 ~ 1.13` | **1.13 m** | ✅ |
| **货架区上方** | `y 1.79 ~ 2.00` | **0.21 m** | ❌ **穿不过** |
| **货架区下方** | `y 0 ~ 0.30` | **0.30 m** | ❌ 刚好等于车宽，**不可通行** |

**两条重要结论**：

1. **`Start → Destination` 沿 `y ≈ 1.0` 的直线完全无遮挡**（正好走中间那条 0.93 m 通道）→ 保底任务的主路径是最简单的情况。
2. 上下两条缝（0.21 / 0.30 m）**比车窄**，规划器若不知道会试图钻进去卡死。
   → **起手 Nav2 参数建议**：`robot_radius = 0.15`、`inflation_radius ≈ 0.15` —— 膨胀后正好把上下两条缝封死，同时三条可用通道仍留 ≥ 0.6 m 净宽。**先在仿真里验，别在真机上撞墙再调。**

---

## 5. 关键决策记录（为什么是现在这样）

| 决策 | 结论 | 依据 |
|---|---|---|
| 麦轮逆运动学 | **不自己推**，用成熟库 + 参考 Yahboom 方案 | 《底盘参考实现》 |
| 感知取流 | **不用 `camera_ros`**：picamera2 + 纯 Python 解码，外层薄 ROS 封装 | 感知端不深绑 ROS，换个平台也不受影响 |
| M3 是否做仿真 | **不做**（用 `tests/` 里的离线图片集 + 评估脚本） | 省 2 人日 |
| M4 是否做仿真 | **做**（即 `sim/`） | 场地不开放，只能靠图纸 |
| 导航栈 | **复用 linorobot2**（Nav2 + SLAM + EKF + Gazebo 一套） | 省掉自己搭栈 |
| 通信域 | 仿真 `ROS_DOMAIN_ID=197`；实机 `42` | 避免互相串扰 |
| 仿真机器人模型 | 先用 linorobot2 的 `mecanum.urdf.xacro`，不等 M1 的 CAD | 解耦 |
| 仿真投入上限 | **2 人日**，绝不为仿真推迟真机调试 | 风险控制 |

---

## 6. 时间线（别忘）

- **提交截止：2026-11-26**
  - 17:00 前：实体机器人送到 **Lab HW 103A**
  - 23:59 前：CAD + 程序上传 **Moodle**
- 今天（2026-10-26）距截止约 **4.5 周**。

---

## 7. 已知开口问题（换设备后继续追）

| # | 问题 | 找谁 |
|---|---|---|
| 1 | 货架精确几何 / 重量；A、B 档的时间阈值；赛前是否允许预先建图；场地精确尺寸 | 问老师 |
| 2 | `microros_samples` 的来源 | 问老师 / 查资料 |
| 3 | 固件里 `MOTOR_ENCODER_CIRCLE` 宏未在 Yahboom 虚拟机里核实 | M2 |
| 4 | **Harmonic 版世界未实测** | 换到 Jazzy 机器后立刻跑（§1 第 3 步） |
| 5 | 图纸 `1.png` 未入库 → 放到 `docs/图纸/`（见该目录说明） | 你 |
| 6 | 本机未装 `robot_localization`（linorobot2 的 EKF 依赖） | bootstrap 脚本已包含 |

---

## 8. 换设备前顺手做的三件事

```bash
git status                 # 期望：干净（工作区无未提交改动）
git log --oneline -5       # 确认最新提交是你要带走的那一版
git push origin main       # 或 git bundle create（见 §2）
```

本仓库**没有**硬编码任何绝对路径 / 本机用户名（已检查），也没有提交 `build/`、`install/`、`log/`、`__pycache__`、`.vscode`（均在 `.gitignore`）。

---

## 9. 一句话版

> 换机器：**建 remote → clone → `bash tools/bootstrap_jazzy.sh` → 跑一次 `field_world.launch.py` → 读 §3 不变量。**
> 只做 Jazzy，别再碰横竖和跨版本。
