# sim —— Gazebo 仿真场地（M4）

把图纸变成可运行的 Gazebo 世界，用于**没场地时也能练导航**。

```text
sim/
├─ worlds/dase7503_field.sdf          ← Harmonic 版（生成物，别手改）
├─ worlds/dase7503_field_fortress.sdf ← Fortress 版（同上）
├─ scripts/generate_field_world.py    ← 唯一事实来源：改坐标改这里
└─ launch/
   ├─ field_world.launch.py           ← 只启动场地
   └─ field_with_robot.launch.py      ← 场地 + linorobot2 麦轮机器人（推荐）
```

**坐标约定**（与图纸一致）：原点在**场地左下角**，+X 向右（长 3.02），+Y 向上（宽 2.00），+Z 天向。围墙高 **0.5 m**，贴在场地**外侧**，内表面正好压在 0/3.02、0/2.0 上（不占场地面积）。

---

## 0. 两个变体，别搞混（附本机环境诊断）

两个世界文件**几何、尺寸、颜色完全一样**，只有 **SDF 版本号 + 插件命名**不同：

| 文件 | 目标环境 | SDF | 插件命名 | 启动命令 |
|---|---|---|---|---|
| `dase7503_field.sdf` | **Gazebo Harmonic**（Ubuntu 24.04 / ROS 2 Jazzy） | 1.10 | `gz-sim-*`、`gz::sim::systems::*` | `gz sim -r <file>` |
| `dase7503_field_fortress.sdf` | **Gazebo Fortress**（Ubuntu 22.04 / ROS 2 Humble） | 1.9 | `ignition-gazebo-*`、`ignition::gazebo::systems::*` | `ign gazebo <file>` |

重新生成：`python3 sim/scripts/generate_field_world.py --target harmonic`（或 `--target fortress`）

### 本机（开发机）诊断结论 —— 为什么 `gz sim` 用不了

| 项 | 实际情况 |
|---|---|
| 系统 | **Ubuntu 22.04.5 LTS (Jammy)** |
| `/usr/bin/gz` | **Gazebo Classic 11.10.2** 的 gz-tools v1。它的子命令只有 `help / camera / joint / log / marker / model / physics / sdf / stats / topic / world` —— **压根没有 `sim`**，这就是 `gz sim` 报 `Invalid arguments` 的原因 |
| Gazebo Classic | 11.10.2（`gazebo` / `gzserver` / `gzclient`，sdformat9）——**不是我们的目标，别用** |
| **Gazebo Fortress** | ✅ 装了：**Ignition Gazebo 6.18.0**，命令是 **`ign gazebo`**（不是 `gz sim`） |
| **Gazebo Harmonic** | ❌ **没装**。没有任何 `libgz-sim8` / `libgz-transport13` / `sdformat14`；anaconda 里那个 `sdformat14` 只是包缓存，conda base 里也没有 gz-sim |
| ROS 2 | **Humble**（`/opt/ros/humble`，含 `ros_gz_sim` / `ros_gz_bridge` → 对应 Fortress） |
| Harmonic 能装吗 | ❌ **apt 装不了**：Harmonic 配的是 Ubuntu 24.04 / ROS 2 Jazzy |

> ⚠️ **与项目要求不一致**：需求 PDF 明确写 **ROS 2 Jazzy**（Ubuntu 24.04），而这台机器是 **Humble**。
> 所以：**仿真最终要在 24.04 + Jazzy + Harmonic 的机器上跑**；这台机器适合做 ① Fortress 版布局核对 ② 与 ROS 版本无关的活（M3 纯 Python 图像算法、M2 的 ESP-IDF 固件）。
> linorobot2 同时有 **`humble` 和 `jazzy`** 两个分支（hardware 仓库也是），两条路都走得通——**但不要混用**。

---

## 1. 解算后的布局

`python3 sim/scripts/generate_field_world.py --check` 会打印下面这张按比例的俯视图（此处是它的输出）：

```text
+------------------------------------------------------------------------+
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG               CCCC               |
|                   GGGGGGGGGGGGGGGGGGGGG        AAAAA  CCCC               |
|                   GGGGGGGGGGGGGGGGGGGGG        AAAAA                     |
|                   GGGGGGGGGGGGGGGGGGGGG        AAAAA                     |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGGGGGGGGGGGG                                  |
|                   GGGGGGGGGGSSSSSSSSSSSSSSSS                             |
|                   GGGGGGGGGGSSSSSSSSSSSSSSSS                             |
|                   GGGGGGGGGGSSSSSSSSSSSSSSSS                             |
|                   GGGGGGGGGGSSSSSSSSSSSSSSSS          BBBB               |
|                             SSSSSSSSSSSSSSSS          BBBB               |
|           DDDD              SSSSSSSSSSSSSSSS          BBBB               |
|           DDDD              SSSSSSSSSSSSSSSS                             |
|                             SSSSSSSSSSSSSSSS                             |
|                             SSSSSSSSSSSSSSSS                             |
|                             SSSSSSSSSSSSSSSS                             |
|                             SSSSSSSSSSSSSSSS                             |
+------------------------------------------------------------------------+
  (0,0)=左下角  G=Destination绿  S=Start蓝  A/B/C/D=货架  场地 3.02 x 2.0 m)
```

| 对象 | x 范围 | y 范围 | 中心 | 平面尺寸 | 距左 | 距右 | 距底 | 距顶 |
|---|---|---|---|---|---|---|---|---|
| **Destination 区**（绿） | 0.800 ~ 1.600 | 0.600 ~ 2.000 | **(1.200, 1.300)** | 0.80 × 1.40 | 0.800 | 1.420 | 0.600 | **0.000** |
| **Start 区**（蓝） | 1.210 ~ 1.810 | 0.000 ~ 0.800 | **(1.510, 0.400)** | 0.60 × 0.80 | 1.210 | 1.210 | **0.000** | 1.200 |
| **RACK A** | 2.000 ~ 2.140 | 1.510 ~ 1.650 | **(2.070, 1.580)** | 0.14 × 0.14 | 2.000 | 0.880 | 1.510 | 0.350 |
| **RACK B** | 2.280 ~ 2.420 | 0.440 ~ 0.580 | **(2.350, 0.510)** | 0.14 × 0.14 | 2.280 | **0.600** | 0.440 | 1.420 |
| **RACK C** | 2.280 ~ 2.420 | 1.650 ~ 1.790 | **(2.350, 1.720)** | 0.14 × 0.14 | 2.280 | **0.600** | 1.650 | 0.210 |
| **RACK D** | 0.450 ~ 0.590 | 0.300 ~ 0.440 | **(0.520, 0.370)** | 0.14 × 0.14 | 0.450 | 2.430 | 0.300 | 1.560 |

> RACK B 与 RACK C **同 X（2.350）**，即"B 在 C 正下方"成立。

货架高 **0.2325 m**，建模为 4 立柱 + 3 层板（`<collision>` + `<visual>` 齐全）。
**区域标记无碰撞**（只有 `<visual>`）——机器人要能开过去。

---

## 2. 图纸坐标的由来与已知冲突（重要）

本版**直接硬编码图纸给定的中心坐标**，不再做约束推导。核对后仍存在 3 处需要你确认的地方：

| # | 情况 | 说明 |
|---|---|---|
| 1 | **Destination 与 Start 重叠** | 按给定中心算：Destination 覆盖 y 0.6~2.0，Start 覆盖 y 0.0~0.8，**两者在 x[1.21, 1.60] × y[0.60, 0.80] 重叠**（0.39 × 0.20 m）。区域无碰撞，**不影响仿真物理**，但渲染时两块色块会叠在一起 |
| 2 | Destination "距顶墙 0.8" 无解 | 距左 0.8 + 高 1.4 已占去 1.4，加上距顶 0.8 → 2.2 > 2.0。按你的要求**顶部贴紧 Y=2.0**，代价是**距顶墙实际 0.0、距底墙 0.6** |
| 3 | RACK A 描述 vs 坐标 | 文字描述是"左上偏中"，但你给的坐标 (2.07, 1.58) 在中上偏右。**按坐标执行** |

**如果想消掉第 1 条的重叠**，改 `ZONES` 表一行即可（改完重跑生成脚本）：

| 方案 | 改动 | 结果 |
|---|---|---|
| **保持现状**（当前实现） | — | 接受 0.39 × 0.20 m 的重叠区；仿真无影响 |
| 恢复"距顶墙 0.8" | `destination` 的 `h` 改回 `1.2`、`cy` 改 `1.40` | y 0.8~2.0，距顶/距底都是 0.8，**与 Start 完全不重叠** |
| Start 往下/往左挪 | `start` 的 `cy`/`cx` 调小 | 需你给出新的中心坐标 |

所有实体均**未超出** 3.02 × 2.00 围墙（自检已确认）。

---

## 3. 怎么跑

### 3.1 只看场地（核对布局 / PPT 截图）

```bash
# 目标环境（Harmonic / Jazzy）
gz sim -r sim/worlds/dase7503_field.sdf
ros2 launch sim/launch/field_world.launch.py

# 本机（Ubuntu 22.04 + Fortress）
ign gazebo sim/worlds/dase7503_field_fortress.sdf
ros2 launch sim/launch/field_world.launch.py backend:=fortress

# 无界面（跑回归时用）
ros2 launch sim/launch/field_world.launch.py headless:=true
```

### 3.2 场地 + 麦轮机器人（推荐，M4 日常用）

```bash
# 一次性准备
sudo apt install ros-jazzy-ros-gz
git clone -b jazzy https://github.com/linorobot/linorobot2.git ros2_ws/src/linorobot2
cd ros2_ws && colcon build && source install/setup.bash

# 每次开跑
ros2 launch sim/launch/field_with_robot.launch.py
```

这条命令把**我们的场地世界**喂给 linorobot2 现成的 `gazebo.launch.py`，它会自动：
生成机器人（`mecanum` 模板 URDF）· 起 EKF · 桥接话题 · 发布 TF · 加载 Nav2 需要的全部坐标变换。
出生点默认在 **Start 区中心 (1.51, 0.40)**，车头朝 **+Y**（面向场地内部）。

### 3.3 它桥接的话题——**正好就是我们的 6 话题契约**

| gz ↔ ROS 桥接 | 我们契约里的 |
|---|---|
| `/cmd_vel` | ✅ 同一个 |
| `/odom/unfiltered` | ✅ 我们用 remap 对接的那个名字 |
| `/imu/data` | ✅ D2 新增的那个 |
| `/scan` | ✅ 同一个 |
| `/clock`、`/joint_states`、`/camera/*` | 仿真自带，非契约 |

**也就是说：仿真里跑通 = 真机上话题名一致**，这正是《[导航栈方案对比](../docs/给队友看/导航栈方案对比.md)》D2/D3 想要的效果。

---

## 4. 改坐标 / 重新生成

```bash
python3 sim/scripts/generate_field_world.py            # 重新生成 SDF
python3 sim/scripts/generate_field_world.py --check     # 只做几何自检 + 俯视图
```

脚本结构（改坐标只需动前两张表）：

| 位置 | 内容 |
|---|---|
| `ZONES` | 两个区域的 **中心坐标 + 尺寸**（硬编码） |
| `RACKS` | 四个货架的 **中心坐标**（硬编码） |
| `FIELD_X/Y`、`WALL_*`、`RACK_*` | 场地/围墙/货架的基础尺寸 |

它会：① 打印各对象的位置与四边距离 ② 自检是否出界、货架是否重叠、**区域是否重叠（告警）** ③ 画 ASCII 俯视图 ④ 写出 SDF。
**不要手改 `worlds/*.sdf`**——改脚本后重新生成，否则下次生成就丢了。

---

## 5. 验证状态（如实说明）

| 项 | 结果 |
|---|---|
| XML 合法性 | ✅ 通过 |
| **sdformat 结构校验（Harmonic 版）** | ✅ 通过（降级 SDF 1.7 校验：本机的 `gz sdf` 来自 Classic，最高只认 1.7；元素层级全部合法） |
| **sdformat 结构校验（Fortress 版）** | ✅ `ign sdf -k` → **Valid.**（1.9 是 Fortress 的原生版本） |
| **Fortress 实跑** | ✅ **已验证**：`ign gazebo -r -s` 无界面跑 25 秒，**0 error / 0 warning**，SceneBroadcaster 正常发布 → 世界加载、物理步进、4 个系统插件全部 OK |
| **SDF 内坐标逐项核对** | ✅ `destination_zone` pose `(1.200000, 1.300000)`、`start_zone` `(1.510000, 0.400000)`、`rack_a` `(2.070000, 1.580000)`、`rack_b` `(2.350000, 0.510000)`、`rack_c` `(2.350000, 1.720000)`、`rack_d` `(0.520000, 0.370000)`；区域尺寸盒子 `0.800000 1.400000` / `0.600000 0.800000` |
| 几何自检 | ✅ 6 个实体全在 3.02×2.00 内；货架之间**无重叠**；⚠️ 区域之间有 **1 处重叠**（见 §2，已显式告警） |
| 12 个 `<model>` / 36 个 `<link>` | ✅ 计数与预期一致 |
| **Harmonic 实跑** | ⚠️ **仍未验证**——本机装不了 Harmonic（见 §0）。请在 24.04 + Jazzy 的机器上跑 `gz sim -r sim/worlds/dase7503_field.sdf` |

**Fortress 版已证明"几何 + 插件 + 物理"这套东西是对的**（两个变体除了版本号和插件名，内容逐字相同），所以 Harmonic 版的风险已经很小——只剩下"插件名/版本号对不对"这一点，而那正是两个变体唯一的差别。

World 里已显式声明 4 个必需的系统插件（`Physics` / `UserCommands` / `SceneBroadcaster` / `Sensors`）——缺任何一个都会表现为"没物理 / 生成不了模型 / 看不到画面 / 雷达无数据"。

---

## 6. 待确认 / 可调

| # | 事项 | 说明 |
|---|---|---|
| 1 | **区域重叠**（§2 第 1 条） | 当前实现是重叠的；要不要按 §2 的方案改，等你确认 |
| 2 | Destination 距顶墙 0.0 | 为保住 1.4 高度而贴顶；若图纸本意是"距顶墙 0.8"，则高度只能是 1.2 |
| 3 | RACK A 位置 (2.07, 1.58) | 按你给的坐标；与"左上偏中"的文字描述不一致 |
| 4 | 货架高 0.2325 / 层数 3 | 按图纸文字取值，与实物核对 |
| 5 | 场地底面材质/灯光 | 图纸未规定，当前为中性灰 + 平行光 + 补光 |
| 6 | 货架是否要贴二维码纹理 | 暂未加（M3 不做仿真，见《[仿真与离线测试方案](../docs/给队友看/仿真与离线测试方案.md)》）；若要在仿真里做集成冒烟测试再加 |
| 7 | `spawn_z` | 默认 0.0；机器人陷地/悬空时按 ±0.02 调 |

---

## 7. 参考

- linorobot2（Gazebo 栈来源）：https://github.com/linorobot/linorobot2 （`jazzy` / `humble` 分支）
- 关联：《[仿真与离线测试方案](../docs/给队友看/仿真与离线测试方案.md)》《[导航栈方案对比](../docs/给队友看/导航栈方案对比.md)》
