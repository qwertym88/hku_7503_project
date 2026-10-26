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

**坐标约定**（与图纸一致）：原点在**场地左下角**，+X 向右，+Y 向上，+Z 天向。
场地是**竖向（纵向）矩形**：**X（宽）= 2.00 m，Y（长）= 3.02 m**，即 x ∈ [0, 2.00]、y ∈ [0, 3.02]。
围墙高 **0.5 m**，贴在场地**外侧**，内表面正好压在 x=0/2.00、y=0/3.02 上（不占场地面积）。
走向：**Destination（绿）在上 → 4 个货架在中间 2×2 → Start（蓝）在下**。

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

## 1. 竖向布局与坐标

`python3 sim/scripts/generate_field_world.py --check` 会打印下面这张按比例的俯视图（此处是它的输出，**上方=场地顶部 y=3.02，下方=场地底部 y=0**）：

```text
+---------------------------------------------+
|             GGGGGGGGGGGGGGGGGGG             |  ← Destination 绿（顶部居中）
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|         AAAAAGGGGGGGGGGGGGGGGGCCCCC         |  ← A 左上 / C 右上（y=2.0）
|         AAAAAGGGGGGGGGGGGGGGGGCCCCC         |
|         AAAAAGGGGGGGGGGGGGGGGGCCCCC         |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|             GGGGGGGGGGGGGGGGGGG             |
|                                             |
|                                             |  ← 中间通道净空 0.82 m
|                                             |
|                                             |
|                                             |
|         DDDDD                 BBBBB         |  ← D 左下 / B 右下（y=1.0）
|         DDDDD                 BBBBB         |
|         DDDDD                 BBBBB         |
|               SSSSSSSSSSSSSSS               |  ← Start 蓝（底部居中，贴底边）
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
|               SSSSSSSSSSSSSSS               |
+---------------------------------------------+
  上方 = 场地顶部(y=3.02)  下方 = 场地底部(y=0)  左 = x=0
  G=Destination绿(上)  S=Start蓝(下)  A左上 C右上 D左下 B右下
  场地 2.00(X/宽) x 3.02(Y/长) m，竖向
```

| 对象 | x 范围 | y 范围 | 中心 | 平面尺寸 | 距左 | 距右 | 距底 | 距顶 |
|---|---|---|---|---|---|---|---|---|
| **Destination 区**（绿） | 0.600 ~ 1.400 | 1.620 ~ 3.020 | **(1.000, 2.320)** | 0.80 × 1.40 | 0.600 | 0.600 | 1.620 | **0.000** |
| **Start 区**（蓝） | 0.700 ~ 1.300 | 0.000 ~ 0.800 | **(1.000, 0.400)** | 0.60 × 0.80 | 0.700 | 0.700 | **0.000** | 2.220 |
| **RACK A**（左上） | 0.430 ~ 0.570 | 1.930 ~ 2.070 | **(0.500, 2.000)** | 0.14 × 0.14 | 0.430 | 1.430 | 1.930 | 0.950 |
| **RACK C**（右上） | 1.430 ~ 1.570 | 1.930 ~ 2.070 | **(1.500, 2.000)** | 0.14 × 0.14 | 1.430 | 0.430 | 1.930 | 0.950 |
| **RACK D**（左下） | 0.430 ~ 0.570 | 0.930 ~ 1.070 | **(0.500, 1.000)** | 0.14 × 0.14 | 0.430 | 1.430 | 0.930 | 1.950 |
| **RACK B**（右下） | 1.430 ~ 1.570 | 0.930 ~ 1.070 | **(1.500, 1.000)** | 0.14 × 0.14 | 1.430 | 0.430 | 0.930 | 1.950 |

> 四个货架在场地**中间排成 2×2 矩形**：上排 A/C 在 y=2.0，下排 D/B 在 y=1.0，左右两列在 x=0.5 / x=1.5。
> 俯视图里 A、C 看起来**贴着** G，那是 ASCII 栅格化的取舍：实际 **A 右缘 0.570 与 G 左缘 0.600 还差 0.03 m**，自检确认**无重叠**。

货架高 **0.2325 m**，建模为 4 立柱 + 3 层板（`<collision>` + `<visual>` 齐全）。
**区域标记无碰撞**（只有 `<visual>`）——机器人要能开过去。

---

## 2. 无重叠校验（本次重点）

脚本的自检会**显式检查重叠**，当前结果全部通过：

```text
[OK] 货架之间无重叠；全部实体在 2.00 x 3.02 边界内
[OK] 区域之间、区域与货架之间【均无重叠】
[i]  Destination 底边 y=1.620，Start 顶边 y=0.800，中间纵向通道净空 0.820 m
```

关键数字：

| 关系 | 净空 |
|---|---|
| Destination 底 (y=1.620) ↔ Start 顶 (y=0.800) | **0.820 m 通道** |
| RACK A 右缘 (x=0.570) ↔ Destination 左缘 (x=0.600) | 0.030 m |
| RACK C 左缘 (x=1.430) ↔ Destination 右缘 (x=1.400) | 0.030 m |
| RACK D/B 顶 (y=1.070) ↔ Start 顶 (y=0.800) | 0.130 m |
| 上排货架 (y=2.070) ↔ 下排货架 (y=1.930) | 0.860 m |

> ⚠️ **方向说明**：本版把场地改成了**竖向**（X=2.00 宽，Y=3.02 长），货架也从"都在右侧"改成"中间 2×2"。
> 这**取代**了最早那版文字描述（"Destination 左上、Start 贴右墙、四个货架沿右侧一列"）。如果那份描述还有效，请说一声——两版不能同时成立。
> 我这边**看不了图纸图像**，只能按你给的坐标执行；坐标是唯一可靠输入。

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
出生点默认在 **Start 区中心 (1.00, 0.40)**，车头朝 **+Y**（从底部朝场地内部，正对货架区）。

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

脚本结构（改坐标只需动前几张表；围墙、地板、光照的位置都**由 `FIELD_X/FIELD_Y` 自动推导**，换横竖不会再漏改）：

| 位置 | 内容 |
|---|---|
| `FIELD_X` / `FIELD_Y` | 场地宽（2.00）/ 长（3.02）—— **改横竖只动这两个数** |
| `ZONES` | 两个区域的 **中心坐标 + 尺寸**（硬编码） |
| `RACKS` | 四个货架的 **中心坐标**（硬编码） |
| `WALL_*` / `RACK_*` | 围墙、货架的基础尺寸 |

它会：① 打印各对象位置与四边距离 ② 自检出界 / 货架重叠 / **区域重叠（告警）** / 纵向通道净空 ③ 画 ASCII 俯视图 ④ 写出 SDF。
**不要手改 `worlds/*.sdf`**——改脚本后重新生成，否则下次生成就丢了。

---

## 5. 验证状态（如实说明）

| 项 | 结果 |
|---|---|
| XML 合法性 | ✅ 通过 |
| **sdformat 结构校验（Harmonic 版）** | ✅ 通过（降级 SDF 1.7 校验：本机的 `gz sdf` 来自 Classic，最高只认 1.7；元素层级全部合法） |
| **sdformat 结构校验（Fortress 版）** | ✅ `ign sdf -k` → **Valid.**（1.9 是 Fortress 的原生版本） |
| **Fortress 实跑** | ✅ **已验证**：`ign gazebo -r -s` 无界面跑 25 秒，**0 error / 0 warning** → 世界加载、物理步进、4 个系统插件全部 OK |
| **SDF 内坐标逐项核对** | ✅ `destination_zone` `(1.000000, 2.320000)`、`start_zone` `(1.000000, 0.400000)`、`rack_a` `(0.500000, 2.000000)`、`rack_b` `(1.500000, 1.000000)`、`rack_c` `(1.500000, 2.000000)`、`rack_d` `(0.500000, 1.000000)` |
| **尺寸核对** | ✅ 场地地板 `2.000000 3.020000`；Destination 盒子 `0.800000 1.400000`；Start 盒子 `0.600000 0.800000`；南北墙 `2.100000 0.050000 0.500000`（内表面 y=0 / y=3.02）；东西墙 `0.050000 3.020000 0.500000`（内表面 x=0 / x=2.00） |
| 几何自检 | ✅ 6 个实体全在 2.00×3.02 内；**区域与货架之间、货架之间全部无重叠**；纵向通道 0.820 m |
| 12 个 `<model>` / 36 个 `<link>` | ✅ 计数与预期一致 |
| **Harmonic 实跑** | ⚠️ **仍未验证**——本机装不了 Harmonic（见 §0）。请在 24.04 + Jazzy 的机器上跑 `gz sim -r sim/worlds/dase7503_field.sdf` |

**Fortress 版已证明"几何 + 插件 + 物理"这套东西是对的**（两个变体除了版本号和插件名，内容逐字相同），所以 Harmonic 版的风险已经很小——只剩下"插件名/版本号对不对"这一点，而那正是两个变体唯一的差别。

World 里已显式声明 4 个必需的系统插件（`Physics` / `UserCommands` / `SceneBroadcaster` / `Sensors`）——缺任何一个都会表现为"没物理 / 生成不了模型 / 看不到画面 / 雷达无数据"。

---

## 6. 待确认 / 可调

| # | 事项 | 说明 |
|---|---|---|
| 1 | 布局方向（横/竖） | 本版按"竖向"做；与最早那版文字描述冲突，需要你确认以哪个为准 |
| 2 | 货架摆位 | 本版按坐标做成了中间 2×2；若图纸还有别的摆法，给坐标最快 |
| 3 | 货架高 0.2325 / 层数 3 | 按图纸文字取值，与实物核对 |
| 4 | 场地底面材质 / 灯光 | 图纸未规定，当前为中性灰 + 平行光 + 补光；位置已随场地中心自动调整 |
| 5 | 货架是否要贴二维码纹理 | 暂未加（M3 不做仿真，见《[仿真与离线测试方案](../docs/给队友看/仿真与离线测试方案.md)》）；若要在仿真里做集成冒烟测试再加 |
| 6 | 二维码尺寸 / 贴放高度 | 图纸未给，如需可在货架面板上加带纹理的 `<visual>` |
| 7 | `spawn_z` | 默认 0.0；机器人陷地/悬空时按 ±0.02 调 |

---

## 7. 参考

- linorobot2（Gazebo 栈来源）：https://github.com/linorobot/linorobot2 （`jazzy` / `humble` 分支）
- 关联：《[仿真与离线测试方案](../docs/给队友看/仿真与离线测试方案.md)》《[导航栈方案对比](../docs/给队友看/导航栈方案对比.md)》
