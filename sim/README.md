# sim —— Gazebo 仿真场地

2026-11-20：以用户手工确认的 `worlds/dase7503_field.sdf` 货架布局为基准，将原点移至原图纸右下角（旧坐标 `3.02, 0`）。场地、区域、货架和二维码的相对位置与物理朝向保持一致，并同步机器人出生位姿及两种世界文件。仿真结果与[实车验收](../tests/保守方案验收.md)分别记录。

## 当前坐标系

单位 m，+Z 朝上。新 **+X = 旧 +Y**，新 **+Y = 旧 −X**，保持右手系。

```text
x_new   = y_old
y_new   = 3.02 - x_old
z_new   = z_old
yaw_new = yaw_old - π/2      # 按需归一化到 [-π, π]

x_old   = 3.02 - y_new
y_old   = x_new
```

场地范围为 **X=0～2.00，Y=0～3.02**。下图按新 X 向右、新 Y 向上显示；Start 位于原点右上方，靠近低 Y 边界，Destination 靠近高 Y 边界。

```text
                     +Y（旧 -X）
                      ↑
               3.02   ┌─────────────────────────┐
                      │      Destination        │
                      │                         │
                      │     A             C     │
                      │                         │
                      │    D          B         │
                      │                         │
                      │         Start           │
                      └─────────────────────────┘ → +X（旧 +Y）
                    (0,0)                     2.00
                       原图纸右下角
```

图仅表示布局，精确位置如下。货架新坐标来自本次用户确认的 SDF，已经同步回生成脚本，不使用先前图纸初值。

| 对象 | 本次确认的旧中心 `(x,y)` | 当前新中心 `(x,y)` | 新坐标尺寸/边界 |
|---|---|---|---|
| Destination（绿） | `(0.40, 1.00)` | `(1.00, 2.62)` | `1.40 × 0.80 m`；X `0.30～1.70`，Y `2.22～3.02` |
| Start（蓝） | `(2.72, 1.00)` | `(1.00, 0.30)` | `0.80 × 0.60 m`；X `0.60～1.40`，Y `0.00～0.60` |
| Rack A | `(1.125, 0.550)` | `(0.550, 1.895)` | X/Y 外廓 `0.40 × 0.15 m`，侧墙高 `0.23 m` |
| Rack B | `(1.825, 1.360)` | `(1.360, 1.195)` | 同上 |
| Rack C | `(1.325, 1.670)` | `(1.670, 1.695)` | 同上 |
| Rack D | `(1.925, 0.410)` | `(0.410, 1.095)` | 同上 |

## 货架与二维码

每架仅两片侧墙有碰撞，位于中心 X 的 `±0.1875 m` 处。新坐标下每墙 **X 向厚 2.5 cm、Y 向长 15 cm、Z 向高 23 cm**，内侧沿 X 净距 **35 cm**，通道沿 **新 ±Y（旧 ±X）** 贯通。没有底板或层板；模型为静态环境障碍，不含举升搬运动力学。围墙高 0.50 m，位于场地边界外侧；地板和区域标记仅视觉。

二维码处于对应 Rack 的 XY 中心，中心距地 **25 cm**、边长 **5 cm**，竖直悬空。仿真网格两面可读。固定路线从 −Y 侧扫 A、B、C 架，从 +Y 侧扫 D 架。

原图 `~/Desktop/QR/A.png`～`D.png` 已原样复制到仓库：运行时不依赖 Desktop 路径。对应内容为 `RACKA_4X6M`、`RACKB_AM9L`、`RACKC_K8Y3`、`RACKD_R983`。

## 起终点圆柱与二维码

S、E 使用当前场地坐标，均为固定圆柱，**直径 6 cm、高 20 cm**，包含圆柱碰撞体。

| 对象 / 模型名 | 中心 `(x,y)`，m | 二维码贴图 | 编码内容 |
|---|---|---|---|
| S / `marker_s` | `(0.60, 0.60)` | `materials/textures/Start.png` | `START` |
| E / `marker_e` | `(0.30, 2.22)` | `materials/textures/End.png` | `END` |

原图 `~/Desktop/QR/Start.png`、`End.png` 已原样复制到仓库。仿真中两张二维码边长 5 cm，图案中心距地 24.5 cm，并带白色留边，使相机能从柱沿上方拍全图案。码牌仅用于渲染。

两张二维码的正面法向均为 **`(1, -1, 0) / √2`**，朝新 +X、−Y 方向，与 +X 的夹角为 −45°。由于原始网格正面朝 −Y，生成脚本中的 `qr_yaw=math.pi / 4` 对应这一朝向。

任务从出生点转向 S 柱读取 `START`。搬运完成后，车到 E 柱东南侧的 `(0.60, 1.92)` m 读取 `END`。每次扫码的原始 PNG 与码值随运行记录保存。

## 修改布局与重新生成

```text
sim/
├─ scripts/generate_field_world.py    # 新坐标 RACKS / MARKERS、区域 ZONES、几何参数
├─ materials/textures/{A,B,C,D,Start,End}.png # 用户原始二维码贴图
├─ meshes/qr_panel.dae               # 双面二维码网格
├─ worlds/dase7503_field.sdf          # Harmonic 世界（自动生成）
├─ worlds/dase7503_field_fortress.sdf # 遗留文件，不随当前场景更新
├─ launch/field_world.launch.py       # 仅场地
└─ launch/field_with_robot.launch.py  # 场地 + linorobot2 机器人
```

调整货架位置时修改 `scripts/generate_field_world.py` 的 `RACKS` 表中 `cx`、`cy`，数值直接使用**新坐标**。尺寸修改 `RACK_GAP`、`RACK_WALL_T`、`RACK_WALL_L`、`RACK_H`；二维码高度/边长修改 `QR_CENTER_Z`、`QR_SIZE`。

圆柱位置修改 `MARKERS` 表中 `cx`、`cy`；直径、高度修改 `MARKER_DIAMETER`、`MARKER_H`，二维码边长/中心高度修改 `MARKER_QR_SIZE`、`MARKER_QR_CENTER_Z`。实体重叠检查对圆柱使用外接正方形，ASCII 图中大写 `S/E` 表示圆柱，小写 `s` 表示 Start 区域。

```bash
# 从仓库根目录检查实体出界、侧墙/圆柱包围盒重叠、资源和 ASCII 俯视图
python3 sim/scripts/generate_field_world.py --check

# 重新生成当前 Harmonic 场景
python3 sim/scripts/generate_field_world.py

# SDF 格式校验
gz sdf -k sim/worlds/dase7503_field.sdf
```

直接改 SDF 后重新生成会覆盖手工改动，需先同步回 `RACKS` / `MARKERS` / `ZONES` 等对应参数。资源路径相对世界文件解析，复制场景时应包含整个 `sim/` 目录。

## 运行

```bash
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
gz sim -r sim/worlds/dase7503_field.sdf

# 或通过 ROS 仅启动场地
ros2 launch sim/launch/field_world.launch.py

# 已安装编译 linorobot2 时生成机器人
ros2 launch sim/launch/field_with_robot.launch.py
```

### M3/M4/M5 联调场景（理想底盘）

2026-11-21：新增自动ABCD拍照/举升/运输/放置入口：`ros2 launch sim_control fixed_route_sim.launch.py`。它从统一YAML生成本轮场景和固定路线，关闭手动面板，等待人工开始；完整命令、标定字段与限制见[固定路线仿真使用说明](../docs/给队友看/固定路线仿真使用说明.md)。以下手动入口仍保留。

该联调入口在 Gazebo 场地中加载无轮胎动力学的理想运动学底盘，接收 `/cmd_vel` 后直接积分并更新模型位姿。车体网格及安装位置来自 SolidWorks 导出的 `robotcar.urdf`，雷达和相机 link 离地分别约 `0.101129 m`、`0.143329 m`，相机沿车头 +X 水平观察。它把 `/scan`、`/camera/image_raw` 和 `/camera/camera_info` 桥接到 ROS，并打开带扫描/图像显示的 RViz 和按住移动的控制面板。此模式不模拟轮胎接触、碰撞避让或真实电机误差。

```bash
cd ros2_ws
colcon build --packages-up-to sim_control --symlink-install
source install/setup.bash
ros2 launch sim_control sim_ideal.launch.py
```

面板可用方向按钮或 W/A/S/D 控制，松开即发布零速；`gui:=false`、`rviz:=false`、`panel:=false` 可分别关闭窗口。详细话题、坐标与安全超时见 [`sim_control/README.md`](../ros2_ws/src/sim_control/README.md)。

机器人默认出生于新 Start 中心 **`(1.00, 0.30)`**，`yaw=π/2`，车头朝新 +Y，保持原先朝场地内部的物理方向。旧坐标的地图、航点、AMCL 初始位姿或外部节点输入若要用于此仿真，须先按上面的变换转换；本次只修改 `sim/`，不将真实场地尚未标定的坐标同步为仿真坐标。

项目运行目标为 Jazzy + Harmonic。Fortress 是遗留兼容文件，本次 S/E 标志未同步到该文件；后续计划移除。

### 几何地图、AMCL 与 RViz

先从仓库根目录构建 ROS 工作区。定位使用 Jazzy 的 `nav2_bringup`，独立扫图使用 `slam_toolbox`。

```zsh
cd ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --packages-up-to sim_control
source install/setup.zsh
```

启动任务时，程序根据 `config` 中的场地边界和 S/E 圆柱生成 1 cm 分辨率的 `field_map.yaml` 和 `field_map.pgm`。Gazebo 使用同一份场地配置。可移动货架由任务维护，定位地图只包含固定结构。`map_server` 发布静态 `/map`，AMCL 根据雷达和 `/odom` 发布 `map→odom`。任务等待定位就绪，直行与转向仍用仿真 `/odom` 闭环。

```zsh
export ROS_DOMAIN_ID=77
GZ_PARTITION=dase7503_mission ros2 launch sim_control fixed_route_sim.launch.py \
  plan_route:=true autostart:=true
```

终端会打印定位地图路径。省略 `autostart` 时，在同一 ROS 域的另一个终端执行 `ros2 service call /mission/start std_srvs/srv/Trigger '{}'`。RViz 的 `Saved field map` 显示定位地图，`AMCL pose` 显示定位，深紫色为整轮路线，青绿色为当前动作尚待执行的路段。`Field and stationary racks` 是可选的几何显示层，随任务更新货架位置。

### 独立自主扫图

`field_mapping.launch.py` 在空场里沿预检过的短回路行驶，回到起点停稳后保存 2 cm 分辨率的雷达栅格，随后自动退出。默认 ROS 域为 78。

```zsh
map_dir=$(mktemp -d /tmp/dase7503-map-XXXXXX)
GZ_PARTITION=dase7503_mapping ros2 launch sim_control field_mapping.launch.py \
  output:="$map_dir/field_lidar"
```

`output` 是文件名前缀。省略时自动在 `/tmp` 生成新目录，终端会打印保存路径。需要对比扫图结果时，在任务启动命令中追加 `map:="$map_dir/field_lidar.yaml"`。地图 YAML 和 PGM 应一起保存，并与任务使用同一场地坐标系。也可用 `field_slam.launch.py` 手动采图，再用 `ros2 run nav2_map_server map_saver_cli -f "$map_dir/field_lidar"` 在同一 ROS 域导出。

### 10 cm 连续曲线规划

任务默认走原固定路线。在任务启动命令上追加 `plan_route:=true clearance:=0.10`，即可在启动 Gazebo 前按当前场地规划连续曲线。终端会给出 `planned_route.yaml`、`route.svg` 和 `field_map.yaml` 的位置。也可选择已生成的[连续曲线路线](routes/offline_10cm_curves.yaml)，或从仓库根目录离线生成并预览：

```bash
PYTHONPATH=ros2_ws/src/mission_control python3 -m mission_control.offline_plan \
  ros2_ws/src/mission_control/config/fixed_route_sim.yaml \
  --clearance 0.10 --output sim/routes/offline_10cm_curves.yaml --map /tmp/field_geometry.yaml
PYTHONPATH=ros2_ws/src/mission_control python3 -m mission_control.validate \
  sim/routes/offline_10cm_curves.yaml --svg /tmp/offline_curves.svg
```

规划器先用 A* 找可行通道，再为去拍照位和去放架位的路段搜索相切的直线和圆弧，找不到合格曲线时沿原折线路段执行。车头方向在接点连续，曲率可以变化，控制器会在急弯处降速。拍照和放架需要到位停稳，窄处允许在端点原地对正，进架、离架和退出沿货架轴线直行。RViz 和 `route.svg` 显示实际下发给控制器的曲线。

每次放货后更新障碍，按扫掠外廓检查车体和载荷到 S/E 柱及其他货架的 10 cm 净空，并在规划时预留 1.2 cm 跟踪余量。当前操作货架在对接时按实际通道校验，35 cm 通道配 25 cm 车体，居中每侧为 5 cm。外墙沿用场地边界净空校验。

当前布局的八段货架转运和最后驶向 E 柱的路段均找到曲线。可用上述离线命令的 `--polyline` 参数生成折线路线作为对照。

迁移实车时，可沿用静态地图加 AMCL 的结构。地图可来自场地实测尺寸或独立雷达采图，再对齐现场墙体、圆柱和任务航点。用实测雷达及编码器/IMU 融合里程计接入定位。货架位置变动时更新路线使用的障碍布局，固定结构变动时更新定位地图。

## 验证记录

2026-11-20 起终点标志：

- 两张原始贴图与 Desktop 文件一致，分别可解码为 `START`、`END`。SDF 中的位置、柱体尺寸、二维码尺寸和朝向均已核对；`gz sdf -k`、脚本生成一致性及实体边界/重叠检查通过。Fortress 文件未同步此变更。

2026-11-20 坐标转换：

- 与转换前用户确认的 Harmonic SDF 比较了 **31 个 visual/collision 几何**的世界顶点，均符合上述刚体变换；最大数值差约 `4.9e-10 m`。确认四架中心来自用户手工布局，几何、二维码带纹理网格和法向、地面及灯光共同转换。
- Harmonic 世界与生成脚本输出一致；出界、侧墙重叠、资源存在性和 `gz sdf -k` 检查通过。机器人出生参数符合新坐标。
- 本次未重新运行相机解码或动态穿架测试。

历史记录：最初的 Y 向通道/14 cm 二维码版本曾通过 Gazebo Harmonic 8.15.0 加载、150 步及相机解码；之后二维码缩为 5 cm，这些历史结果不代表当前版本识别率。原 PNG 与 Desktop 源文件曾核对 SHA256 一致。实车定位、动态穿架和搬运仍需单独验收。
