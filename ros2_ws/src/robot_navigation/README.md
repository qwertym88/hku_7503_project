# robot_navigation —— 雷达 / 定位 / 导航（M4）

## 职责
```text
MS200 → /scan → slam_toolbox(建图) → 地图 → EKF+AMCL(定位) → Nav2 → /cmd_vel
```

## 状态（2026-11-09，本机 Ubuntu 24.04 + Jazzy + 真雷达实测）

| 任务 | 状态 | 说明 |
|---|---|---|
| M4-1 雷达 → /scan | ✅ **实测通过** | Oradar 官方 ROS2 驱动，10.0 Hz，frame_id=laser |
| M4-2 建图（slam_toolbox） | ✅ 链路通 | 桌面实测出图；**场地正式地图待进场地建** |
| M4-3 定位（EKF+AMCL） | ✅ 链路通 | EKF 已跑通（mock 里程计）；真 odom+IMU 等 M2 固件 |
| M4-4 地图保存/载入 | ✅ 实测通过 | `map_saver_cli` 保存 + Nav2 载入跑通 |
| M4-5 Nav2 配置 | ✅ 实测通过 | 全栈激活，发目标 → /cmd_vel 出 Twist |
| M4-6 已知点导航 | 🟡 链路通 | mock 底盘下发目标 SUCCEEDED；真车 ±5cm 待底盘 |
| M4-7 停靠点 | 🟡 初值就绪 | 按图纸几何推算，装车后实测微调 |
| M4-10 兜底航点驱动 | ✅ 逻辑实测 | 转向/雷达急停逻辑验证；带车跑通待底盘 |
| M4-11/12 仿真 | ⏸️ 暂停 | 按决策转真机优先；场地世界已验证（见 HANDOFF） |

## 快速开始（真机，雷达插 USB 后）

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
export ROS_DOMAIN_ID=42

# ① 只测雷达（M4-1 验收：~10Hz、frame_id=laser）
ros2 launch robot_navigation lidar.launch.py
ros2 topic hz /scan

# ② 建图（M2 固件未就绪时加 mock_base:=true；就绪后去掉）
ros2 launch robot_navigation slam.launch.py mock_base:=true rviz:=true
# 推着车（或手提雷达）把场地走一遍 → 保存：
ros2 run nav2_map_server map_saver_cli -f maps/dase7503_field

# ③ 导航（M2 固件未就绪时加 mock_base:=true）
ros2 launch robot_navigation navigation.launch.py map:=maps/dase7503_field.yaml rviz:=true
# RViz 顶部 2D Goal Pose 点目标，或：
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.20, y: 1.33}}}"

# ④ 兜底航点驱动（Nav2 调不动时；不要与 ③ 同时跑）
ros2 run robot_navigation waypoint_driver                       # 默认 rack_a → destination
ros2 run robot_navigation waypoint_driver --ros-args -p route:="[rack_b,destination]"
```

## 文件地图

```text
config/
├─ ekf.yaml          EKF：odom/unfiltered + imu/data → odom TF（复用 linorobot2 结构）
├─ slam.yaml         slam_toolbox 建图参数（小场地调参版）
├─ navigation.yaml   Nav2 全栈（半径0.15/膨胀0.20/±5cm 精停/Twist 非Stamped/无横移）
└─ dock_points.yaml  停靠点（start/destination/rack_a-d，按 HANDOFF §4 几何推算）
launch/
├─ lidar.launch.py       雷达驱动 + robot_state_publisher（TF 链）
├─ slam.launch.py        建图全链路（雷达+EKF+slam_toolbox，可选 mock/rviz）
└─ navigation.launch.py  导航全链路（雷达+EKF+Nav2+AMCL，可选 mock/rviz）
robot_navigation/
├─ mock_odom.py        T0 mock 底盘：M2 就绪前发零速 odom/unfiltered（D9）
└─ waypoint_driver.py M4-10 兜底：里程计定位 + 固定航点 + 雷达急停（~130 行）
rviz/nav.rviz          RViz 配置（TF/雷达/地图/路径/AMCL 粒子）
```

## 关键决策与依据

- **雷达驱动**：MS200 = Orbbec Oradar MS200（230400 波特、47 字节定长包、12 点/包、CRC8）。
  用官方 [oradar_ros2_driver](https://github.com/Alliance-Algorithm/oradar_ros2_driver)（MIT），
  已 vendor 到 `ros2_ws/src/oradar_lidar/`。
- **/odom 对接**：M2 固件发 `/odom`，EKF 输入名是 `odom/unfiltered` —— **用 remap，不改固件**（决策 D3）。
  M2 就绪前用 `mock_odom` 顶替（同话题名，切换零改动）。
- **AMCL 初始位姿**：`navigation.yaml` 里 `set_initial_pose` 写死 Start 区 (2.72, 1.00, 朝-X)，
  比赛摆车即用；换测试地图时用 RViz "2D Pose Estimate" 或 `/initialpose` 覆盖。
- **安全层**：collision_monitor（雷达近距降速/刹车）+ 兜底驱动自带急停，双保险。
- ⚠️ **不变量**：`enable_stamped_cmd_vel: false`（否则 Jazzy Nav2 发 TwistStamped，车不动不报错）；
  `velocity_smoother` 的 y 速度钳 0（保底无横移）。

## 待办（按优先级）

- [ ] **进场地建正式地图** → `maps/dase7503_field.yaml`（现在只有桌面测试图 `test_desk`）
- [ ] M2 固件就绪后：去掉 `mock_base`，实测 EKF 融合真 odom+IMU（定位跳变 < 3cm）
- [ ] 真车已知点导航 ±5cm 验收（M4-6）
- [ ] 停靠点实测微调（相机扫到 QR + 托叉伸入货架，M4-7）
- [ ] 载货状态参数（M4-8）+ 参数冻结（M4-9）
- [ ] 雷达/相机安装外参实测后更新 `robot_description/urdf/dase7503_nav.urdf`（有 TODO 标记）

> 仿真路线（linorobot2 Gazebo）已按 11-09 决策暂停：场地世界 `sim/` 已验证可用（HANDOFF #4 关闭），
> 但本机 Gazebo 渲染与沙箱冲突；真机链路已全通，仿真不再投入。
