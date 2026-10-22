# robot_navigation —— 雷达 / 定位 / 导航（M4）

## 职责
```text
MS200 → /scan → 建图(slam_toolbox) → 地图 → 定位(AMCL) → Nav2 → /cmd_vel
```

## 接口
- 收 `/scan`，发 `/cmd_vel`。
- 提供"给定目标坐标，自动到达并停稳"的能力，供 `mission_control` 调用。

## 待办
- [ ] MS200 驱动 → `/scan`
- [ ] **引入 linorobot2 栈**（`description` / `base`(EKF) / `navigation` / `gazebo`），见《[导航栈方案对比](../../../docs/给队友看/导航栈方案对比.md)》
- [ ] 建图 + 保存到 `maps/`（复用其 `slam.yaml`）
- [ ] EKF 融合 `odom/unfiltered` + `/imu/data`，再 AMCL 定位（定位跳变 < 3cm）
- [ ] Nav2 配置（复用其 `navigation.yaml`，膨胀半径调保守防撞墙）
- [ ] 停靠点标定（货架前 / 目的区）
- [ ] 兜底方案：里程计 + 雷达避障的简易航点驱动（~100 行）

当前为空壳。**参数以实测为准**，不要照抄示例。
注意：linorobot2 的 Nav2 用**差速**控制器（RPP + 差分 AMCL），不输出横向速度——与我们的保底路线一致。
