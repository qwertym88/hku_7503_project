# robot_description —— URDF / 机器人模型（M1 + M5）

## 职责
整车的 URDF/xacro 描述、TF 树、可视化模型。供 RViz 显示和导航 footprint 使用。

## 关键
- `base_link → laser`、`base_link → camera` 的外参**按实测填写**，不要写 0。
- 导航 costmap 的 footprint 要包含**载货包络**（货架尺寸 + 5cm 余量）。
- 与 `cad/` 的 CAD 保持一致；CAD 改尺寸后同步更新 URDF。

当前为空壳，待 M1 出底盘 CAD 后建模。
