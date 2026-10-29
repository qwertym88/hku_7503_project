# robot_description —— URDF / 机器人模型（M1 + M5）

## 职责
整车的 URDF/xacro 描述、TF 树、可视化模型。供 RViz 显示和导航 footprint 使用。

## 关键
- `base_link → laser`、`base_link → camera` 的外参**按实测填写**，不要写 0。
- 导航 costmap 的 footprint 要包含**载货包络**（货架尺寸 + 5cm 余量）。
- 与 `cad/` 的 CAD 保持一致；CAD 改尺寸后同步更新 URDF。

## 已导入的 SolidWorks 模型

2026-10-29 从 `~/Desktop/solidworks` 导入了底盘和四个轮子的 STL/URDF：

- 模型：`urdf/dase7503_robot.urdf`
- 网格：`meshes/base_link.STL`、`meshes/wheel_{LB,LF,RF,RB}.STL`
- 原始导出留档：`cad/exports/solidworks-20261029/`
- 查看模型：`ros2 launch robot_description display.launch.py`
- 无桌面运行：`ros2 launch robot_description display.launch.py use_gui:=false`（RViz 仍需要图形显示）

导出 URDF 已做 ROS 2 接入修正：包 URI 改为 `robot_description`，四个重复的
`rev` 关节改成唯一名称。四个轮子目前使用 `continuous` 关节，供仿真和关节状态发布
使用；轮子轴向按各 STL 局部坐标设置，仍需在 Gazebo 中用驱动插件验证正负方向。
雷达、相机、举升机构外参和 Gazebo 驱动/传感器插件必须由 M1/M3/M4 实测后补齐。

## 待验收问题

- 用卡尺确认 CAD 导出的单位、整车包络和轮心坐标；当前 URDF 的惯量/质量是导出值，不能直接作为整车动力学参数。
- 确认 `base_link` 原点与 `base_footprint`、雷达、相机的实测 TF。
- 将真实轮子 joint、`ros2_control`/Gazebo Harmonic 插件与 M2 的编码器命名对齐。
