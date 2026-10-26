# sim —— Gazebo 仿真场地

场地已按 [1.png](../1.png) 的横向俯视图重建。**原点在左下角**，`+X` 向右、`+Y` 向上、`+Z` 向上；单位均为米。围墙高 0.50 m，位于场地外侧，内表面就是边界线。

```text
sim/
├─ scripts/generate_field_world.py    # 唯一事实来源：坐标和尺寸改这里
├─ worlds/dase7503_field.sdf          # Gazebo Harmonic（自动生成）
├─ worlds/dase7503_field_fortress.sdf # Gazebo Fortress（自动生成）
└─ launch/field_with_robot.launch.py  # 场地 + linorobot2 麦轮机器人
```

## 图纸布局

场地为 **3.02 × 2.00 m** 横向矩形（`x ∈ [0, 3.02]`、`y ∈ [0, 2.00]`）。与先前错误的竖向 2.00 × 3.02 m 布局不同，Destination 在左侧、Start 在右侧。

| 对象 | 中心 `(x, y)` | 尺寸 / 边界 |
|---|---:|---|
| Destination（绿） | `(0.40, 1.00)` | `x=0.00~0.80`，`y=0.30~1.70`，`0.80 × 1.40 m` |
| Start（蓝） | `(2.72, 1.00)` | `x=2.42~3.02`，`y=0.60~1.40`，`0.60 × 0.80 m` |
| Rack A | `(1.20, 1.58)` | `0.14 × 0.14 m` 平面，`0.2325 m` 高 |
| Rack B | `(1.85, 0.51)` | 同上 |
| Rack C | `(1.85, 1.72)` | 同上 |
| Rack D | `(1.35, 0.37)` | 同上 |

区域标记仅为视觉元素，没有碰撞；货架由四根立柱和三层板组成，带碰撞。`--check` 会检查全部对象是否出界、货架是否重叠以及区域是否压到货架。

## 生成和检查

```bash
# 只检查坐标、边界、重叠，并输出 ASCII 俯视图
python3 sim/scripts/generate_field_world.py --check

# 生成 Harmonic 世界（默认）
python3 sim/scripts/generate_field_world.py

# 生成 Fortress 世界
python3 sim/scripts/generate_field_world.py --target fortress
```

不要手改 `worlds/*.sdf`：它们由脚本生成，下次重新生成会覆盖。

## 运行

```bash
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
gz sim -r sim/worlds/dase7503_field.sdf

# Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress
ign gazebo sim/worlds/dase7503_field_fortress.sdf

# 已安装并编译 linorobot2 后：在 Start 中心生成麦轮机器人，车头朝 -X
ros2 launch sim/launch/field_with_robot.launch.py
```

两个世界的几何完全一致，仅 SDF 版本和 Gazebo 系统插件命名不同。
