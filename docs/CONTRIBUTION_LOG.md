# 贡献日志（用于 5% 同伴互评）

规则：**每周追加**，一行一件事；由各成员自己填，M5 汇总。演示前导出作为互评依据。

| 周次 | 成员 | 任务 | 产出 | 耗时(h) | 日期 |
|---|---|---|---|---|---|
| W1 | M1 | 示例：底盘布局草图 | `cad/assembly/` | 3 | 2026-10-24 |
| W3 | M4 | MS200 雷达驱动接入（Oradar 官方驱动 vendor + 协议确认 230400/47B 包） | `ros2_ws/src/oradar_lidar/`，/scan 10.03Hz 实测 | 2 | 2026-11-09 |
| W3 | M4 | M4 导航包：EKF/slam/Nav2 配置 + 停靠点 + 三条 launch + mock 底盘 + 兜底航点驱动 | `ros2_ws/src/robot_navigation/`，全链路真雷达实测（建图/保存/导航 SUCCEEDED/cmd_vel Twist） | 5 | 2026-11-09 |
| W3 | M4 | 导航 URDF（TF 链 base_footprint→laser/imu/cam）+ Harmonic 场地世界验证（HANDOFF #4 关闭） | `robot_description/urdf/dase7503_nav.urdf`；`sim/` 500 迭代 0 错 | 1 | 2026-11-09 |
| W3 | M3 | QR 感知包全链路：纯 Python 解码核心（OpenCV→放大→pyzbar 三级）+ 多帧投票去抖 + picamera2/回放/USB 三帧源 + 薄 ROS 封装 → /qr | `ros2_ws/src/qr_perception/`；本机图片回放 6 类码各发一次实测 | 5 | 2026-11-10 |
| W3 | M3 | 离线测试集（官方 6 码 + 旋转/倾斜/模糊/明暗/低分辨率/噪声 72 张）+ 评测/预览/生成三工具 + 打印样张 + Pi 部署脚本 | `tests/qr_dataset/`、`tools/qr_*.py`、`tools/setup_pi_qr.sh`；离线回归 100%、单测 8/8 | 3 | 2026-11-10 |
| | | | | | |

## 填写说明
- "产出"写**仓库里的路径**或实物/截图位置，便于核查。
- 不要写"参与讨论"这类无法验证的事项。
- 截止前若未填，视为该周无贡献记录。
