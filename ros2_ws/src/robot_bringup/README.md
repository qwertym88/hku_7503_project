# robot_bringup：建图/定位与固定路线启动（M5）

2026-11-11，执行基线v2.1，当前仅README，启动包与launch待实现。与M4交付两个模式：

- 建图：micro-ROS agent、真雷达、EKF与slam_toolbox、RViz，人工采图；mission关闭。
- 定位执行：agent、真雷达、同一EKF、map_server+AMCL、固定路线mission及RViz。关闭slam_toolbox、Nav2 planner/controller与旧waypoint_driver。

真实建图与实时定位必做；在线规划和路径跟随可选。原始/odom由固件发，EKF输出/odometry/filtered及唯一局部TF；SLAM/AMCL互斥发map→odom。详细接线见[架构](../../../docs/给队友看/建图定位与固定路线方案.md)。

启动后先零速待命；地图、首次定位、scan/局部反馈/TF及参数就绪才接受人工开始。不增加厘米级定位启动门槛。运行时真正链路失效、故障或180s截止须中断；重启/急停释放不自动续跑；遥控建图、自动和手动模式互斥，速度唯一发布。

交付真实采图/保存重载、执行模式初始化、开始/取消/复位命令与另一人复现记录；实现后补[运行说明](../../../docs/给队友看/运行说明.md)。职责与证据见[任务安排](../../../docs/给队友看/分工与任务拆解.md)。
