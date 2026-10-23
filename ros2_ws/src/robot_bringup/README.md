# robot_bringup —— 统一启动与配置（M5）

## 职责
一个 launch 按顺序拉起全系统，集中放配置。**不含算法。**

```text
bringup.launch.py
  1. QR 感知（picamera2 内核）        → /qr
  2. 雷达驱动（MS200）                → /scan
  3. micro-ROS agent（连接 ESP32）    ↔ /cmd_vel /odom /imu/data /lifter
  4. 定位（EKF + AMCL）               → map→odom
  5. 导航（Nav2）                     → /cmd_vel
  6. mission_control                  → /lifter
```

## 目录（待建）
```text
robot_bringup/
├─ launch/
│  ├─ bringup.launch.py      全系统
│  ├─ sensors.launch.py      只起相机+雷达
│  └─ navigation.launch.py   只起定位+导航
└─ config/
   ├─ interfaces.yaml        6 个话题名集中定义
   ├─ nav2_params.yaml       导航参数
   ├─ mission.yaml           状态机参数（货架坐标、超时）
   └─ camera_qr.yaml         相机与 QR 参数
```

当前为空壳。**配置里的坐标/外参必须实测填写，禁止沿用示例值。**
