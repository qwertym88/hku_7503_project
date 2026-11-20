# M3/M4/M5 Gazebo integration rig

Run from the repository root after sourcing ROS 2 Jazzy and this workspace:

```bash
cd ros2_ws
colcon build --packages-select sim_control --symlink-install
source install/setup.bash
ros2 launch sim_control sim_ideal.launch.py
```

This starts the Harmonic field, the 0.30 x 0.30 m ideal robot at `(1.0, 0.3, π/2)`, a front RGB camera and 360° LiDAR, RViz, and a small hold-to-move panel. Hold the arrow buttons (or W/A/S/D); releasing sends zero velocity. The panel publishes `geometry_msgs/Twist` on `/cmd_vel`.

The ideal base integrates `linear.x` and `angular.z` directly, updates the Gazebo model pose through the `ros_gz_interfaces/SetEntityPose` bridge, and publishes `/odom` and `odom → base_footprint`. It intentionally bypasses wheel dynamics and collision response; this rig is for interface/perception integration, not contact or motor validation. A 0.35 s command watchdog stops motion if the panel exits unexpectedly.

Sensor outputs are `/scan`, `/camera/image_raw`, and `/camera/camera_info`. The LiDAR frame is `laser` at 0.09 m above ground. The camera is on `cam_link` at 0.15 m with optical frame `camera_optical_frame`. The integration URDF `robot_description/urdf/dase7503_sim.urdf` provides the matching TF chain. Gazebo and ROS nodes use simulation time.

Useful launch options: `gui:=false rviz:=false panel:=false` for headless runs, or `spawn_x:=1.0 spawn_y:=0.3 spawn_yaw:=1.5707963` to choose another initial pose.
