# M3/M4/M5 Gazebo integration rig

Run from the repository root after sourcing ROS 2 Jazzy and this workspace:

```bash
cd ros2_ws
colcon build --packages-up-to sim_control --symlink-install
source install/setup.bash
ros2 launch sim_control sim_ideal.launch.py
```

This starts the Harmonic field, the CAD robotcar model with ideal motion at `(1.0, 0.3, π/2)`, a front RGB camera and 360° LiDAR, RViz, and a small hold-to-move panel. Hold the arrow buttons (or W/A/S/D); releasing sends zero velocity. The panel publishes `geometry_msgs/Twist` on `/cmd_vel`.

The ideal base integrates `linear.x` and `angular.z` directly, updates the Gazebo model pose through the `ros_gz_interfaces/SetEntityPose` bridge, and publishes `/odom` and `odom → base_footprint`. It intentionally bypasses wheel dynamics and collision response; this rig is for interface/perception integration, not contact or motor validation. A 0.35 s command watchdog stops motion if the panel exits unexpectedly.

Sensor outputs are `/scan`, `/camera/image_raw`, and `/camera/camera_info`. Geometry and sensor mount transforms come from the supplied SolidWorks `robotcar/urdf/robotcar.urdf`. Meshes are bundled in `robot_description/meshes/robotcar`; the launch file resolves their installed paths for Gazebo, so the Desktop export is not needed at runtime. The integration URDF `robot_description/urdf/dase7503_sim.urdf` and Gazebo model `sim/models/ideal_robot/model.sdf` provide matching fixed transforms. Gazebo and ROS nodes use simulation time.

| Link | Parent | XYZ (m) | RPY (rad) |
| --- | --- | --- | --- |
| `base_link` | `base_footprint` | `(0, 0, 0.016328982)` | `(0, 0, 0)` |
| `Lidar` | `base_link` | `(0.068835963, 0, 0.0848)` | `(0, 0, 0)` |
| `Camera` | `base_link` | `(0.001, 0, 0.127)` | `(0, 0, 0)` |
| `laser` | `Lidar` | `(0, 0, 0)` | `(0, 0, 0)` |
| `cam_link` | `Camera` | `(0, 0, 0)` | `(0, 0, 0)` |
| `camera_optical_frame` | `cam_link` | `(0, 0, 0)` | `(-π/2, 0, -π/2)` |

The ground offset is computed from the lowest tyre mesh vertex after applying the original wheel joint transforms. The LiDAR and camera link origins are therefore approximately **0.101129 m** and **0.143329 m** above the floor. The camera looks horizontally along vehicle +X; the previous provisional downward pitch is removed. These are the exported mounting link origins; the CAD export does not provide a separate calibrated optical centre or LiDAR measurement-centre offset. Sensor parameters (camera FOV/resolution and LiDAR range/rate) remain provisional simulation settings.

Wheel joint origins and all CAD meshes are preserved. The simulation fixes the wheel joints because motion is imposed on the whole model; no wheel dynamics or joint-state publisher is required. The real-robot navigation URDF remains separate.

Useful launch options: `gui:=false rviz:=false panel:=false` for headless runs, or `spawn_x:=1.0 spawn_y:=0.3 spawn_yaw:=1.5707963` to choose another initial pose.
