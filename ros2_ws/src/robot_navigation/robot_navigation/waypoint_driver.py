#!/usr/bin/env python3
"""兜底航点驱动（M4-10）：Nav2 调不通时的备用方案。

不依赖 Nav2/AMCL：直接用 EKF 的 /odom 做里程计定位，按固定航点序列走，
雷达做紧急停车。~120 行，无外部依赖。

用法（真机，EKF 已在跑）：
  # 走默认路线：起点 → Rack A → 目的区
  ros2 run robot_navigation waypoint_driver

  # 自定义路线（航点名见 config/dock_points.yaml）
  ros2 run robot_navigation waypoint_driver --ros-args \
      -p route:="[rack_b,destination]" -p linear_speed:=0.15

注意：里程计会漂移（无 AMCL 修正），只适合短距离、结构化路线的兜底。
"""

import math
import os

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from ament_index_python.packages import get_package_share_directory

# 控制参数（保守：兜底方案宁慢勿撞）
LINEAR_SPEED = 0.15       # m/s
ANGULAR_SPEED = 0.6       # rad/s
YAW_TOLERANCE = 0.05      # rad，航向对准阈值
XY_TOLERANCE = 0.05       # m，到点阈值（与 M4-6 的 ±5cm 一致）
SLOW_RADIUS = 0.3         # m，接近航点开始减速
STOP_DIST = 0.25          # m，雷达紧急停车距离（前方扇形内最近点）


def yaw_from_quaternion(q):
    """四元数 → 偏航角（仅平面）。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WaypointDriver(Node):

    def __init__(self):
        super().__init__('waypoint_driver')

        self.declare_parameter('route', ['rack_a', 'destination'])
        self.declare_parameter('linear_speed', LINEAR_SPEED)
        self.declare_parameter('angular_speed', ANGULAR_SPEED)
        self.declare_parameter(
            'waypoints_yaml',
            os.path.join(get_package_share_directory('robot_navigation'),
                         'config', 'dock_points.yaml'))

        route = list(self.get_parameter('route').value)
        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)

        with open(self.get_parameter('waypoints_yaml').value, 'r') as f:
            cfg = yaml.safe_load(f)
        self.waypoints = []
        for name in route:
            if name not in cfg:
                raise ValueError(f'航点 {name} 不在 dock_points.yaml 里')
            wp = cfg[name]
            self.waypoints.append(
                (float(wp['x']), float(wp['y']), float(wp['yaw']), name))

        self.pose = None            # (x, y, yaw)，收到 /odom 前为 None
        self.front_min = float('inf')
        self.idx = 0

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info(
            f'兜底航点驱动：{len(self.waypoints)} 个航点 '
            + ' → '.join(n for *_, n in self.waypoints))

    # ---------- 回调 ----------

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))
        if not hasattr(self, '_got_first_odom'):
            self._got_first_odom = True
            self.get_logger().info(
                f'收到首条 /odom：位姿 ({p.x:.2f}, {p.y:.2f})')

    def on_scan(self, msg):
        """取车头前方 ±45° 扇形内的最近距离。"""
        n = len(msg.ranges)
        if n == 0:
            return
        # /scan 角度从 angle_min 起；车头对应 0 rad
        front_min = float('inf')
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                a = normalize_angle(msg.angle_min + i * msg.angle_increment)
                if abs(a) < math.pi / 4:  # 前方 ±45°
                    front_min = min(front_min, r)
        self.front_min = front_min

    # ---------- 主循环 ----------

    def tick(self):
        cmd = Twist()
        if self.pose is None:
            self.cmd_pub.publish(cmd)  # 还没定位：停车等待
            return

        if self.idx >= len(self.waypoints):
            self.get_logger().info('全部航点完成')
            self.timer.cancel()
            return

        tx, ty, tyaw, name = self.waypoints[self.idx]
        x, y, yaw = self.pose
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)

        # 1) 雷达紧急停车：前方太近 → 禁平移（原地转向仍允许：圆形 footprint 转圈不会扫到障碍）
        front_blocked = self.front_min < STOP_DIST

        if dist > XY_TOLERANCE:
            # 2) 先对准航点方向，再前进
            err = normalize_angle(bearing - yaw)
            if abs(err) > YAW_TOLERANCE:
                cmd.angular.z = max(-self.w, min(self.w, 2.0 * err))
            else:
                cmd.angular.z = max(-self.w, min(self.w, 0.8 * err))
                speed = self.v if dist > SLOW_RADIUS else \
                    self.v * max(0.3, dist / SLOW_RADIUS)
                if not front_blocked:
                    cmd.linear.x = speed
        else:
            # 3) 到点：转到目标朝向，然后下一个航点
            err = normalize_angle(tyaw - yaw)
            if abs(err) > 0.1:
                cmd.angular.z = max(-self.w, min(self.w, 1.5 * err))
            else:
                self.get_logger().info(
                    f'到达航点 {name} (误差 {dist*100:.1f} cm)')
                self.idx += 1

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
