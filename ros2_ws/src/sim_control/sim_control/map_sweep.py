"""Survey the empty Gazebo field, then save its LiDAR occupancy grid."""

import math
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from mission_control.clearance import SceneClearance, drive_violation, turn_violation
from mission_control.route import angle, load_config, pose


def survey_path(cfg):
    start = pose(cfg['start_pose'])
    points = [(1., 1.6), (1.55, 1.6), (1.55, 2.45), (0.75, 2.45),
              (0.75, 1.6), (1., 1.6), start[:2]]
    scene = SceneClearance(cfg, {})
    previous = start
    for x, y in points:
        yaw = math.atan2(y - previous[1], x - previous[0])
        heading = (*previous[:2], yaw)
        target = (x, y, yaw)
        hit = turn_violation(scene, previous, yaw, 0.12) or drive_violation(scene, heading, target, 0.12)
        if hit:
            raise ValueError(f'Unsafe survey leg {previous[:2]} -> {(x, y)}: {hit}')
        previous = target
    hit = turn_violation(scene, previous, start[2], 0.12)
    if hit:
        raise ValueError(f'Unsafe survey finish at {start[:2]}: {hit}')
    return start, iter(points)


class MapSweep(Node):
    def __init__(self):
        super().__init__('field_map_sweep')
        self.declare_parameter('config', '')
        self.declare_parameter('output', '')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('The automatic survey requires Gazebo simulation time')
        self.start_pose, self.points = survey_path(load_config(self.get_parameter('config').value))
        self.target = next(self.points)
        self.output = Path(self.get_parameter('output').value)
        self.map_file = self.output.with_name(self.output.name + '.yaml')
        self.image_file = self.output.with_name(self.output.name + '.pgm')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom = None
        self.map_seen = None
        self.started = time.monotonic()
        self.moving = False
        self.settled_at = None
        self.staging = None
        self.save_job = None
        self.result = None
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.odom = (p.x, p.y, yaw, stamp, time.monotonic())

    def on_map(self, msg):
        if 0 in msg.data and 100 in msg.data:
            self.map_seen = time.monotonic()

    def command(self, linear=0., angular=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.cmd.publish(msg)

    def save(self):
        self.staging = tempfile.TemporaryDirectory(dir=self.output.parent, prefix='.field-mapping-')
        self.save_job = subprocess.Popen([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-t', '/map',
            '-f', str(Path(self.staging.name) / self.output.name),
            '--fmt', 'pgm', '--free', '0.196', '--occ', '0.65',
            '--ros-args', '-p', 'save_map_timeout:=10.0', '-p', 'map_subscribe_transient_local:=true',
        ])

    def finish_save(self):
        if self.save_job.returncode:
            raise RuntimeError(f'Nav2 map saver exited with status {self.save_job.returncode}')
        staging = Path(self.staging.name)
        image, metadata = staging / self.image_file.name, staging / self.map_file.name
        # Publish the YAML last, after its relative image path is usable.
        self.image_file.hardlink_to(image)
        try:
            self.map_file.hardlink_to(metadata)
        except OSError:
            self.image_file.unlink()
            raise
        self.get_logger().info(f'Saved scan map: {self.map_file}')
        self.result = 0

    def tick(self):
        now = time.monotonic()
        if now - self.started > 120:
            raise RuntimeError('Survey timed out waiting for motion, a fresh map, or map save')
        if self.count_publishers('/cmd_vel') > 1:
            raise RuntimeError('Close teleop and other /cmd_vel publishers before scanning')
        if self.save_job is not None:
            if self.save_job.poll() is not None:
                self.finish_save()
            return
        if self.odom is None:
            return
        x, y, yaw, stamp, receipt = self.odom
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if now - receipt > 0.5 or not -0.1 <= age <= 0.5:
            self.command()
            if self.moving:
                raise RuntimeError('Lost fresh Gazebo odometry during the survey')
            return
        if not self.moving:
            if self.map_seen is None:
                return
            if math.dist((x, y), self.start_pose[:2]) > 0.03 or abs(angle(yaw - self.start_pose[2])) > 0.08:
                raise RuntimeError('Survey must start at the configured Gazebo spawn pose')
            self.moving = True
            self.get_logger().info('LiDAR map received; starting the empty-field loop')

        if self.target is not None:
            dx, dy = self.target[0] - x, self.target[1] - y
            distance = math.hypot(dx, dy)
            if distance <= 0.018:
                self.command()
                self.target = next(self.points, None)
                return
            error = angle(math.atan2(dy, dx) - yaw)
            speed = min(0.22, 1.6 * distance) if abs(error) < 0.10 else 0.
            self.command(speed, max(-0.85, min(0.85, 2.5 * error)))
            return

        error = angle(self.start_pose[2] - yaw)
        if abs(error) > 0.03:
            self.command(angular=max(-0.85, min(0.85, 2.5 * error)))
            return
        self.command()
        if self.settled_at is None:
            self.settled_at = now
        elif now - self.settled_at >= 2.0 and self.map_seen > self.settled_at:
            self.save()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapSweep()
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0 if node.result == 0 else 1
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 1
    except (OSError, RuntimeError, ValueError) as exc:
        rclpy.logging.get_logger('field_map_sweep').error(str(exc))
        return 1
    finally:
        if node is not None:
            if rclpy.ok():
                node.command()
            if node.save_job is not None and node.save_job.poll() is None:
                node.save_job.terminate()
                node.save_job.wait()
            if node.staging is not None:
                node.staging.cleanup()
            node.destroy_node()
        rclpy.try_shutdown()
