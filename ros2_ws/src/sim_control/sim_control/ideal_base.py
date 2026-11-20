#!/usr/bin/env python3
"""Perfect planar kinematics that directly poses the Gazebo model."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import TransformBroadcaster


class IdealBase(Node):
    def __init__(self):
        super().__init__('ideal_base')
        self.declare_parameter('world_name', 'dase7503_field')
        self.declare_parameter('entity_name', 'ideal_robot')
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 0.3)
        self.declare_parameter('start_yaw', math.pi / 2)
        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.yaw = float(self.get_parameter('start_yaw').value)
        self.vx = 0.0
        self.wz = 0.0
        self.last_command_wall = 0.0
        self.last_tick = self.get_clock().now()
        self.pose_future = None
        self.pose_service_confirmed = False
        self.pose_failure_reported = False

        world = self.get_parameter('world_name').value
        self.entity_name = self.get_parameter('entity_name').value
        self.pose_client = self.create_client(
            SetEntityPose, f'/world/{world}/set_pose')
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info(
            f'Ideal base at ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f}); '
            'linear.x and angular.z are integrated without wheel dynamics.')

    def on_command(self, msg):
        self.vx = max(-0.5, min(0.5, float(msg.linear.x)))
        self.wz = max(-1.5, min(1.5, float(msg.angular.z)))
        self.last_command_wall = time.monotonic()

    def _move_gazebo_model(self):
        if self.pose_future is not None:
            if not self.pose_future.done():
                return
            try:
                result = self.pose_future.result()
                if not result.success:
                    if not self.pose_failure_reported:
                        self.get_logger().warning(
                            'Gazebo set_pose returned success=false; waiting for model spawn')
                        self.pose_failure_reported = True
                elif not self.pose_service_confirmed:
                    self.get_logger().info('Gazebo ideal_robot pose service connected')
                    self.pose_service_confirmed = True
            except Exception as exc:  # service may race the initial model spawn
                self.get_logger().warning(f'Gazebo set_pose failed: {exc}')
            self.pose_future = None

        if not self.pose_client.service_is_ready():
            return
        request = SetEntityPose.Request()
        request.entity.name = self.entity_name
        request.pose.position.x = self.x
        request.pose.position.y = self.y
        request.pose.position.z = 0.0
        request.pose.orientation.z = math.sin(self.yaw / 2.0)
        request.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_future = self.pose_client.call_async(request)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds / 1e9
        self.last_tick = now
        if dt < 0.0 or dt > 0.2:
            dt = 0.0

        # Stop if the panel/other controller disappears; use wall time so the
        # timeout remains meaningful if Gazebo is paused.
        if time.monotonic() - self.last_command_wall > 0.35:
            self.vx = 0.0
            self.wz = 0.0

        next_yaw = self.yaw + self.wz * dt
        if abs(self.wz) > 1e-9:
            self.x += self.vx / self.wz * (math.sin(next_yaw) - math.sin(self.yaw))
            self.y -= self.vx / self.wz * (math.cos(next_yaw) - math.cos(self.yaw))
        else:
            self.x += self.vx * math.cos(self.yaw) * dt
            self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(next_yaw), math.cos(next_yaw))

        stamp = now.to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_pub.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = transform.transform.rotation.z
        odom.pose.pose.orientation.w = transform.transform.rotation.w
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)
        self._move_gazebo_model()


def main(args=None):
    rclpy.init(args=args)
    node = IdealBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.vx = 0.0
        node.wz = 0.0
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
