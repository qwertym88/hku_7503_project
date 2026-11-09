#!/usr/bin/env python3
"""T0 mock 底盘（决策 D9）：M2 固件就绪前的占位里程计。

发布 odom/unfiltered（nav_msgs/Odometry，零速、位姿原点），让 EKF → slam_toolbox/Nav2
的全链路在只有雷达的机器上就能跑通、联调。

M2 固件就绪后：停掉本节点即可，固件的 /odom 在 bringup 里 remap 成 odom/unfiltered，
其余代码零改动（见《模块接口说明》#2）。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class MockOdom(Node):

    def __init__(self):
        super().__init__('mock_odom')
        self.pub = self.create_publisher(Odometry, 'odom/unfiltered', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.msg = Odometry()
        self.msg.header.frame_id = 'odom'
        self.msg.child_frame_id = 'base_footprint'
        # 位姿/速度全零：车"原地不动"，SLAM 靠扫描匹配、Nav2 靠 AMCL 修正
        self.msg.pose.pose.orientation.w = 1.0
        self.get_logger().warn(
            'mock 底盘运行中：odom/unfiltered 为零速占位数据（仅用于联调，'
            '真机请勿启动本节点）')

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockOdom()
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
