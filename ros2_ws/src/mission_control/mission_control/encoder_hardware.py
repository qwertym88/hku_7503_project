"""Follow the fixed hardware route using only the chassis wheel odometry."""

import math

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .hardware import HardwareMission
from .node import main as run_node
from .route import compose, pose, relative


class EncoderHardwareMission(HardwareMission):
    encoder_only = True

    def setup_cargo(self):
        super().setup_cargo()
        self.odom_origin = None
        self.route_origin = pose(self.cfg['start_pose'])
        self.route_tf = TransformBroadcaster(self)

    def on_odom(self, msg):
        super().on_odom(msg)
        self.on_raw_odom(msg)

    def localization_ready(self):
        return (not self.reseed_at_start and self.odom_origin is not None
                and self.fresh('odom', self.cfg['control']['feedback_timeout_s']))

    def prepare_start_localization(self, pending, now):
        fault = self.chassis_feedback_health(precheck=True)
        if fault:
            return fault
        self.odom_origin = self.odom_pose
        self.reseed_at_start = False

    def feedback_health(self, precheck):
        fault = self.chassis_feedback_health(precheck)
        if fault:
            return fault
        if self.odom_origin is None or self.reseed_at_start:
            return 'Waiting to anchor encoder odometry to the start pose'
        self.p = compose(self.route_origin, relative(self.odom_origin, self.odom_pose))
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.route_frame
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x, transform.transform.translation.y, yaw = self.p
        transform.transform.rotation.z = math.sin(yaw / 2.)
        transform.transform.rotation.w = math.cos(yaw / 2.)
        self.route_tf.sendTransform(transform)

    def motion_hold(self, now):
        return False

    def finish_reset(self):
        self.odom_origin = None
        super().finish_reset()

    def reset_detail(self):
        return 'Reset complete; encoder odometry will be anchored at the next start'


def main(args=None):
    run_node(args, node_type=EncoderHardwareMission)
