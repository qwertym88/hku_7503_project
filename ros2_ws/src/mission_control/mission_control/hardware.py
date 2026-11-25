"""Fixed-route execution using AMCL and the chassis micro-ROS interfaces."""

import json
import math
import time

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt8

from .node import MissionNode, main as run_node, stamp_seconds
from .route import angle, compose, pose


class HardwareMission(MissionNode):
    simulation = False

    def setup_cargo(self):
        self.lift_baseline = None
        self.progress_correction = None
        self.create_subscription(String, '/chassis/diagnostics', self.on_cargo, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_raw_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)

    def on_raw_odom(self, msg):
        velocity = msg.twist.twist
        if (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_footprint'
                and all(math.isfinite(v) for v in (velocity.linear.x, velocity.linear.y, velocity.angular.z))):
            self.feedback['raw_odom'] = (time.monotonic(), stamp_seconds(msg))
        else:
            self.feedback.pop('raw_odom', None)

    def on_imu(self, msg):
        if msg.header.frame_id == 'imu_link' and math.isfinite(msg.angular_velocity.z):
            self.feedback['imu'] = (time.monotonic(), stamp_seconds(msg))
        else:
            self.feedback.pop('imu', None)

    def on_cargo(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            status = None
        integer_fields = ('imu_valid', 'enabled', 'lift_inhibited', 'lifter_position',
                          'lifter_rx', 'lifter_rejected', 'lifter_completed')
        text_fields = ('lifter_state', 'lifter_fault', 'lifter_last_result')
        if (not isinstance(status, dict)
                or not all(type(status.get(key)) is int for key in integer_fields)
                or not all(isinstance(status.get(key), str) for key in text_fields)):
            self.cargo_status = None
            return
        self.cargo_status = status
        self.feedback['cargo'] = (time.monotonic(), self.ros_seconds())

    def health(self, precheck=False):
        fault = super().health(precheck)
        if fault:
            return fault
        limit = self.cfg['control']['feedback_timeout_s']
        for name in ('raw_odom', 'imu'):
            if not self.fresh(name, limit):
                return f'{name} missing/stale'
        if not self.cmd_pub.get_subscription_count() or not self.lift_pub.get_subscription_count():
            return 'Waiting for chassis /cmd_vel and /lifter subscriptions'
        transform = self.map_transform()
        if transform is None:
            return 'Waiting for AMCL map-to-odom transform'
        t, q = transform.transform.translation, transform.transform.rotation
        correction = (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if not all(math.isfinite(v) for v in correction):
            return 'Invalid map-to-odom transform'
        self.p = compose(correction, self.odom_pose)
        if self.mission.state == 'RUNNING' and self.progress_correction is not None:
            projected = compose(self.progress_correction, self.odom_pose)
            if math.dist(projected[:2], self.p[:2]) > 0.05 or abs(angle(projected[2]-self.p[2])) > 0.12:
                return 'AMCL pose correction exceeded 0.05 m / 0.12 rad in one update'
        self.progress_correction = correction
        if precheck:
            start = pose(self.cfg['start_pose'])
            if math.dist(self.p[:2], start[:2]) > 0.025 or abs(angle(self.p[2]-start[2])) > 0.05:
                return 'Place the robot at the configured start pose and set the RViz initial pose'

    def cargo_health(self, precheck):
        status = self.cargo_status
        if not status or not self.fresh('cargo', self.cfg['control']['cargo_timeout_s']):
            return 'Chassis diagnostics missing/stale/invalid'
        if status.get('command_locked'):
            return 'Chassis commands are locked after reconnect; stop controllers and call /chassis/unlock'
        if not status['imu_valid']:
            return 'Chassis reports invalid IMU'
        if status['lifter_fault'] != 'none' or status['lifter_state'] not in ('idle', 'raising', 'lowering'):
            return f"Lifter {status['lifter_state']}: {status['lifter_fault']}"
        if self.count_publishers('/lifter') != 1:
            return '/lifter must have exactly one publisher'
        blocked = status['lift_inhibited'] or status['lifter_state'] != 'idle'
        if precheck and (blocked or status['enabled'] or status['lifter_position'] != 0):
            return 'Chassis must be stationary and firmware must report lifter idle at position 0'
        if self.mission.state == 'RUNNING' and self.mission.action.kind in ('drive', 'turn', 'curve'):
            if blocked or status['lifter_position'] != int(self.mission.carrying is not None):
                return 'Lifter state disagrees with the active drive action'

    def start_cargo_effect(self, action):
        self.lift_baseline = None

    def poll_cargo_effect(self, action):
        status = self.cargo_status
        baseline = self.lift_baseline
        if baseline is None:
            if not (status['enabled'] or status['lift_inhibited'] or status['lifter_state'] != 'idle'):
                self.lift_baseline = status
                self.lift_pub.publish(UInt8(data=int(action.kind == 'lift')))
            return
        if status['lifter_rx'] == baseline['lifter_rx']:
            return
        result = status['lifter_last_result']
        if (status['lifter_rx'] != baseline['lifter_rx'] + 1
                or status['lifter_rejected'] != baseline['lifter_rejected']
                or result not in ('accepted', 'duplicate')):
            self.mission.acknowledge(action.id, False, f'Lifter command mismatch/rejected: {result}')
            return
        target = int(action.kind == 'lift')
        completed = (status['lifter_completed'] > baseline['lifter_completed']
                     if result == 'accepted' else baseline['lifter_position'] == target)
        if (completed and status['lifter_state'] == 'idle' and not status['lift_inhibited']
                and status['lifter_position'] == target):
            self.mission.acknowledge(action.id, True, f'Firmware target {target} ({result})')

    def reset_service(self, request, response):
        if self.mission.state == 'RUNNING':
            response.message = 'Cancel first and restore the physical scene'
        elif self.camera_job is not None and not self.camera_job.done():
            response.message = 'Wait for the previous photo write to complete'
        else:
            self.zero()
            fault = self.health(precheck=True)
            if fault:
                response.message = fault
            else:
                self.finish_reset()
                response.success = True
                response.message = 'Mission reset using the restored physical scene'
        return response


def main(args=None):
    run_node(args, node_type=HardwareMission)
