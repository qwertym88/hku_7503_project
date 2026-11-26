#!/usr/bin/env python3
"""Record teleoperation and replay it on the chassis with odometry feedback."""

import argparse
import json
import math
from pathlib import Path
import signal
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String, UInt8

from chassis_route_budget import ROOT, constant
from mission_control.route import angle, relative


PERIOD = 0.05
ZERO = (0., 0.)
CHASSIS = ('chassis', '/')
TELEOP = ('dase_teleop', '/')
TOPICS = ('/cmd_vel', '/lifter')
MAX_ERROR = (0.20, math.radians(20))
STOP_ERROR = (0.08, math.radians(8))


def limits():
    header = ROOT / 'firmware/chassis/components/car_motion/car_motion.h'
    return (constant(header, 'CHASSIS_MAX_LINEAR_MPS'),
            constant(header, 'CHASSIS_MAX_ANGULAR_RADPS') / 2)


def stopped(velocity):
    return abs(velocity[0]) <= 0.01 and abs(velocity[1]) <= 0.02


class Feedback(Node):
    def __init__(self, name):
        super().__init__(name)
        self.odom = self.diagnostic = None
        self.odom_at = self.diagnostic_at = -math.inf
        self.error = None
        self.last_stamp = 0
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/chassis/diagnostics', self.on_diagnostic, qos_profile_sensor_data)

    def endpoints(self, topic, publishers):
        query = self.get_publishers_info_by_topic if publishers else self.get_subscriptions_info_by_topic
        return [(entry.node_name, entry.node_namespace) for entry in query(topic)]

    def on_odom(self, msg):
        position, q = msg.pose.pose.position, msg.pose.pose.orientation
        velocity = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if (msg.header.frame_id, msg.child_frame_id) != ('odom', 'base_footprint') or (
            not all(math.isfinite(v) for v in (position.x, position.y, q.x, q.y, q.z, q.w, *velocity))
            or abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.) > 0.05
            or stamp <= 0 or stamp < self.last_stamp
        ):
            self.error = '里程计无效或时间戳倒退'
            return
        if stamp > self.last_stamp:
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            self.odom = (position.x, position.y, yaw), velocity
            self.last_stamp, self.odom_at = stamp, time.monotonic()

    def on_diagnostic(self, msg):
        try:
            data = json.loads(msg.data)
            for key in ('enabled', 'lift_inhibited'):
                if data[key] not in (0, 1):
                    raise ValueError(key)
            if (len(data['counts']) != 4 or any(type(n) is not int for n in data['counts'])
                    or len(data['speed']) != 4 or any(not math.isfinite(v) for v in data['speed'])
                    or any(type(data[key]) is not int for key in
                           ('lifter_rx', 'lifter_rejected', 'lifter_completed', 'lifter_position'))
                    or any(not isinstance(data[key], str) for key in
                           ('lifter_state', 'lifter_fault', 'lifter_last_result'))
                    or type(data.get('cycles', 0)) is not int
                    or type(data.get('lifter_timeout_ms', 5000)) is not int
                    or not 0 < data.get('lifter_timeout_ms', 5000) <= 30000):
                raise ValueError('诊断字段无效')
        except (KeyError, TypeError, ValueError):
            self.error = '底盘诊断无效'
            return
        if (self.diagnostic is not None
                and data.get('cycles', 0) < self.diagnostic.get('cycles', 0)):
            self.error = '底盘控制周期计数倒退'
            return
        self.diagnostic, self.diagnostic_at = data, time.monotonic()

    def ready(self, now):
        return (now - self.odom_at <= 0.30 and now - self.diagnostic_at <= 1.4
                and self.endpoints('/odom', True) == [CHASSIS]
                and self.endpoints('/chassis/diagnostics', True) == [CHASSIS])

    def check_feedback(self, now):
        if self.error:
            raise RuntimeError(self.error)
        if not self.ready(now):
            raise RuntimeError('底盘里程计或诊断断流')
        if self.diagnostic.get('command_locked') or self.diagnostic['lifter_fault'] != 'none':
            raise RuntimeError('底盘已锁定或升降机构故障')

    def stationary(self):
        data = self.diagnostic
        return (stopped(self.odom[1]) and not data['enabled']
                and all(abs(v) <= 0.005 for v in data['speed']))


class Recorder(Feedback):
    def __init__(self):
        super().__init__('dase_route_recorder')
        self.origin = None
        self.start = 0.
        self.command, self.command_at = ZERO, -math.inf
        self.samples, self.diagnostics, self.events = [], [], []
        self.next_lift = None
        self.lift_count = 0
        self.max_linear, self.max_angular = limits()
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 20)
        self.create_subscription(UInt8, '/lifter', self.on_lift, 20)

    def graph_ok(self):
        return all(self.endpoints(topic, True) == [TELEOP] for topic in TOPICS)

    def on_command(self, msg):
        command = msg.linear.x, msg.angular.z
        if (msg.linear.y != 0. or not all(math.isfinite(v) for v in command)
                or abs(command[0]) > self.max_linear + 1e-9
                or abs(command[1]) > self.max_angular + 1e-9):
            self.error = '录到超出遥控范围的速度指令'
            return
        self.command, self.command_at = command, time.monotonic()
        if self.origin is not None:
            self.events.append({'t': self.command_at - self.start, 'cmd': command})

    def on_lift(self, msg):
        if self.origin is None:
            return
        if msg.data not in (0, 1) or self.next_lift is not None:
            self.error = '录到无效或过于密集的升降指令'
            return
        self.events.append({'t': time.monotonic() - self.start, 'lift': msg.data})
        self.next_lift = msg.data
        self.lift_count += 1

    def on_diagnostic(self, msg):
        super().on_diagnostic(msg)
        if self.origin is not None and not self.error:
            self.save_diagnostic()

    def save_diagnostic(self):
        previous = self.diagnostics[-1]['counts'] if self.diagnostics else None
        counts = self.diagnostic['counts']
        delta = None if previous is None else [
            (current - before + 2**31) % 2**32 - 2**31
            for current, before in zip(counts, previous)
        ]
        self.diagnostics.append(dict(self.diagnostic, t=self.diagnostic_at - self.start, count_delta=delta))

    def sample(self, now):
        command = self.command if now - self.command_at < 0.48 else ZERO
        pose, velocity = self.odom
        self.samples.append(dict(t=now - self.start, cmd=command, pose=relative(self.origin, pose),
                                 velocity=velocity, odom_stamp_ns=self.last_stamp,
                                 diagnostic_index=len(self.diagnostics) - 1, lift=self.next_lift))
        self.next_lift = None

    def run(self, wait, interrupted):
        def initially_ready(now):
            return (self.graph_ok() and self.ready(now) and self.stationary()
                    and self.command == ZERO and self.diagnostic['lifter_state'] == 'idle'
                    and not self.diagnostic['lift_inhibited'])

        wait_for_ready(self, wait, initially_ready, '打开本机遥控窗口，保持小车静止')
        self.start, self.origin = time.monotonic(), self.odom[0]
        self.save_diagnostic()
        print('开始录制。走完路径后松开摇杆，等升降完成，再按 Ctrl+C 保存。', flush=True)
        stop_deadline = math.inf
        deadline = self.start
        while True:
            now = wait(deadline)
            self.check_feedback(now)
            if not self.graph_ok():
                raise RuntimeError('录制期间遥控发布者发生变化')
            if now - deadline >= 2 * PERIOD:
                raise RuntimeError('20 Hz 录制循环超时')
            self.sample(now)
            if interrupted():
                stop_deadline = min(stop_deadline, now + 5.)
                baseline = self.diagnostics[0]
                received = self.diagnostic['lifter_rx'] - baseline['lifter_rx']
                if self.diagnostic['lifter_rejected'] != baseline['lifter_rejected']:
                    raise RuntimeError('录制中的升降指令被底盘拒绝')
                if (self.samples[-1]['cmd'] == ZERO and self.stationary()
                        and self.diagnostic['lifter_state'] == 'idle'
                        and not self.diagnostic['lift_inhibited'] and received == self.lift_count):
                    break
                if now >= stop_deadline:
                    raise RuntimeError('车未停稳或升降回执未完成')
            deadline = next_tick(deadline, now)
        if len(self.samples) < 2 or not any(sample['cmd'] != ZERO or sample['lift'] is not None
                                          for sample in self.samples):
            raise RuntimeError('没有录到有效的行驶或升降动作')
        return dict(format=1, rate_hz=20, samples=self.samples, diagnostics=self.diagnostics, events=self.events)


def next_tick(deadline, now):
    # Skip over a delayed tick without publishing catch-up bursts.
    return max(deadline + PERIOD, now + PERIOD if now - deadline >= PERIOD else deadline)


def wait_for_ready(node, wait, condition, message):
    print(message, flush=True)
    timeout = time.monotonic() + 15.
    ready_since = None
    while time.monotonic() < timeout:
        now = wait(time.monotonic() + PERIOD)
        if node.error:
            raise RuntimeError(node.error)
        if condition(now):
            node.check_feedback(now)
            ready_since = now if ready_since is None else ready_since
            if now - ready_since >= 0.30:
                return
        else:
            ready_since = None
    raise RuntimeError('等待底盘与控制接口就绪超时')


def correction(actual, reference, command, max_linear, max_angular):
    along, cross, heading = relative(actual, reference)
    linear, angular = command
    if linear:
        change = max(-0.08, min(0.08, along))
        # Preserve the direction of the recorded motion, including in reverse.
        direction = math.copysign(1., linear)
        linear = direction * max(0., min(max_linear, abs(linear) + direction * change))
        heading += direction * 2.0 * cross
    if command != ZERO:
        angular += max(-0.25, min(0.25, 1.5 * heading))
        angular = max(-max_angular, min(max_angular, angular))
    return linear, angular


class Player(Feedback):
    def __init__(self):
        super().__init__('dase_route_player')
        self.cmd_pub = self.lift_pub = None
        self.owner = (self.get_name(), self.get_namespace())
        self.max_linear, self.max_angular = limits()
        self.active = False

    def graph_ok(self, owner):
        return all(self.endpoints(topic, True) == owner
                   and self.endpoints(topic, False).count(CHASSIS) == 1 for topic in TOPICS)

    def publish(self, command):
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        self.cmd_pub.publish(msg)

    def prepare(self, wait, position):
        wait_for_ready(self, wait, lambda now: self.ready(now) and self.stationary() and self.graph_ok([]),
                       '等待底盘。请关闭本机遥控窗口和固定路线服务。')
        data = self.diagnostic
        if (data['lifter_state'] != 'idle' or data['lift_inhibited']
                or (position in (0, 1) and data['lifter_position'] != position)):
            raise RuntimeError(f'请先恢复录制时的升降初态（{position}，0 为放下，1 为抬升）')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(
            depth=1, lifespan=Duration(seconds=0.2)))
        self.lift_pub = self.create_publisher(UInt8, '/lifter', QoSProfile(
            depth=1, lifespan=Duration(seconds=0.3)))
        wait_for_ready(self, wait, lambda now: self.graph_ok([self.owner]) and self.ready(now)
                       and self.stationary() and self.cmd_pub.get_subscription_count()
                       and self.lift_pub.get_subscription_count(), '正在连接底盘控制接口')

    def lift_complete(self, baseline, target):
        status = self.diagnostic
        received = status['lifter_rx'] - baseline['lifter_rx']
        if received == 0:
            return False
        result = status['lifter_last_result']
        if (received != 1 or status['lifter_rejected'] != baseline['lifter_rejected']
                or result not in ('accepted', 'duplicate')):
            raise RuntimeError(f'升降指令被拒绝或回执不一致（{result}）')
        completed = (status['lifter_completed'] > baseline['lifter_completed'] if result == 'accepted'
                     else baseline['lifter_position'] == target)
        return (completed and status['lifter_state'] == 'idle'
                and status['lifter_position'] == target and not status['lift_inhibited'])

    def check_pose(self, actual, reference, tolerance):
        distance = math.dist(actual[:2], reference[:2])
        heading = abs(angle(actual[2] - reference[2]))
        if distance > tolerance[0] or heading > tolerance[1]:
            raise RuntimeError(f'偏离录制轨迹（位置 {distance:.3f} m，航向 {math.degrees(heading):.1f}°）')

    def run(self, recording, wait):
        samples = recording['samples']
        self.prepare(wait, recording['diagnostics'][0]['lifter_position'])
        origin = self.odom[0]
        print('开始回放，按 Ctrl+C 停车。', flush=True)
        self.active = True
        index = 0
        deadline = time.monotonic()
        still_since = pending = blocked_since = None
        while True:
            now = wait(deadline)
            if now - deadline >= 2 * PERIOD:
                raise RuntimeError('20 Hz 回放循环超时')
            self.check_feedback(now)
            if not self.graph_ok([self.owner]):
                raise RuntimeError('回放期间控制接口被占用或底盘断开')
            sample = samples[min(index, len(samples) - 1)]
            target = sample['lift'] if index < len(samples) else None
            command = tuple(sample['cmd'])
            actual = relative(origin, self.odom[0])
            self.check_pose(actual, sample['pose'], MAX_ERROR)
            if self.stationary():
                still_since = now if still_since is None else still_since
            else:
                still_since = None
            if pending:
                baseline, lift_target, until = pending
                if self.lift_complete(baseline, lift_target):
                    pending = None
                elif now >= until:
                    raise RuntimeError('等待升降完成超时')
            elif self.diagnostic['lift_inhibited']:
                raise RuntimeError('升降机构被其他动作占用')

            if index == len(samples) and not pending and still_since is not None:
                self.check_pose(actual, sample['pose'], STOP_ERROR)
                print(f'回放完成，共 {len(samples)} 个采样点。', flush=True)
                return

            blocked = bool(pending and (command != ZERO or target is not None))
            if target is not None and not pending:
                self.check_pose(actual, sample['pose'], STOP_ERROR)
                blocked = still_since is None or now - still_since < 0.25
                if not blocked:
                    self.lift_pub.publish(UInt8(data=target))
                    lifter_timeout = self.diagnostic.get('lifter_timeout_ms', 5000)
                    pending = self.diagnostic, target, now + lifter_timeout / 1000 + 1.4
                    print('抬升' if target else '放下', flush=True)

            if blocked or index == len(samples):
                blocked_since = now if blocked_since is None else blocked_since
                if not pending and now - blocked_since > 2.0:
                    raise RuntimeError('等待底盘停稳超时')
            else:
                blocked_since = None
                index += 1
            self.publish(ZERO if pending or blocked else
                         correction(actual, sample['pose'], command, self.max_linear, self.max_angular))
            deadline = next_tick(deadline, now)

    def stop(self):
        if self.active and self.graph_ok([self.owner]):
            self.publish(ZERO)
            self.cmd_pub.wait_for_all_acked(Duration(seconds=0.15))


def load_recording(path):
    data = json.loads(path.read_text())
    if data['format'] != 1 or data['rate_hz'] != 20:
        raise ValueError('录制文件格式或采样率不匹配')
    samples = data['samples']
    max_linear, max_angular = limits()
    last = -math.inf
    for row in samples:
        t, command, pose, lift = row['t'], row['cmd'], row['pose'], row['lift']
        if (len(command) != 2 or len(pose) != 3 or not all(math.isfinite(v) for v in (t, *command, *pose))
                or t < 0 or t <= last or abs(command[0]) > max_linear + 1e-9
                or abs(command[1]) > max_angular + 1e-9
                or (lift is not None and (type(lift) is not int or lift not in (0, 1) or tuple(command) != ZERO))):
            raise ValueError('录制文件含无效时间、位姿、速度或升降指令')
        last = t
    if (len(samples) < 2 or tuple(samples[0]['cmd']) != ZERO or tuple(samples[-1]['cmd']) != ZERO
            or data['diagnostics'][0]['lifter_position'] not in (-1, 0, 1)):
        raise ValueError('录制文件缺少静止首尾或升降初态')
    first, last = data['diagnostics'][0], data['diagnostics'][-1]
    targets = [sample['lift'] for sample in samples if sample['lift'] is not None]
    if (last['lifter_rx'] - first['lifter_rx'] != len(targets)
            or last['lifter_rejected'] != first['lifter_rejected']
            or (targets and last['lifter_position'] != targets[-1])):
        raise ValueError('录制中的升降指令缺少完整回执或有指令被拒绝')
    return data


def save_recording(path, recording):
    # Link the complete file into place without replacing any existing recording.
    with tempfile.NamedTemporaryFile('w', dir=path.parent) as output:
        json.dump(recording, output, allow_nan=False, separators=(',', ':'))
        output.flush()
        path.hardlink_to(output.name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('record', 'replay'))
    parser.add_argument('file', type=Path)
    args = parser.parse_args()
    recording_mode = args.operation == 'record'
    try:
        if recording_mode:
            if args.file.exists():
                raise FileExistsError(args.file)
            args.file.parent.mkdir(parents=True, exist_ok=True)
        else:
            recording = load_recording(args.file)
    except (OSError, ValueError, KeyError, IndexError, TypeError) as exc:
        parser.exit(1, f'{exc}\n')

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Recorder() if recording_mode else Player()
    interrupted = False

    def interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    def wait(deadline):
        # Use a monotonic wall clock for scheduling and input watchdogs.
        while True:
            if interrupted and (not recording_mode or node.origin is None):
                raise KeyboardInterrupt
            now = time.monotonic()
            if now >= deadline:
                return now
            rclpy.spin_once(node, timeout_sec=min(0.02, deadline - now))

    signals = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        if recording_mode:
            def was_interrupted():
                return interrupted

            recording = node.run(wait, was_interrupted)
            save_recording(args.file, recording)
            print(f'已保存 {len(recording["samples"])} 个采样点至 {args.file}', flush=True)
        else:
            node.run(recording, wait)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'已中止（{exc}）', flush=True)
        return 1
    finally:
        if not recording_mode:
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in signals.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
