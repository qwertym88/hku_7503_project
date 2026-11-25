"""Check the installed robot's sensor streams and exercise two supervised actions."""

import argparse
from contextlib import nullcontext
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String, UInt8

from .chassis_segment import PERIOD, Segment, make_action
from .motion import motion_request
from .route import relative


DISTANCE = 0.20
SPEED = 0.06
HARDWARE_CAMERA_SOURCES = frozenset(('rpicam', 'rpicam-vid', 'rpicamvid', 'picamera2', 'cv', 'webcam', 'usb'))
LOCKED_MESSAGE = '底盘重连后已锁定。停止控制程序并确认现场安全后，调用 /chassis/unlock'
CONTROL = dict(position_tolerance_m=0.010, max_cross_track_m=0.030,
               max_heading_error_rad=0.17, turn_rps=0.25, acceleration_mps2=0.20,
               angular_acceleration_rps2=0.60, segment_timeout_s=12.0, no_progress_s=2.0,
               stopped_linear_mps=0.01, stopped_angular_rps=0.02, settle_s=0.30)


def odom_data(msg):
    pos, quat, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
    angles = (quat.x, quat.y, quat.z, quat.w)
    velocity = (twist.linear.x, twist.angular.z)
    if ((msg.header.frame_id, msg.child_frame_id) != ('odom', 'base_footprint')
            or not all(map(math.isfinite, (pos.x, pos.y, *angles, *velocity)))
            or abs(sum(value**2 for value in angles) - 1.0) > 0.05):
        raise ValueError('无效的里程计坐标系、位姿或速度')
    yaw = math.atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y**2 + quat.z**2))
    return (pos.x, pos.y, yaw), velocity


def imu_data(msg):
    gyro, accel = msg.angular_velocity, msg.linear_acceleration
    norm = math.hypot(accel.x, accel.y, accel.z)
    if msg.header.frame_id != 'imu_link' or not all(map(math.isfinite, (
        gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z,
    ))) or not 5.0 <= norm <= 15.0:
        raise ValueError('IMU 坐标系或静置加速度异常')
    return {'accel_norm_mps2': norm, 'gyro_z_radps': gyro.z}


def scan_data(msg):
    valid = sum(math.isfinite(distance) and msg.range_min <= distance <= msg.range_max
                for distance in msg.ranges)
    if msg.header.frame_id != 'laser' or not valid or not math.isfinite(msg.angle_increment) or not msg.angle_increment:
        raise ValueError('雷达坐标系、扫描角度或有效测距异常')
    return {'valid_ranges': valid, 'rays': len(msg.ranges)}


def image_data(msg):
    if (msg.encoding != 'bgr8' or not msg.width or not msg.height
            or msg.step < msg.width * 3 or len(msg.data) != msg.step * msg.height):
        raise ValueError('相机 bgr8 图像尺寸或数据长度异常')
    return {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding}


def diagnostics_data(msg):
    data = json.loads(msg.data)
    integers = ('cycles', 'rx', 'rejected', 'enabled', 'imu_valid', 'lift_inhibited', 'lifter_position',
                'lifter_rx', 'lifter_rejected', 'lifter_completed', 'lifter_timeout_ms')
    strings = ('lifter_state', 'lifter_fault', 'lifter_last_result')
    if (not isinstance(data, dict) or not all(type(data.get(key)) is int for key in integers)
            or not all(isinstance(data.get(key), str) for key in strings)):
        raise ValueError('底盘诊断缺少当前固件所需字段')
    return data


@dataclass
class Stream:
    topic: str
    max_age: float
    stamp: int = -1
    seen_at: float = -math.inf
    samples: int = 0
    data: object = None
    problem: str = '未收到数据'

    def record(self, sequence, data):
        if sequence <= self.stamp:
            raise ValueError('时间戳或计数未递增')
        self.stamp, self.seen_at, self.data = sequence, time.monotonic(), data
        self.samples += 1
        self.problem = ''

    def issue(self):
        if self.problem:
            return f'{self.topic} {self.problem}'
        if self.samples < 2 or time.monotonic() - self.seen_at > self.max_age:
            return f'{self.topic} 缺少连续的新鲜数据'
        return ''


class SelfCheck(Node):
    def __init__(self):
        super().__init__('hardware_self_check')
        self.odom = self.watch('/odom', Odometry, odom_data, 0.4)
        self.diagnostics = self.watch('/chassis/diagnostics', String, diagnostics_data, 1.2)
        self.imu = self.watch('/imu/data', Imu, imu_data, 0.4)
        self.scan = self.watch('/scan', LaserScan, scan_data, 0.75)
        self.image = self.watch('/camera/image_raw', Image, image_data, 1.5)
        self.camera_params = self.create_client(GetParameters, '/qr_perception/get_parameters')

    def watch(self, topic, msg_type, validate, max_age):
        stream = Stream(topic, max_age)

        def receive(msg):
            try:
                data = validate(msg)
                if msg_type is String:
                    sequence = data['cycles']
                else:
                    stamp = msg.header.stamp
                    sequence = stamp.sec * 1_000_000_000 + stamp.nanosec
                    age = (self.get_clock().now().nanoseconds - sequence) / 1e9
                    if sequence <= 0 or not -0.2 <= age <= max_age:
                        raise ValueError('消息时间戳过期或时钟不同步')
                stream.record(sequence, data)
            except ValueError as exc:
                stream.problem = str(exc)
                stream.samples = 0

        self.create_subscription(msg_type, topic, receive, qos_profile_sensor_data)
        return stream

    def spin_until(self, deadline):
        while (remaining := deadline - time.monotonic()) > 0:
            rclpy.spin_once(self, timeout_sec=min(0.02, remaining))

    def publishers(self, topic):
        return [(info.node_namespace, info.node_name) for info in self.get_publishers_info_by_topic(topic)]

    def chassis_subscriber(self, topic):
        endpoints = self.get_subscriptions_info_by_topic(topic)
        return sum(info.node_namespace == '/' and info.node_name == 'chassis' for info in endpoints) == 1

    def chassis_issue(self):
        if issue := self.odom.issue() or self.diagnostics.issue():
            return issue
        for stream in (self.odom, self.diagnostics):
            if self.publishers(stream.topic) != [('/', 'chassis')]:
                return f'{stream.topic} 需要由 /chassis 独占发布'
        for topic in ('/cmd_vel', '/lifter'):
            if not self.chassis_subscriber(topic):
                return f'{topic} 需要唯一的底盘 /chassis 订阅端'
        return ''

    def imu_issue(self):
        if issue := self.imu.issue():
            return issue
        if (self.diagnostics.issue() or self.diagnostics.data['imu_valid'] != 1
                or self.publishers(self.imu.topic) != [('/', 'chassis')]):
            return '需要 /chassis 发布 IMU 并报告 imu_valid=1'
        return ''

    def radar_issue(self):
        if issue := self.scan.issue():
            return issue
        if self.publishers(self.scan.topic) != [('/', 'ms200')]:
            return '/scan 需要由真机驱动 /ms200 独占发布'
        return ''

    def camera_issue(self, source):
        if issue := self.image.issue():
            return issue
        if self.publishers(self.image.topic) != [('/', 'qr_perception')]:
            return f'{self.image.topic} 需要由 /qr_perception 独占发布'
        if not source:
            return '已收到图像，但未能从 /qr_perception/get_parameters 读到 source'
        if source not in HARDWARE_CAMERA_SOURCES:
            return f'已收到图像，但 source={source} 不是自检接受的真机帧源'
        return ''

    def passive(self):
        future = None
        source = None
        deadline = time.monotonic() + 8.0
        while True:
            if future is None and self.camera_params.service_is_ready():
                future = self.camera_params.call_async(GetParameters.Request(names=['source']))
            if future is not None and future.done() and future.exception() is None:
                values = future.result().values
                source = values[0].string_value if values else None
            issues = {'底层 ROS 通信': self.chassis_issue(), 'IMU': self.imu_issue(),
                      '雷达': self.radar_issue(), '相机': self.camera_issue(source)}
            if not any(issues.values()) or time.monotonic() >= deadline:
                return issues, {'imu': self.imu.data, 'radar': self.scan.data,
                                'camera': self.image.data, 'camera_source': source,
                                'command_locked': bool((self.diagnostics.data or {}).get('command_locked'))}
            rclpy.spin_once(self, timeout_sec=0.10)

    def stopped(self):
        vx, wz = self.odom.data[1]
        return (not self.diagnostics.data['enabled'] and abs(vx) <= CONTROL['stopped_linear_mps']
                and abs(wz) <= CONTROL['stopped_angular_rps'])

    def check_publishers(self, owned_topic):
        for topic in ('/cmd_vel', '/lifter'):
            expected = [('/', self.get_name())] if topic == owned_topic else []
            if self.publishers(topic) != expected:
                raise RuntimeError(f'{topic} 发布者有冲突，退出任务节点、遥控和其他测试工具后重试')

    def check_active(self, owned_topic):
        if issue := self.chassis_issue() or self.imu_issue() or self.radar_issue():
            raise RuntimeError(issue)
        self.check_publishers(owned_topic)
        if self.diagnostics.data.get('command_locked'):
            raise RuntimeError(LOCKED_MESSAGE)
        if self.diagnostics.data['lifter_fault'] != 'none':
            raise RuntimeError(f"升降机构故障 {self.diagnostics.data['lifter_fault']}")

    def acquire(self, pub, topic):
        started = time.monotonic()
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            issue = self.chassis_issue() or self.imu_issue() or self.radar_issue()
            elapsed = time.monotonic() - started
            if (elapsed >= 0.65 and not issue and pub.get_subscription_count()
                    and self.diagnostics.seen_at >= started):
                break
            if elapsed >= 3.0:
                raise RuntimeError(issue or f'{topic} 未匹配到底盘订阅端或未收到新诊断')
        self.check_active(topic)
        if not self.stopped() or self.diagnostics.data['lifter_state'] != 'idle':
            raise RuntimeError('底盘或升降机构尚未停稳')

    def drive(self):
        action = make_action('drive', DISTANCE, SPEED, CONTROL)
        segment = Segment(action, CONTROL)
        self.check_publishers(None)
        pub = self.create_publisher(Twist, '/cmd_vel', 1)
        sent = False
        try:
            self.acquire(pub, '/cmd_vel')
            baseline = self.diagnostics.data
            if baseline['lift_inhibited'] or baseline['lifter_position'] not in (-1, 0):
                raise RuntimeError('前进要求机构处于低位，且驱动未被互锁')
            origin = self.odom.data[0]
            started = deadline = time.monotonic()
            try:
                while not segment.complete:
                    self.spin_until(deadline)
                    now = time.monotonic()
                    if now - deadline > PERIOD:
                        raise RuntimeError('前进控制周期超时')
                    self.check_active('/cmd_vel')
                    diag = self.diagnostics.data
                    if (diag['lift_inhibited'] or diag['lifter_state'] != 'idle'
                            or diag['lifter_rx'] != baseline['lifter_rx'] or diag['rejected'] != baseline['rejected']):
                        raise RuntimeError('前进期间出现互锁、升降指令或底盘拒绝指令')
                    pose, velocity = self.odom.data
                    offset = relative(origin, pose)
                    if offset[0] < -CONTROL['position_tolerance_m']:
                        raise RuntimeError('里程计检测到实际运动方向与前进指令相反')
                    vx, wz = segment.tick(now - started, offset, velocity)
                    command = Twist()
                    command.linear.x, command.angular.z = vx, wz
                    sent = True
                    pub.publish(command)
                    deadline += PERIOD
            finally:
                if sent:
                    for _ in range(20):
                        pub.publish(Twist())
                        self.spin_until(time.monotonic() + PERIOD)
            self.check_active('/cmd_vel')
            final = relative(origin, self.odom.data[0])
            _, reached, _, fault = motion_request(action, final, CONTROL)
            if (not self.stopped() or not reached or fault
                    or self.diagnostics.data['rx'] <= baseline['rx']):
                raise RuntimeError(f'零速后未确认到点、停稳或板端收令，最终里程计 {final!r}')
            return {'target_m': DISTANCE, 'odom_forward_m': final[0], 'odom_lateral_m': final[1],
                    'odom_yaw_rad': final[2], 'elapsed_s': time.monotonic() - started}
        finally:
            self.destroy_publisher(pub)

    def lift_to(self, target):
        self.check_publishers(None)
        pub = self.create_publisher(UInt8, '/lifter', 1)
        try:
            self.acquire(pub, '/lifter')
            baseline = self.diagnostics.data
            start_positions = (-1, 0) if target == 1 else (1,)
            if baseline['lift_inhibited'] or baseline['lifter_position'] not in start_positions:
                raise RuntimeError('机构的诊断位置与本次行程的起始位不符，或互锁未解除')
            if not 0 < baseline['lifter_timeout_ms'] <= 30000:
                raise RuntimeError('固件升降超时参数无效')
            deadline = time.monotonic() + baseline['lifter_timeout_ms'] / 1000 + 1.2
            pub.publish(UInt8(data=target))
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                self.check_active('/lifter')
                if not self.stopped():
                    raise RuntimeError('升降期间检测到底盘运动')
                diag = self.diagnostics.data
                if diag['lifter_rx'] == baseline['lifter_rx']:
                    continue
                if (diag['lifter_rx'] != baseline['lifter_rx'] + 1
                        or diag['lifter_rejected'] != baseline['lifter_rejected']
                        or diag['lifter_last_result'] != 'accepted'):
                    raise RuntimeError(f"升降指令回执异常 {diag['lifter_last_result']}")
                if (diag['lifter_completed'] == baseline['lifter_completed'] + 1
                        and diag['lifter_state'] == 'idle' and diag['lifter_position'] == target
                        and not diag['lift_inhibited']):
                    return {'firmware_position': target, 'command_result': 'accepted'}
            raise RuntimeError('等待升降回执或完成超时')
        finally:
            self.destroy_publisher(pub)


def confirm(message, token):
    try:
        return input(f'{message}\n输入 {token} 确认，直接回车跳过或停止本项 > ').strip() == token
    except EOFError:
        return False


def run_checks(node, observe_only, report):
    def record(item, status, detail):
        report[item] = {'status': status, 'detail': detail}
        print(f'[{status}] {item}  {detail}', flush=True)

    issues, samples = node.passive()
    for item, issue in issues.items():
        record(item, 'FAIL' if issue else 'PASS', issue or '已收到连续有效数据或发现所需 ROS 端点')
    report['sample_details'] = samples
    skip_reason = None
    if samples.get('command_locked'):
        print(LOCKED_MESSAGE, flush=True)
        skip_reason = '底盘指令仍锁定'
    elif any(issues.values()):
        skip_reason = '有被动检查未通过'
    elif observe_only:
        skip_reason = '仅检查传感器与通信'
    if skip_reason:
        for item in ('前进 0.20 m', '升降往返'):
            record(item, 'SKIP', skip_reason)
        return

    drive = '前进 0.20 m'
    lift = '升降往返'
    if confirm('将空载车放在平坦地面，确认机构在低位、车前至少 1 m 内无人和障碍，可随时急停。', 'DRIVE'):
        record(drive, 'RUNNING', '速度上限 0.06 m/s')
        try:
            detail = node.drive()
        except RuntimeError as exc:
            record(drive, 'FAIL', str(exc))
            record(lift, 'SKIP', '前进失败，请检查底盘后重试')
            return
        print(f"里程计前进 {detail['odom_forward_m'] * 100:.1f} cm。", flush=True)
        if not confirm('目视确认车辆正常前进并完全停稳，地面量距请另记。', 'STOPPED'):
            record(drive, 'FAIL', {**detail, 'visual_confirmation': False})
            record(lift, 'SKIP', '实际停车未确认')
            return
        record(drive, 'PASS', {**detail, 'visual_confirmation': True})
    else:
        record(drive, 'SKIP', '操作员未授权前进')

    if not confirm('确认底盘已停稳，举升机构空载且在低位，手和物品已离开升降行程。', 'LIFT'):
        record(lift, 'SKIP', '操作员未授权抬升')
        return
    record(lift, 'RUNNING', '等待抬升回执')
    try:
        raised = node.lift_to(1)
        record(lift, 'RUNNING', {'raise': raised, 'phase': '等待观察抬升并确认下降'})
        if not confirm('目视确认已抬起并停住，下降行程已清空。', 'LOWER'):
            record(lift, 'FAIL', {'raise': raised, 'reason': '下降未确认，机构可能保持在高位'})
            return
        lowered = node.lift_to(0)
        seen = confirm('目视确认已完全回到低位。', 'LOW')
        record(lift, 'PASS' if seen else 'FAIL', {'raise': raised, 'lower': lowered, 'visible_low': seen})
    except RuntimeError as exc:
        record(lift, 'FAIL', f'{exc}。请确认机构实际位置并现场处置。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe-only', action='store_true', help='仅检查雷达、相机、ROS 通信和 IMU')
    parser.add_argument('--output', type=Path, help='可选 JSON 报告路径，拒绝覆盖已有文件')
    args = parser.parse_args()
    if not args.observe_only and not sys.stdin.isatty():
        parser.error('动作测试需要交互终端，非交互运行请使用 --observe-only')
    try:
        report_output = args.output.open('x', encoding='utf-8') if args.output else nullcontext()
    except OSError as exc:
        parser.error(f'无法创建报告文件：{exc}')
    with report_output as output:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = None
        report = {'started_at': datetime.now().astimezone().isoformat(), 'observe_only': args.observe_only}

        def terminate(_signum, _frame):
            raise KeyboardInterrupt

        signal.signal(signal.SIGTERM, terminate)
        try:
            node = SelfCheck()
            run_checks(node, args.observe_only, report)
        except KeyboardInterrupt:
            print('\n自检已中断，请确认车辆停稳和升降机构位置。', flush=True)
            report['interrupted'] = True
            return 130
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
            if output is not None:
                json.dump(report, output, ensure_ascii=False, indent=2, allow_nan=False)
                output.write('\n')
        results = [item['status'] for item in report.values() if isinstance(item, dict) and 'status' in item]
        if 'FAIL' in results:
            return 1
        return 2 if 'SKIP' in results and not args.observe_only else 0
