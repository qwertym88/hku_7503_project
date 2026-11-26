"""ROS mission adapter and Gazebo cargo backend."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import time

import cv2
import numpy as np
import rclpy
from qr_perception.core import QrDecoder
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .core import Mission
from .field_map import rasterize
from .route import action_samples, load_config


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def matches_qr(text, label):
    if label in ('START', 'END'):
        return text == label
    prefix = f'RACK{label}_'
    suffix = text[len(prefix):]
    return text.startswith(prefix) and suffix.isascii() and suffix.isalnum()


def decode_expected_qr(decoder, pixels, label, stamp):
    width = pixels.shape[1]
    # The route aims the camera at the target. A central crop can isolate it
    # when another rack is visible or the full-frame detector misses it.
    central = pixels[:, width // 4:3 * width // 4]
    gray = cv2.cvtColor(central, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    contrast = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    for region in (pixels, central, contrast):
        for result in decoder.decode(region, stamp):
            if matches_qr(result.text, label):
                return result.text
    return None


def capture_image(msg, filename, decoder, label):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if msg.encoding not in channels:
        raise ValueError(f'Unsupported image encoding {msg.encoding}')
    count = channels[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * count:
        raise ValueError('Invalid image dimensions/step')
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width*count].reshape(msg.height, msg.width, count)
    if msg.encoding == 'rgb8':
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'rgba8':
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    elif msg.encoding == 'bgra8':
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    text = None
    if decoder is not None:
        if count == 1:
            pixels = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
        text = decode_expected_qr(decoder, pixels, label, stamp_seconds(msg))
        if text is None:
            return None
    if not cv2.imwrite(str(filename), pixels):
        raise OSError(f'Could not write {filename}')
    return str(filename), text


class MissionNode(Node):
    simulation = True
    encoder_only = False

    def __init__(self):
        super().__init__('fixed_route_mission')
        self.declare_parameter('config', get_package_share_directory('mission_control') + '/config/fixed_route_sim.yaml')
        self.declare_parameter('output_dir', 'runtime_logs/fixed_route')
        self.declare_parameter('autostart', False)
        self.declare_parameter('feedback_topic', '/odom')
        self.declare_parameter('require_localization', False)
        if self.get_parameter('use_sim_time').value != self.simulation:
            raise ValueError('use_sim_time must match the mission backend')
        self.config_path = Path(self.get_parameter('config').value).resolve()
        self.cfg = load_config(self.config_path)
        if self.cfg['simulation_only'] != self.simulation:
            raise ValueError('Route configuration does not match the mission backend')
        self.mission = Mission(self.cfg)
        self.qr_decoder = QrDecoder(deep_every_n=1) if 'marker_scans' in self.cfg else None
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.require_localization = self.get_parameter('require_localization').value
        if not self.simulation and not self.encoder_only and not self.require_localization:
            raise ValueError('Hardware requires AMCL')
        if self.encoder_only and (self.require_localization or self.feedback_topic != '/odom'):
            raise ValueError('Encoder mode requires raw /odom with AMCL disabled')
        self.route_frame = 'map' if self.require_localization or self.encoder_only else 'odom'
        self.p = None
        self.odom_pose = None
        self.velocity = (0., 0.)
        self.feedback = {}
        self.localization = None
        self.localization_after = 0.
        self.cargo_status = None
        self.image = None
        self.camera_job = None
        self.cargo_job = None
        self.effect_id = None
        self.photo_requested_stamp = None
        self.photo_requested_wall = None
        self.photo_stamp = None
        self.log_file = None
        self.run_dir = None
        self.reset_job = None
        self.reset_stage = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mission_photo')
        self.birth = time.monotonic()
        self.autostart_attempted = False
        self.last_status = 0.

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(UInt8, '/lifter', 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/mission/status', retained)
        self.map_pub = self.create_publisher(OccupancyGrid, '/mission/scene_map', retained)
        self.path_pub = self.create_publisher(NavPath, '/mission/planned_path', retained)
        self.active_path_pub = self.create_publisher(NavPath, '/mission/active_path', retained)
        self.displayed_racks = None
        self.displayed_action = None
        self.path_pub.publish(self.route_path(self.mission.actions, height=0.025))
        self.create_subscription(Odometry, self.feedback_topic, self.on_odom, qos_profile_sensor_data)
        if self.require_localization:
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_localization, retained)
        if not self.encoder_only:
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.setup_cargo()
        self.create_service(Trigger, '/mission/start', self.start_service)
        self.create_service(Trigger, '/mission/cancel', self.cancel_service)
        self.create_service(Trigger, '/mission/reset', self.reset_service)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Mission loaded: {len(self.mission.actions)} actions; config={self.config_path}')

    def setup_cargo(self):
        self.create_subscription(String, '/sim/cargo/status', self.on_cargo, 10)
        self.cargo_clients = {(key, kind): self.create_client(Trigger, f'/sim/cargo/{key}/{kind}')
                              for key in self.cfg['order'] for kind in ('lift','lower')}
        self.reset_cargo_client = self.create_client(Trigger, '/sim/cargo/reset')
        self.reset_base_client = self.create_client(Trigger, '/sim/reset_base')

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def zero(self):
        self.cmd_pub.publish(Twist())

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        values = [p.x, p.y, q.x, q.y, q.z, q.w, msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1) > 0.05:
            self.feedback.pop('odom', None)
            return
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
            self.feedback.pop('odom', None)
            return
        self.odom_pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if self.simulation:
            self.p = self.odom_pose
        self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.feedback['odom'] = (time.monotonic(), stamp_seconds(msg))

    def on_scan(self, msg):
        if msg.ranges and msg.header.frame_id == 'laser' and any(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges):
            self.feedback['scan'] = (time.monotonic(), stamp_seconds(msg))

    def on_localization(self, msg):
        pose = msg.pose.pose
        values = [pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w]
        if (msg.header.frame_id != 'map' or stamp_seconds(msg) <= self.localization_after
                or not all(math.isfinite(value) for value in values)):
            return
        self.localization = msg

    def localization_ready(self):
        if not self.require_localization:
            return True
        return self.map_transform() is not None

    def map_transform(self):
        if self.simulation and self.localization is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            return None
        # AMCL postdates the transform by its tolerance, including between pose updates.
        age = self.ros_seconds() - stamp_seconds(transform)
        return transform if -2.0 <= age <= self.cfg['control']['scan_timeout_s'] else None

    def reset_localization(self):
        self.localization = None
        self.localization_after = self.ros_seconds()
        self.initial_pose_pub.publish(self.initial_pose_message(0.01))

    def initial_pose_message(self, variance):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        start = self.cfg['start_pose']
        msg.pose.pose.position.x = float(start['x'])
        msg.pose.pose.position.y = float(start['y'])
        msg.pose.pose.orientation.z = math.sin(start['yaw'] / 2)
        msg.pose.pose.orientation.w = math.cos(start['yaw'] / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = variance
        return msg

    def on_image(self, msg):
        if msg.width and msg.height and msg.data:
            self.image = msg
            self.feedback['image'] = (time.monotonic(), stamp_seconds(msg))

    def on_cargo(self, msg):
        try:
            self.cargo_status = json.loads(msg.data)
            self.feedback['cargo'] = (time.monotonic(), self.ros_seconds())
        except (ValueError, TypeError):
            self.cargo_status = None

    def fresh(self, name, limit):
        receipt, stamp = self.feedback.get(name, (-math.inf, -math.inf))
        age = self.ros_seconds() - stamp
        return time.monotonic()-receipt <= limit and -0.1 <= age <= limit

    def health(self, precheck=False):
        return self.feedback_health(precheck) or self.cargo_health(precheck)

    def feedback_health(self, precheck):
        c = self.cfg['control']
        if not self.fresh('odom', c['feedback_timeout_s']):
            return 'odom missing/stale'
        if not self.encoder_only and not self.fresh('scan', c['scan_timeout_s']):
            return 'scan missing/stale'
        if self.require_localization and not self.localization_ready():
            return 'Waiting for AMCL map-to-odom transform'
        if self.count_publishers('/cmd_vel') != 1:
            return '/cmd_vel must have exactly one publisher; close teleop/other controllers'
        if precheck:
            if not self.fresh('image', c['image_timeout_s']):
                return 'Camera missing/stale'
            if any(abs(v) > lim for v, lim in zip(self.velocity, (c['stopped_linear_mps'],c['stopped_angular_rps']))):
                return 'Robot must be stationary'

    def cargo_health(self, precheck):
        if not self.fresh('cargo', self.cfg['control']['cargo_timeout_s']):
            return 'cargo missing/stale'
        if not self.cargo_status or not self.cargo_status.get('healthy'):
            return 'Gazebo cargo adapter not ready/healthy'
        if precheck:
            if self.cargo_status.get('carrying') or self.cargo_status.get('placed'):
                return 'Cargo scene must be reset before starting'
            if not all(client.service_is_ready() for client in self.cargo_clients.values()):
                return 'Cargo services not ready'

    def begin(self):
        if self.reset_stage or self.mission.state != 'IDLE':
            return False, 'Not idle; use /mission/reset and wait for IDLE'
        fault = self.health(precheck=True)
        if fault:
            return False, fault
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        directory = Path(self.get_parameter('output_dir').value).expanduser().resolve() / run_id
        try:
            directory.mkdir(parents=True)
            shutil.copyfile(self.config_path, directory / 'config.yaml')
            log_file = (directory / 'events.jsonl').open('w', encoding='utf-8', buffering=1)
        except OSError as exc:
            return False, f'Cannot create run evidence: {exc}'
        ok, message = self.mission.start(time.monotonic(), self.p)
        if not ok:
            log_file.close()
            return ok, message
        if self.log_file:
            self.log_file.close()
        self.run_dir, self.log_file = directory, log_file
        self.effect_id = None
        self.get_logger().info(f'Started; evidence: {directory}')
        return True, f'Started; evidence: {directory}'

    def start_service(self, request, response):
        response.success, response.message = self.begin()
        return response

    def cancel_service(self, request, response):
        self.mission.stop('operator cancel')
        self.zero()
        response.success, response.message = True, 'Zero speed; cargo remains supported. Reset scene before restart.'
        return response

    def reset_service(self, request, response):
        if self.mission.state == 'RUNNING' or self.reset_stage:
            response.success, response.message = False, 'Cancel first / reset already in progress'
        elif any(job is not None and not job.done() for job in (self.cargo_job,self.camera_job)):
            response.success, response.message = False, 'Effect still completing; wait before resetting scene'
        elif not self.reset_cargo_client.service_is_ready() or not self.reset_base_client.service_is_ready():
            response.success, response.message = False, 'Simulation reset services unavailable'
        else:
            self.zero()
            self.reset_stage = 'cargo'
            self.reset_started = time.monotonic()
            self.reset_job = self.reset_cargo_client.call_async(Trigger.Request())
            response.success, response.message = True, 'Reset accepted; wait for /mission/status IDLE and reset_pending=false'
        return response

    def poll_reset(self):
        self.zero()
        if time.monotonic()-self.reset_started > 10:
            self.reset_job.cancel()
            self.get_logger().error('Scene reset timed out; relaunch simulation')
            self.mission.state, self.mission.reason = 'STOPPED', 'scene reset timeout'
            self.reset_stage = None
        elif self.reset_stage == 'localization':
            if self.localization_ready():
                self.finish_reset()
        elif self.reset_job.done():
            try:
                result = self.reset_job.result()
                if not result.success:
                    raise RuntimeError(result.message)
                if self.reset_stage == 'cargo':
                    self.reset_stage = 'base'
                    self.reset_job = self.reset_base_client.call_async(Trigger.Request())
                elif self.require_localization:
                    self.reset_stage = 'localization'
                    self.reset_localization()
                else:
                    self.finish_reset()
            except Exception as exc:
                self.mission.state, self.mission.reason = 'STOPPED', f'scene reset failed: {exc}'
                self.get_logger().error(self.mission.reason)
                self.reset_stage = None

    def finish_reset(self):
        self.mission.reset()
        self.effect_id = None
        self.reset_stage = None
        self.get_logger().info('Scene reset completed; ready for a new start')

    def start_cargo_effect(self, action):
        self.lift_pub.publish(UInt8(data=1 if action.kind == 'lift' else 0))
        client = self.cargo_clients[(action.rack, action.kind)]
        if client.service_is_ready():
            self.cargo_job = client.call_async(Trigger.Request())
        else:
            self.mission.acknowledge(action.id, False, 'cargo service unavailable')

    def poll_cargo_effect(self, action):
        if self.cargo_job and self.cargo_job.done():
            try:
                result = self.cargo_job.result()
                self.mission.acknowledge(action.id, result.success, result.message)
            except Exception as exc:
                self.mission.acknowledge(action.id, False, f'cargo service failed: {exc}')

    def handle_effect(self):
        a = self.mission.action
        if a.kind not in ('photo', 'lift', 'lower'):
            return
        if self.effect_id != a.id:
            self.effect_id = a.id
            self.camera_job = self.cargo_job = None
            if a.kind == 'photo':
                self.photo_requested_stamp = self.ros_seconds()
                self.photo_requested_wall = time.monotonic()
                self.photo_stamp = self.photo_requested_stamp
                self.mission.emit('photo_requested', id=a.id, requested_ros_s=self.photo_requested_stamp,
                                  first_frame_timeout_s=self.photo_first_frame_timeout(), photo_timeout_s=a.timeout)
            else:
                self.start_cargo_effect(a)
        if a.kind == 'photo':
            self.poll_photo(a)
        elif not self.mission.effect_done:
            self.poll_cargo_effect(a)

    def photo_first_frame_timeout(self):
        control = self.cfg['control']
        return control.get('photo_first_frame_timeout_s', control['image_timeout_s'])

    def report_photo_saved(self, action_id, filename, qr_text, **details):
        self.mission.emit('photo_saved', id=action_id, path=filename, qr=qr_text, **details)
        if qr_text is not None:
            banner = '=' * 64
            self.get_logger().info(f'\n{banner}\n  QR 扫描成功  {qr_text}\n  图片已保存  {filename}\n{banner}')

    def poll_photo(self, action, accept_new_frame=True):
        if self.mission.effect_done:
            return
        if self.camera_job is not None:
            if not self.camera_job.done():
                return
            try:
                capture = self.camera_job.result()
            except Exception as exc:
                self.mission.acknowledge(action.id, False, f'camera capture failed: {exc}')
                return
            self.camera_job = None
            if capture is not None:
                filename, qr_text = capture
                self.report_photo_saved(action.id, filename, qr_text, pose=list(self.p),
                                        requested_ros_s=self.photo_requested_stamp, image_ros_s=self.photo_stamp,
                                        elapsed_s=time.monotonic() - self.photo_requested_wall)
                self.mission.acknowledge(action.id, True, filename)
                return
        if not accept_new_frame:
            return
        first_frame = self.photo_stamp == self.photo_requested_stamp
        limit = self.cfg['control']['image_timeout_s']
        new_frame = (self.image is not None and self.fresh('image', limit)
                     and stamp_seconds(self.image) > self.photo_stamp)
        if first_frame:
            first_frame_timeout = self.photo_first_frame_timeout()
            deadline = self.photo_requested_wall + first_frame_timeout
            received = self.feedback['image'][0] if new_frame else None
            if (received is not None and self.photo_requested_wall <= received
                    and (received <= deadline or self.mission.skip_qr_timeout)):
                self.mission.emit('photo_first_frame', id=action.id, image_ros_s=stamp_seconds(self.image),
                                  elapsed_s=received - self.photo_requested_wall)
            else:
                if time.monotonic() >= deadline and not self.mission.skip_qr_timeout:
                    self.mission.acknowledge(action.id, False,
                                             f'no new camera frame at photo pose within {first_frame_timeout:g} s')
                return
        if new_frame:
            filename = self.run_dir / f'{action.rack}_photo.png'
            self.photo_stamp = stamp_seconds(self.image)
            self.camera_job = self.pool.submit(capture_image, self.image, filename, self.qr_decoder, action.rack)

    def flush_events(self):
        events, self.mission.events = self.mission.events, []
        for event in events:
            event.update(wall_monotonic_s=time.monotonic(), ros_time_s=self.ros_seconds())
            if self.log_file:
                self.log_file.write(json.dumps(event, ensure_ascii=False) + '\n')
            if event['event'] in ('action_enter', 'stopped', 'done', 'localization_paused', 'localization_resumed',
                                  'photo_requested', 'photo_first_frame', 'photo_saved', 'photo_skipped'):
                self.get_logger().info(json.dumps(event, ensure_ascii=False))
            if event['event'] in ('done', 'stopped') and self.run_dir:
                (self.run_dir / 'result.json').write_text(json.dumps(event, ensure_ascii=False, indent=2), encoding='utf-8')

    def route_path(self, actions, height):
        msg = NavPath()
        msg.header.frame_id = self.route_frame
        if actions:
            positions = [actions[0].start]
            for action in actions:
                if action.kind == 'curve':
                    positions.extend(action_samples(action, linear_step=0.02, angular_step=0.08))
                else:
                    positions.append(action.target)
            for x, y, yaw in positions:
                p = PoseStamped()
                p.header = msg.header
                p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, height
                p.pose.orientation.z = math.sin(yaw / 2)
                p.pose.orientation.w = math.cos(yaw / 2)
                msg.poses.append(p)
        return msg

    def publish_visualization(self):
        mission = self.mission
        racks = {key: p for key, p in mission.racks.items() if key != mission.carrying}
        if racks != self.displayed_racks:
            field = rasterize(self.cfg['scene'], racks)
            msg = OccupancyGrid()
            msg.header.frame_id = self.route_frame
            msg.info.resolution = field.resolution
            msg.info.width, msg.info.height = field.width, field.height
            msg.info.origin.position.x, msg.info.origin.position.y = field.origin
            msg.info.origin.orientation.w = 1.
            msg.data = field.cells
            self.map_pub.publish(msg)
            self.displayed_racks = racks

        display = (mission.state, mission.index)
        if display != self.displayed_action:
            actions = []
            if mission.state == 'RUNNING':
                key, stage = mission.action.rack, mission.action.state
                for action in mission.actions[mission.index:]:
                    if (action.rack, action.state) != (key, stage):
                        break
                    actions.append(action)
            self.active_path_pub.publish(self.route_path(actions, height=0.055))
            self.displayed_action = display

    def idle_tick(self, now):
        self.zero()

    def start_status(self):
        return {'start_blocker': None}

    def motion_hold(self, now):
        return False

    def tick(self):
        now = time.monotonic()
        try:
            if self.reset_stage:
                self.poll_reset()
            else:
                if (self.simulation and self.get_parameter('autostart').value and not self.autostart_attempted
                        and now-self.birth > 3 and not self.health(precheck=True)):
                    self.autostart_attempted = True
                    ok, message = self.begin()
                    if not ok:
                        self.get_logger().error(message)
                if self.mission.state == 'RUNNING':
                    # Check deadlines/feedback BEFORE issuing external effects.
                    fault = self.health()
                    paused = not fault and self.motion_hold(now)
                    command = self.mission.tick(now, self.p, self.velocity, fault, paused=paused)
                    if self.mission.state == 'RUNNING' and not paused:
                        self.handle_effect()
                    if self.mission.state == 'STOPPED':
                        command = (0.,0.)
                    msg = Twist()
                    msg.linear.x, msg.angular.z = command
                    self.cmd_pub.publish(msg)
                else:
                    self.idle_tick(now)
            self.flush_events()
        except Exception as exc:
            self.mission.stop(f'adapter exception: {exc}')
            self.zero()
            self.get_logger().error(str(exc))
        if now-self.last_status >= 0.2:
            self.last_status = now
            self.publish_visualization()
            a = self.mission.action
            status = dict(state=self.mission.state, reason=self.mission.reason,
                action=a.id if a else None, stage=a.state if a else None,
                rack=a.rack if a else None, carrying=self.mission.carrying,
                placed=self.mission.placed, pose=self.p, reset_pending=bool(self.reset_stage),
                localization_ready=self.localization_ready(),
                elapsed_s=now-self.mission.started if self.mission.started else 0.,
                run_dir=str(self.run_dir) if self.run_dir else None, simulation_only=self.simulation,
                **self.start_status())
            self.status_pub.publish(String(data=json.dumps(status)))

    def close(self):
        self.mission.stop('node shutdown')
        if rclpy.ok():
            self.zero()
        self.flush_events()
        if self.log_file:
            self.log_file.close()
        self.pool.shutdown(wait=True, cancel_futures=True)


def main(args=None, node_type=MissionNode):
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
