"""Fixed-route execution using AMCL and the chassis micro-ROS interfaces."""

from dataclasses import dataclass, field
import json
import math
import time

from nav_msgs.msg import Odometry
from nav2_msgs.srv import SetInitialPose
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String, UInt8
from std_srvs.srv import Empty, Trigger
from tf2_ros import TransformException

from .motion import curve_request, motion_request
from .node import MissionNode, capture_image, main as run_node, matches_qr, stamp_seconds
from .route import angle, compose


def planar_transform(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


def correction_delta(first, second, odom):
    before, after = compose(first, odom), compose(second, odom)
    return math.dist(before[:2], after[:2]), abs(angle(after[2] - before[2]))


@dataclass
class LocalizationHold:
    started: float
    sample_after: float | None = None
    samples: list = field(default_factory=list)
    update_job: object = None
    update_after: float = 0.
    detail: str = 'Waiting for the chassis to stop'


class HardwareMission(MissionNode):
    simulation = False
    preparation_timeout_s = 30.0
    localization_hold_timeout_s = 6.
    localization_stable_samples = 3
    localization_stable_m = 0.02
    localization_stable_rad = 0.05

    def setup_cargo(self):
        self.lift_baseline = None
        self.progress_correction = None
        self.localization_hold = None
        self.preparation = None
        self.preparation_id = 0
        self.preparation_result = None
        self.preparation_detail = ''
        self.reseed_at_start = True
        self.clear_route_photo()
        if self.mission.skip_qr_timeout:
            self.create_subscription(String, '/qr/frame_detections', self.on_qr_frame, 10)
        self.unlock = self.create_client(Trigger, '/chassis/unlock')
        if not self.encoder_only:
            self.declare_parameter('amcl_transform_tolerance_s', 0.35)
            self.amcl_transform_tolerance = self.get_parameter('amcl_transform_tolerance_s').value
            self.amcl_seed = self.create_client(SetInitialPose, '/set_initial_pose')
            self.amcl_update = self.create_client(Empty, '/request_nomotion_update')
            self.create_subscription(Odometry, '/odom', self.on_raw_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/chassis/diagnostics', self.on_cargo, qos_profile_sensor_data)

    def clear_route_photo(self):
        self.next_photo = None
        self.photo_eligible_after_ns = 0
        self.qr_images = {}
        self.qr_frames = {}
        self.route_photo_frame = None
        self.route_photo_job = None
        self.route_photo_saved = None

    def update_next_photo(self):
        if not self.mission.skip_qr_timeout:
            return
        if self.next_photo and self.mission.index <= self.next_photo[0]:
            return
        self.clear_route_photo()
        self.photo_eligible_after_ns = self.get_clock().now().nanoseconds
        self.next_photo = next(((index, action) for index, action in
                                enumerate(self.mission.actions[self.mission.index:], self.mission.index)
                                if action.kind == 'photo'), None)

    def on_image(self, msg):
        super().on_image(msg)
        if msg.width and msg.height and msg.data and self.photo_frame_eligible():
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.match_route_photo_frame(stamp, image=msg)

    def on_qr_frame(self, msg):
        if not self.photo_frame_eligible():
            return
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(data, dict) or type(data.get('stamp_ns')) is not int or not isinstance(data.get('texts'), list):
            return
        label = self.next_photo[1].rack
        text = next((text for text in data['texts'] if isinstance(text, str) and matches_qr(text, label)), None)
        if text:
            self.match_route_photo_frame(data['stamp_ns'], text=text)

    def photo_frame_eligible(self):
        if (self.mission.state != 'RUNNING' or self.next_photo is None
                or self.route_photo_saved is not None or self.route_photo_frame is not None):
            return False
        index, action = self.next_photo
        return (self.mission.index < index or
                (self.mission.index == index and not self.mission.effect_done
                 and time.monotonic() - self.mission.entered < action.timeout))

    def match_route_photo_frame(self, stamp, *, image=None, text=None):
        if stamp <= self.photo_eligible_after_ns:
            return
        if image is None:
            image = self.qr_images.pop(stamp, None)
            if image is None:
                self.qr_frames[stamp] = text
                if len(self.qr_frames) > 8:
                    self.qr_frames.pop(next(iter(self.qr_frames)))
                return
        if text is None:
            text = self.qr_frames.pop(stamp, None)
            if text is None:
                self.qr_images[stamp] = image
                if len(self.qr_images) > 8:
                    self.qr_images.pop(next(iter(self.qr_images)))
                return
        self.route_photo_frame = (image, text)
        self.qr_frames.clear()
        self.qr_images.clear()

    def handle_effect(self):
        self.update_next_photo()
        if self.next_photo:
            self.poll_route_photo()
        super().handle_effect()

    def poll_route_photo(self):
        frame = self.route_photo_frame
        if frame is None:
            return
        image, text = frame
        _, action = self.next_photo
        if self.route_photo_job is None:
            if self.mission.action == action and self.camera_job is not None:
                return  # Let the existing capture finish before writing the same file.
            filename = self.run_dir / f'{action.rack}_photo.png'
            self.route_photo_job = self.pool.submit(capture_image, image, filename, None, action.rack)
            return
        if not self.route_photo_job.done():
            return
        try:
            filename, _ = self.route_photo_job.result()
        except Exception as exc:
            self.get_logger().warning(f'{action.id}: could not save in-route QR capture: {exc}')
        else:
            self.route_photo_saved = filename
            self.report_photo_saved(action.id, filename, text, image_ros_s=stamp_seconds(image), capture='in_route')
        self.route_photo_frame = self.route_photo_job = None

    def poll_photo(self, action):
        if self.mission.effect_done:
            return
        if self.next_photo and self.next_photo[1] == action:
            if self.route_photo_saved:
                self.mission.acknowledge(action.id, True, self.route_photo_saved)
                return
            if self.route_photo_frame is not None:
                if self.camera_job is None or not self.camera_job.done():
                    return
                super().poll_photo(action, accept_new_frame=False)
                return
        super().poll_photo(action, accept_new_frame=(
            not self.mission.skip_qr_timeout or time.monotonic() - self.mission.entered < action.timeout))

    def begin(self):
        started, detail = super().begin()
        if started:
            self.update_next_photo()
        return started, detail

    def finish_reset(self):
        self.clear_route_photo()
        super().finish_reset()

    def on_raw_odom(self, msg):
        velocity = msg.twist.twist
        if (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_footprint'
                and all(math.isfinite(v) for v in (velocity.linear.x, velocity.linear.y, velocity.angular.z))):
            self.raw_velocity = (velocity.linear.x, velocity.linear.y, velocity.angular.z)
            self.feedback['raw_odom'] = (time.monotonic(), stamp_seconds(msg))
        else:
            self.feedback.pop('raw_odom', None)

    def on_cargo(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            status = None
        integer_fields = ('enabled', 'lift_inhibited', 'lifter_position',
                          'lifter_rx', 'lifter_rejected', 'lifter_completed')
        text_fields = ('lifter_state', 'lifter_fault', 'lifter_last_result')
        if (not isinstance(status, dict)
                or not all(type(status.get(key)) is int for key in integer_fields)
                or not all(isinstance(status.get(key), str) for key in text_fields)):
            self.cargo_status = None
            return
        self.cargo_status = status
        self.feedback['cargo'] = (time.monotonic(), self.ros_seconds())

    def map_transform(self):
        if self.reseed_at_start:
            return None
        return super().map_transform()

    def feedback_health(self, precheck):
        fault = self.chassis_feedback_health(precheck)
        if fault:
            return fault
        transform = self.map_transform()
        if transform is None:
            return 'Waiting for AMCL map-to-odom transform'
        correction = planar_transform(transform)
        if not all(math.isfinite(v) for v in correction):
            return 'Invalid map-to-odom transform'
        self.map_correction = correction
        self.p = compose(correction, self.odom_pose)
        if precheck:
            self.progress_correction = correction

    def chassis_feedback_health(self, precheck):
        fault = super().feedback_health(precheck)
        if fault:
            return fault
        if not self.fresh('raw_odom', self.cfg['control']['feedback_timeout_s']):
            return 'raw_odom missing/stale'
        if not self.cmd_pub.get_subscription_count() or not self.lift_pub.get_subscription_count():
            return 'Waiting for chassis /cmd_vel and /lifter subscriptions'

    def clear_localization_hold(self):
        hold = self.localization_hold
        if hold and hold.update_job is not None and not hold.update_job.done():
            hold.update_job.cancel()
        self.localization_hold = None

    def motion_hold(self, now):
        mission = self.mission
        hold = self.localization_hold
        if hold is None:
            previous = self.progress_correction
            if previous is not None:
                distance, heading = correction_delta(previous, self.map_correction, self.odom_pose)
                if distance > 0.05 or heading > 0.12:
                    self.localization_hold = LocalizationHold(now)
                    mission.emit('localization_paused', id=mission.action.id,
                                 translation_m=distance, heading_rad=heading)
                    return True
            self.progress_correction = self.map_correction
            return False

        if now - hold.started >= self.localization_hold_timeout_s:
            mission.stop(f'{mission.action.id}: AMCL recovery timed out: {hold.detail}')
            self.clear_localization_hold()
            return True

        hold.detail = self.localization_resume_blocker(hold, now)
        if hold.detail:
            return True

        if mission.action.kind == 'photo' and not mission.effect_done:
            self.effect_id = None  # Restart the stopped scan if a capture is still needed.
        mission.resume_after_pause(now, hold.started)
        self.progress_correction = self.map_correction
        mission.emit('localization_resumed', id=mission.action.id, paused_s=now - hold.started,
                     pose=list(self.p))
        self.clear_localization_hold()
        return False

    def localization_resume_blocker(self, hold, now):
        c = self.cfg['control']
        vx, vy, wz = self.raw_velocity
        if (max(math.hypot(vx, vy), abs(self.velocity[0])) > c['stopped_linear_mps']
                or max(abs(wz), abs(self.velocity[1])) > c['stopped_angular_rps']):
            hold.sample_after = None
            hold.samples.clear()
            return 'Waiting for the chassis to stop'
        if hold.sample_after is None:
            hold.sample_after = self.ros_seconds()
        if (now >= hold.update_after and (hold.update_job is None or hold.update_job.done())
                and self.amcl_update.service_is_ready()):
            hold.update_job = self.amcl_update.call_async(Empty.Request())
            hold.update_after = now + 0.25

        self.collect_localization_sample(hold)
        if self.ros_seconds() - hold.sample_after > c['scan_timeout_s']:
            hold.samples.clear()
        if len(hold.samples) < self.localization_stable_samples:
            return f'Waiting for stable AMCL updates ({len(hold.samples)}/{self.localization_stable_samples})'
        if any(not self.corrections_agree(sample, self.map_correction) for sample in hold.samples):
            return 'Waiting for the current transform to agree with the AMCL updates'
        fault = self.mission.footprint_fault(self.p) or self.resume_route_fault()
        if fault:
            return fault
        if self.mission.action.kind == 'photo' and self.camera_job is not None and not self.camera_job.done():
            return 'Waiting for the previous camera capture to finish'

    def corrections_agree(self, first, second):
        distance, heading = correction_delta(first, second, self.odom_pose)
        return distance <= self.localization_stable_m and heading <= self.localization_stable_rad

    def collect_localization_sample(self, hold):
        msg = self.localization
        if msg is None:
            return
        stamp = stamp_seconds(msg)
        if stamp <= hold.sample_after or self.ros_seconds() - stamp > self.cfg['control']['scan_timeout_s']:
            return
        # AMCL also re-broadcasts its old TF. Only count a new pose paired
        # with its transform at the scan timestamp plus AMCL's TF tolerance.
        tf_time = Time.from_msg(msg.header.stamp) + Duration(seconds=self.amcl_transform_tolerance)
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', tf_time)
        except TransformException:
            return
        sample = planar_transform(transform)
        if not all(math.isfinite(v) for v in sample):
            return
        hold.sample_after = stamp
        if any(not self.corrections_agree(previous, sample) for previous in hold.samples):
            hold.samples.clear()
        hold.samples.append(sample)
        del hold.samples[:-self.localization_stable_samples]

    def resume_route_fault(self):
        mission = self.mission
        action = mission.action
        c = self.cfg['control']
        if action.kind == 'curve':
            return curve_request(action, mission.curve_path, mission.curve_progress, self.p, c)[3]
        if action.kind == 'drive':
            dx, dy = self.p[0] - action.start[0], self.p[1] - action.start[1]
            direction = (action.target[0] - action.start[0], action.target[1] - action.start[1])
            length = math.hypot(*direction)
            along = (dx * direction[0] + dy * direction[1]) / length
            if not -c['position_tolerance_m'] <= along <= length + c['position_tolerance_m']:
                return 'Recovered pose is beyond the active straight segment'
            return motion_request(action, self.p, c)[3]
        if math.dist(self.p[:2], action.target[:2]) > c['max_cross_track_m']:
            return 'Recovered pose is too far from the active waypoint'
        if action.kind != 'turn' and abs(angle(self.p[2] - action.target[2])) > c['max_heading_error_rad']:
            return 'Recovered heading is too far from the active waypoint heading'

    def chassis_health(self):
        status = self.cargo_status
        if not status or not self.fresh('cargo', self.cfg['control']['cargo_timeout_s']):
            return 'Chassis diagnostics missing/stale/invalid'
        if status['lifter_fault'] != 'none' or status['lifter_state'] not in ('idle', 'raising', 'lowering'):
            return f"Lifter {status['lifter_state']}: {status['lifter_fault']}"
        if self.count_publishers('/lifter') != 1:
            return '/lifter must have exactly one publisher'

    def cargo_health(self, precheck):
        fault = self.chassis_health()
        if fault:
            return fault
        status = self.cargo_status
        if status.get('command_locked'):
            return 'Chassis commands are locked after reconnect'
        blocked = status['lift_inhibited'] or status['lifter_state'] != 'idle'
        if precheck and (blocked or status['enabled'] or status['lifter_position'] != 0):
            return (f"Waiting for stopped chassis and low lifter (enabled={status['enabled']}, "
                    f"state={status['lifter_state']}, position={status['lifter_position']})")
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
        completed, detail = self.lift_result(baseline, int(action.kind == 'lift'))
        if completed is not None:
            self.mission.acknowledge(action.id, completed, detail)

    def lift_result(self, baseline, target):
        status = self.cargo_status
        if status['lifter_rx'] == baseline['lifter_rx']:
            return None, 'Waiting for the firmware lifter acknowledgement'
        result = status['lifter_last_result']
        if (status['lifter_rx'] != baseline['lifter_rx'] + 1
                or status['lifter_rejected'] != baseline['lifter_rejected']
                or result not in ('accepted', 'duplicate')):
            return False, f'Lifter command mismatch/rejected: {result}'
        completed = (status['lifter_completed'] > baseline['lifter_completed']
                     if result == 'accepted' else baseline['lifter_position'] == target)
        if (completed and status['lifter_state'] == 'idle' and not status['lift_inhibited']
                and status['lifter_position'] == target):
            return True, f'Firmware target {target} ({result})'
        return None, f"Waiting for lifter target {target} (state={status['lifter_state']})"

    def request_preparation(self, operation):
        if self.preparation:
            return False, f"Already preparing {self.preparation['operation']}"
        if self.mission.state == 'RUNNING':
            return False, 'Mission is running; cancel it before restoring the physical scene'
        if operation == 'start' and self.mission.state != 'IDLE':
            return False, 'Restore the physical scene and reset before starting another run'
        self.preparation_id += 1
        self.preparation = dict(operation=operation, deadline=time.monotonic() + self.preparation_timeout_s,
                                still_since=None, unlock_job=None, unlock_after=0.,
                                lower_baseline=None, seed_job=None, seed_acknowledged=False,
                                update_job=None, update_after=0.)
        self.preparation_result = None
        self.preparation_detail = 'Waiting for chassis feedback'
        self.get_logger().info(f'Preparing mission {operation}')
        return True, str(self.preparation_id)

    def start_service(self, request, response):
        response.success, response.message = self.request_preparation('start')
        return response

    def reset_service(self, request, response):
        response.success, response.message = self.request_preparation('reset')
        return response

    def cancel_service(self, request, response):
        self.autostart_attempted = True
        if self.preparation:
            self.finish_preparation(False, 'Preparation cancelled')
        return super().cancel_service(request, response)

    def finish_preparation(self, success, detail):
        for key in ('unlock_job', 'seed_job', 'update_job'):
            job = self.preparation[key]
            if job is not None and not job.done():
                job.cancel()
        if self.preparation['operation'] == 'start':
            self.reseed_at_start = not success
        self.preparation = None
        self.preparation_result, self.preparation_detail = success, detail
        if not success:
            self.get_logger().error(detail)

    def prepare_chassis(self, pending, now):
        status = self.cargo_status
        fault = self.chassis_health()
        if fault:
            if pending['lower_baseline'] is not None or (status and status['lifter_fault'] != 'none'):
                raise RuntimeError(fault)
            pending['still_since'] = None
            return fault
        if self.count_publishers('/cmd_vel') != 1:
            pending['still_since'] = None
            return '/cmd_vel must have exactly one publisher; close teleop/other controllers'
        if not self.cmd_pub.get_subscription_count() or not self.lift_pub.get_subscription_count():
            pending['still_since'] = None
            return 'Waiting for chassis /cmd_vel and /lifter subscriptions'
        control = self.cfg['control']
        if not self.fresh('raw_odom', control['feedback_timeout_s']):
            pending['still_since'] = None
            return 'Waiting for raw chassis odometry'
        speeds = status.get('speed')
        if (not isinstance(speeds, list) or len(speeds) != 4
                or any(type(v) not in (float, int) or not math.isfinite(v) for v in speeds)):
            raise RuntimeError('Chassis diagnostics lack valid wheel speeds')
        vx, vy, wz = self.raw_velocity
        if (status['enabled'] or any(abs(v) > 0.005 for v in speeds)
                or math.hypot(vx, vy) > control['stopped_linear_mps']
                or abs(wz) > control['stopped_angular_rps']):
            pending['still_since'] = None
            if pending['lower_baseline'] is not None:
                raise RuntimeError('Chassis moved during automatic lowering')
            return 'Waiting for the chassis to stop'
        if pending['still_since'] is None:
            pending['still_since'] = now
        baseline = pending['lower_baseline']
        if baseline is not None:
            if status.get('command_locked'):
                raise RuntimeError('Chassis locked during automatic lowering')
            complete, detail = self.lift_result(baseline, 0)
            if complete is False:
                raise RuntimeError(detail)
            return None if complete else detail
        if status['lift_inhibited'] or status['lifter_state'] != 'idle':
            return f"Waiting for lifter to settle ({status['lifter_state']})"
        if status.get('command_locked'):
            return self.prepare_unlock(pending, now)
        if status['lifter_position'] == 0:
            return None
        if now - pending['still_since'] < 0.2:
            return 'Waiting for wheels to settle before lowering'
        pending['lower_baseline'] = status
        self.lift_pub.publish(UInt8(data=0))
        return 'Lowering lifter to position 0'

    def prepare_unlock(self, pending, now):
        if not self.unlock.service_is_ready():
            return 'Waiting for /chassis/unlock'
        job = pending['unlock_job']
        if job is not None and job.done():
            result = job.result()
            pending['unlock_job'] = None
            if not result.success:
                self.get_logger().info(f'Chassis unlock waiting: {result.message}')
        if pending['unlock_job'] is None and now >= pending['unlock_after'] and now - pending['still_since'] >= 1.0:
            pending['unlock_job'] = self.unlock.call_async(Trigger.Request())
            pending['unlock_after'] = now + 1.0
        return 'Waiting for chassis unlock and firmware diagnostics'

    def prepare_start_localization(self, pending, now):
        seed = pending['seed_job']
        if seed is None:
            if not self.amcl_seed.service_is_ready():
                return 'Waiting for AMCL /set_initial_pose'
            # Match AMCL's configured cold start: the operator supplies a known pose.
            initial = self.initial_pose_message(0.)
            self.localization = None
            self.localization_after = stamp_seconds(initial)
            pending['seed_job'] = self.amcl_seed.call_async(SetInitialPose.Request(pose=initial))
            return 'Setting the known start pose in AMCL'
        if not seed.done():
            return 'Waiting for AMCL to process the start pose'
        if not pending['seed_acknowledged']:
            seed.result()
            pending['seed_acknowledged'] = True
            self.localization = None
            self.localization_after = self.ros_seconds()

        if self.localization is None:
            # A queued scan can trigger the first publication with a pre-seed
            # timestamp. Request an update while stationary if that happens.
            update = pending['update_job']
            if (now >= pending['update_after'] and (update is None or update.done())
                    and self.amcl_update.service_is_ready()):
                pending['update_job'] = self.amcl_update.call_async(Empty.Request())
                pending['update_after'] = now + 0.5
            return 'Waiting for a fresh AMCL estimate at the start'

        transform = super().map_transform()
        # AMCL can also re-broadcast the previous map-to-odom transform with a
        # new timestamp. Pair it with an actual post-seed pose publication.
        pose_tf_stamp = stamp_seconds(self.localization) + self.amcl_transform_tolerance
        if transform is None or stamp_seconds(transform) + 1e-6 < pose_tf_stamp:
            return 'Waiting for the transform from the new AMCL estimate'
        self.reseed_at_start = False

    def idle_tick(self, now):
        # Even zero commands restart the firmware's post-reconnect unlock timer.
        self.clear_localization_hold()
        if self.get_parameter('autostart').value and not self.autostart_attempted:
            self.autostart_attempted = True
            if not self.preparation:
                self.request_preparation('start')
        pending = self.preparation
        if not pending:
            return
        try:
            if now >= pending['deadline']:
                raise RuntimeError(f'Preparation timed out: {self.preparation_detail}')
            self.preparation_detail = self.prepare_chassis(pending, now)
            if self.preparation_detail:
                return
            if pending['operation'] == 'reset':
                if any(job is not None and not job.done() for job in (self.camera_job, self.route_photo_job)):
                    self.preparation_detail = 'Waiting for previous photo to finish'
                    return
                self.reseed_at_start = True
                self.p = self.progress_correction = None
                self.finish_reset()
                self.finish_preparation(True, self.reset_detail())
                return
            if self.reseed_at_start:
                self.preparation_detail = self.prepare_start_localization(pending, now)
                if self.preparation_detail:
                    return
            self.preparation_detail = self.health(precheck=True)
            if self.preparation_detail:
                return
            self.finish_preparation(*self.begin())
        except RuntimeError as exc:
            self.finish_preparation(False, str(exc))

    def reset_detail(self):
        return 'Reset complete; AMCL will initialize on start'

    def start_status(self):
        blocker = self.preparation_detail if self.preparation else None
        if not self.preparation and self.mission.state == 'IDLE':
            blocker = self.health(precheck=True)
        hold = self.localization_hold if self.mission.state == 'RUNNING' else None
        return dict(start_blocker=blocker, preparation_id=self.preparation_id,
                    preparation_pending=self.preparation is not None,
                    preparation_result=self.preparation_result, preparation_detail=self.preparation_detail,
                    localization_hold=hold.detail if hold else None)


def main(args=None):
    run_node(args, node_type=HardwareMission)
