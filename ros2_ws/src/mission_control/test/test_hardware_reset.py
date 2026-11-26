"""Regression coverage for hardware mission localization."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.task import Future
from tf2_ros import ExtrapolationException

from nav2_msgs.srv import SetInitialPose
from std_srvs.srv import Empty

from mission_control.hardware import HardwareMission
from mission_control.route import Action


@pytest.fixture
def mission():
    class TestMission(HardwareMission):
        def __init__(self):
            self.autostart_attempted = True
            self.preparation = None
            self.preparation_id = 0
            self.mission = SimpleNamespace(state='STOPPED')
            self.camera_job = None
            self.route_photo_job = None
            self.reseed_at_start = False
            self.localization_hold = None
            self.localization_after = 0.
            self.amcl_transform_tolerance = 0.35
            self.localization = None
            self.cfg = {'control': {'scan_timeout_s': 0.75}}
            self.tf_buffer = Mock()
            self.ros_seconds = Mock(return_value=100.4)
            self.prepare_chassis = Mock(return_value=None)
            self.amcl_seed = Mock()
            self.amcl_seed.call_async.side_effect = lambda request: Future()
            self.amcl_update = Mock()
            self.amcl_update.call_async.side_effect = lambda request: Future()
            self.initial_pose_message = Mock(return_value=PoseWithCovarianceStamped())
            self.health = Mock(side_effect=self.check_transform)
            self.finish_reset = Mock(side_effect=self.reset_state)
            self.begin = Mock(return_value=(True, 'Started'))
            self.get_parameter = Mock(return_value=SimpleNamespace(value=False))
            self.get_logger = Mock()

        def check_transform(self, precheck):
            if self.map_transform() is None:
                return 'Waiting for AMCL map-to-odom transform'

        def reset_state(self):
            self.mission.state = 'IDLE'

    return TestMission()


def test_reset_completes_without_localization(mission):
    assert mission.request_preparation('reset')[0]
    mission.idle_tick(mission.preparation['deadline'] - 1.)
    assert mission.preparation_result is True
    assert mission.mission.state == 'IDLE'
    mission.health.assert_not_called()
    mission.amcl_seed.call_async.assert_not_called()
    mission.tf_buffer.lookup_transform.assert_not_called()
    mission.begin.assert_not_called()


def test_start_requires_post_seed_pose_and_its_transform(mission):
    mission.mission.state = 'IDLE'
    mission.reseed_at_start = True
    assert mission.request_preparation('start')[0]
    now = mission.preparation['deadline'] - 1.
    mission.idle_tick(now)
    seed = mission.preparation['seed_job']
    assert isinstance(mission.amcl_seed.call_async.call_args.args[0], SetInitialPose.Request)
    mission.initial_pose_message.assert_called_once_with(0.)
    mission.begin.assert_not_called()

    mission.ros_seconds.return_value = 100.1
    seed.set_result(SetInitialPose.Response())
    mission.idle_tick(now + 0.1)
    assert isinstance(mission.amcl_update.call_async.call_args.args[0], Empty.Request)
    old_pose = PoseWithCovarianceStamped()
    old_pose.header.frame_id = 'map'
    old_pose.header.stamp.sec = 100
    mission.on_localization(old_pose)
    assert mission.localization is None
    mission.begin.assert_not_called()

    new_pose = PoseWithCovarianceStamped()
    new_pose.header.frame_id = 'map'
    new_pose.header.stamp.sec = 100
    new_pose.header.stamp.nanosec = 200_000_000
    mission.on_localization(new_pose)
    transform = TransformStamped()
    transform.header.stamp.sec = 100
    transform.header.stamp.nanosec = 540_000_000
    mission.tf_buffer.lookup_transform.return_value = transform
    mission.idle_tick(now + 0.2)
    mission.begin.assert_not_called()

    transform.header.stamp.nanosec = 550_000_000
    mission.idle_tick(now + 0.3)
    assert mission.preparation_result is True
    mission.begin.assert_called_once()


def test_start_can_retry_after_timeout(mission):
    mission.mission.state = 'IDLE'
    mission.reseed_at_start = True
    assert mission.request_preparation('start')[0]
    deadline = mission.preparation['deadline']
    mission.idle_tick(deadline - 1.)
    seed = mission.preparation['seed_job']
    mission.idle_tick(deadline)
    assert mission.preparation_result is False
    assert mission.reseed_at_start
    assert seed.cancelled()
    mission.begin.assert_not_called()

    assert mission.request_preparation('start')[0]
    mission.idle_tick(mission.preparation['deadline'] - 1.)
    assert mission.amcl_seed.call_async.call_count == 2


@pytest.fixture
def recovery(mission):
    action = Action('start-turn', 'START', 'TO_START', 'turn', (1., .3, 0.), (1., .3, 1.), 1., .8, 40.)
    mission.mission = Mock(state='RUNNING', action=action, effect_done=False)
    mission.mission.footprint_fault.return_value = None
    mission.cfg['control'].update(stopped_angular_rps=.04, stopped_linear_mps=.01,
                                  max_cross_track_m=.045, max_heading_error_rad=.30)
    mission.odom_pose = (0., 0., 0.)
    mission.velocity = (0., 0.)
    mission.raw_velocity = (0., 0., .1)
    mission.progress_correction = (1., .3, 0.)
    mission.ros_seconds.return_value = 100.
    return mission


def observe(recovery, stamp, x=1., yaw=.2):
    recovery.map_correction = recovery.p = (x, .3, yaw)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec, tenth = divmod(round(stamp * 10), 10)
    pose.header.stamp.nanosec = tenth * 100_000_000
    recovery.on_localization(pose)
    transform = TransformStamped()
    transform.transform.translation.x = x
    transform.transform.translation.y = .3
    transform.transform.rotation.z = math.sin(yaw / 2)
    transform.transform.rotation.w = math.cos(yaw / 2)
    recovery.tf_buffer.lookup_transform.return_value = transform
    recovery.ros_seconds.return_value = stamp + .02


def test_recovery_requires_three_fresh_consistent_amcl_updates(recovery):
    observe(recovery, 100.1)
    assert recovery.motion_hold(10.)
    assert recovery.motion_hold(10.1)  # Wheels still moving.
    recovery.raw_velocity = (0., 0., 0.)
    assert recovery.motion_hold(10.2)
    assert not recovery.localization_hold.samples

    observe(recovery, 100.4)
    recovery.tf_buffer.lookup_transform.side_effect = ExtrapolationException('TF pending')
    assert recovery.motion_hold(10.4)
    assert not recovery.localization_hold.samples
    recovery.tf_buffer.lookup_transform.side_effect = None
    assert recovery.motion_hold(10.42)
    assert recovery.motion_hold(10.44)
    assert len(recovery.localization_hold.samples) == 1

    observe(recovery, 100.6, yaw=.32)
    assert recovery.motion_hold(10.6)
    assert len(recovery.localization_hold.samples) == 1
    observe(recovery, 100.8, yaw=.33)
    assert recovery.motion_hold(10.8)
    observe(recovery, 101., yaw=.34)
    assert not recovery.motion_hold(11.)
    assert recovery.localization_hold is None
    assert recovery.progress_correction == recovery.map_correction
    recovery.mission.resume_after_pause.assert_called_once_with(11., 10.)
    recovery.amcl_update.call_async.assert_called()


def test_recovery_times_out_if_stable_pose_is_off_route(recovery):
    observe(recovery, 100.1, x=1.1)
    assert recovery.motion_hold(10.)
    recovery.raw_velocity = (0., 0., 0.)
    assert recovery.motion_hold(10.2)
    for stamp in (100.4, 100.6, 100.8):
        observe(recovery, stamp, x=1.1)
        assert recovery.motion_hold(10. + stamp - 100.)
    assert 'too far from the active waypoint' in recovery.localization_hold.detail
    assert recovery.motion_hold(16.)
    recovery.mission.stop.assert_called_once()
    assert 'AMCL recovery timed out' in recovery.mission.stop.call_args.args[0]
    recovery.mission.resume_after_pause.assert_not_called()
