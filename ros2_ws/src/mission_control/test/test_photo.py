"""Check that the first-frame timeout does not weaken the image freshness limit."""

from concurrent.futures import Future
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

pytest.importorskip('rclpy')

from sensor_msgs.msg import Image
from std_msgs.msg import String

from mission_control import node
from mission_control.hardware import HardwareMission
from qr_perception.qr_node import bgr_to_image


@pytest.fixture
def photo():
    poller = SimpleNamespace(
        mission=Mock(effect_done=False, skip_qr_timeout=False), camera_job=None, image=None, feedback={},
        cfg={'control': {'image_timeout_s': 3., 'photo_first_frame_timeout_s': 10.}},
        photo_requested_wall=20., photo_requested_stamp=100., photo_stamp=100.,
        fresh=Mock(return_value=True), pool=Mock(), run_dir=Path('/tmp'), qr_decoder=None)
    poller.photo_first_frame_timeout = lambda: node.MissionNode.photo_first_frame_timeout(poller)
    return poller


def image_at(stamp):
    msg = Image()
    msg.header.stamp.sec = stamp
    return msg


def test_first_frame_can_arrive_after_original_deadline(photo, monkeypatch):
    action = SimpleNamespace(id='start-photo', rack='START')
    monkeypatch.setattr(node.time, 'monotonic', lambda: 27.0)
    node.MissionNode.poll_photo(photo, action)
    photo.mission.acknowledge.assert_not_called()

    photo.image = image_at(107)
    photo.feedback['image'] = (27., 107.)
    node.MissionNode.poll_photo(photo, action)
    photo.fresh.assert_called_with('image', 3.)
    photo.mission.emit.assert_called_once_with('photo_first_frame', id=action.id, image_ros_s=107., elapsed_s=7.)
    photo.pool.submit.assert_called_once()


@pytest.mark.parametrize('received_at, stamp', [(30.1, 110), (29.0, 100)])
def test_late_or_pre_request_frame_cannot_satisfy_first_frame_deadline(photo, monkeypatch, received_at, stamp):
    action = SimpleNamespace(id='start-photo', rack='START')
    photo.image = image_at(stamp)
    photo.feedback['image'] = (received_at, float(stamp))
    monkeypatch.setattr(node.time, 'monotonic', lambda: 30.2)
    node.MissionNode.poll_photo(photo, action)
    photo.pool.submit.assert_not_called()
    photo.mission.acknowledge.assert_called_once_with(
        action.id, False, 'no new camera frame at photo pose within 10 s')


@pytest.mark.parametrize('detection_first', [False, True])
def test_in_route_capture_saves_exact_frame_and_acknowledges_only_at_photo_step(tmp_path, detection_first):
    hardware = object.__new__(HardwareMission)
    hardware.clear_route_photo()
    approach = SimpleNamespace(id='turn', kind='turn')
    scan = SimpleNamespace(id='start-photo', rack='START', kind='photo')
    hardware.mission = Mock(state='RUNNING', action=approach, index=0, actions=[approach, scan],
                            skip_qr_timeout=True, effect_done=False)
    hardware.feedback = {}
    hardware.get_clock = Mock()
    hardware.get_clock().now().nanoseconds = 100_000_000_000
    hardware.run_dir = tmp_path
    hardware.pool = Mock()
    hardware.camera_job = None
    hardware.get_logger = Mock()

    def run_inline(function, *args):
        job = Future()
        job.set_result(function(*args))
        return job

    hardware.pool.submit.side_effect = run_inline
    hardware.update_next_photo()
    pixels = np.full((4, 6, 3), (11, 22, 33), dtype=np.uint8)
    frame = bgr_to_image(pixels, 'camera', image_at(102).header.stamp)
    other = bgr_to_image(np.zeros_like(pixels), 'camera', image_at(101).header.stamp)
    hardware.on_image(other)
    hardware.on_qr_frame(String(data=json.dumps({'stamp_ns': 101_000_000_000, 'texts': ['RACKA_TEST']})))
    hardware.on_qr_frame(String(data=json.dumps({'stamp_ns': 99_000_000_000, 'texts': ['START']})))
    assert hardware.route_photo_frame is None

    detection = String(data=json.dumps({'stamp_ns': 102_000_000_000, 'texts': ['START']}))
    if detection_first:
        hardware.on_qr_frame(detection)
        assert hardware.route_photo_frame is None
        hardware.on_image(frame)
    else:
        hardware.on_image(frame)
        hardware.on_qr_frame(detection)
    hardware.handle_effect()
    hardware.handle_effect()

    filename = str(tmp_path / 'START_photo.png')
    assert np.array_equal(cv2.imread(filename), pixels)
    hardware.mission.acknowledge.assert_not_called()
    hardware.mission.emit.assert_called_once_with(
        'photo_saved', id=scan.id, path=filename, qr='START', image_ros_s=102., capture='in_route')
    hardware.mission.action = scan
    hardware.mission.index = 1
    hardware.poll_photo(scan)
    hardware.mission.acknowledge.assert_called_once_with(scan.id, True, filename)
