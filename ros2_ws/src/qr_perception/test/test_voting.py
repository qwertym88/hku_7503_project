# -*- coding: utf-8 -*-
"""MultiFrameVoter 单元测试（colcon test / pytest / 直接 python3 均可跑，不需要 ROS）。

  colcon test --packages-select qr_perception
  python3 ros2_ws/src/qr_perception/test/test_voting.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from qr_perception.voting import MultiFrameVoter  # noqa: E402


class TestVoting(unittest.TestCase):

    def test_single_frame_noise_not_published(self):
        """单帧误读攒不够票，不上报。"""
        v = MultiFrameVoter(confirm_frames=2)
        self.assertEqual(v.update(["RACKA_4X6M"]), [])
        self.assertEqual(v.update(["RACKA_4X6M"]), ["RACKA_4X6M"])

    def test_no_spam_while_visible(self):
        """码一直在视野里也只发一次。"""
        v = MultiFrameVoter(confirm_frames=2)
        out = []
        for _ in range(12):
            out += v.update(["RACKA_4X6M"])
        self.assertEqual(out, ["RACKA_4X6M"])

    def test_short_absence_stays_silent(self):
        """短暂消失（< absent_frames）后回来，仍静默。"""
        v = MultiFrameVoter(confirm_frames=2, absent_frames=5)
        v.update(["RACKA_4X6M"])
        v.update(["RACKA_4X6M"])
        for _ in range(3):
            v.update([])
        self.assertEqual(v.update(["RACKA_4X6M"]), [])

    def test_long_absence_reannounces(self):
        """消失够久后回来，重新上报（重新对准同一货架的场景）。"""
        v = MultiFrameVoter(confirm_frames=2, absent_frames=5)
        v.update(["RACKA_4X6M"])
        v.update(["RACKA_4X6M"])
        for _ in range(5):
            v.update([])
        self.assertEqual(v.update(["RACKA_4X6M"]), [])
        self.assertEqual(v.update(["RACKA_4X6M"]), ["RACKA_4X6M"])

    def test_intermittent_detection_accumulates(self):
        """间歇识别（每 2 帧看到 1 次）也能攒够票确认。"""
        v = MultiFrameVoter(confirm_frames=2, absent_frames=15)
        out = []
        for texts in (["END"], [], ["END"], [], ["END"]):
            out += v.update(texts)
        self.assertEqual(out, ["END"])

    def test_empty_frames_never_confirm(self):
        """空帧永远不上报。"""
        v = MultiFrameVoter()
        for _ in range(50):
            self.assertEqual(v.update([]), [])

    def test_confirm_one_immediate(self):
        """confirm_frames=1 时来一个发一个（随后静默）。"""
        v = MultiFrameVoter(confirm_frames=1, absent_frames=5)
        self.assertEqual(v.update(["START"]), ["START"])
        self.assertEqual(v.update(["START"]), [])

    def test_multiple_codes_pick_most_voted(self):
        """同帧多码：票最高者优先确认。"""
        v = MultiFrameVoter(confirm_frames=2)
        v.update(["RACKA_4X6M", "END"])
        self.assertEqual(v.update(["RACKA_4X6M", "END"]), ["RACKA_4X6M"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
