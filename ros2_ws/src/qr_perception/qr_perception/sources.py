# -*- coding: utf-8 -*-
"""帧来源（M3-1 取流）—— 纯 Python，不依赖 ROS。

统一接口：open() → capture() -> (BGR ndarray, stamp) → close()。
三种实现：
  PicameraSource : 树莓派官方 picamera2（Pi 5 + Camera Module 3，不用 camera_ros）。
                   picamera2 在 import 时才加载，本模块在没装它的机器上也能正常 import。
  FileSource     : 图片目录 / 视频文件 / 单张图片回放 —— 没有相机时调试整条链路用。
  CvSource       : 任意 V4L2 USB 摄像头（cv2.VideoCapture），开发机临时测试用。

工程要点（见《QR感知方案》§5）：
  - 分辨率默认 640x480：检测够用，帧率有保证，不上全分辨率。
  - Camera Module 3 支持固定对焦：focus_diopters（屈光度，1/米）设成停靠距离的倒数，
    避免自动对焦近距离"拉风箱"。
  - 曝光时间可锁（exposure_time_us）：车动的时候宁可暗一点也不要拖影。
"""

from __future__ import annotations

import glob
import os
import time

import cv2
import numpy as np

IMAGE_EXTS = ("*.png", "*.jpg", "*.jpeg", "*.bmp")


class FrameSourceError(RuntimeError):
    """取流失败（打不开 / 读不到帧）。"""


class PicameraSource:
    """picamera2 取流（树莓派专用）。"""

    def __init__(self, width: int = 640, height: int = 480, focus_diopters: float | None = None,
                 exposure_time_us: int | None = None, analog_gain: float | None = None,
                 af_mode_auto: bool = False):
        self.width, self.height = int(width), int(height)
        self.focus_diopters = focus_diopters
        self.exposure_time_us = exposure_time_us
        self.analog_gain = analog_gain
        self.af_mode_auto = af_mode_auto
        self._picam2 = None

    def open(self):
        try:
            from picamera2 import Picamera2  # 延迟 import：非树莓派机器上不炸
        except ImportError as e:
            raise FrameSourceError(
                "picamera2 未安装（树莓派上: sudo apt install python3-picamera2）") from e

        picam2 = Picamera2()
        # XBGR8888 = BGRA 字节序，转 BGR 一次 cvtColor 即可
        config = picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "XBGR8888"})
        picam2.configure(config)

        controls = {}
        if self.focus_diopters is not None:
            # AfMode 0 = 手动；LensPosition 单位是屈光度（1/米）。
            # 例：停靠距离 0.4 m → focus_diopters = 2.5
            controls.update({"AfMode": 0, "LensPosition": float(self.focus_diopters)})
        elif self.af_mode_auto:
            controls.update({"AfMode": 1})  # 连续自动对焦（默认即是，显式写出来便于排查）
        if self.exposure_time_us is not None:
            # 锁曝光：关 AE，手动给曝光时间（µs）。10ms 内基本不会拖影。
            controls.update({"AeEnable": False, "ExposureTime": int(self.exposure_time_us)})
            if self.analog_gain is not None:
                controls.update({"AnalogueGain": float(self.analog_gain)})
        if controls:
            picam2.set_controls(controls)

        picam2.start()
        self._picam2 = picam2
        # 起流第一帧曝光未稳，丢掉
        self._picam2.capture_array("main")

    def capture(self):
        arr = self._picam2.capture_array("main")  # 阻塞等帧
        if arr is None:
            raise FrameSourceError("picamera2 返回空帧")
        frame = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return frame, time.time()

    def close(self):
        if self._picam2 is not None:
            self._picam2.stop()
            self._picam2 = None

    def __str__(self):
        return f"picamera2({self.width}x{self.height})"


class FileSource:
    """离线回放：图片目录（按文件名排序）/ 视频文件 / 单张图片。

    没有相机、没有 ROS 也能把「取流→解码→投票→发布」整条链路跑起来，
    是本机（无相机）调试与 CI 回归的主力。
    """

    def __init__(self, path: str, fps: float = 10.0, loop: bool = True):
        self.path = path
        self.fps = float(fps) if fps and fps > 0 else 10.0
        self.loop = bool(loop)
        self._images = None
        self._video = None
        self._idx = 0
        self._last_t = 0.0
        self._single = None

    def open(self):
        if not os.path.exists(self.path):
            raise FrameSourceError(f"回放路径不存在: {self.path}")
        if os.path.isdir(self.path):
            files = []
            for ext in IMAGE_EXTS:
                files.extend(glob.glob(os.path.join(self.path, ext)))
            self._images = sorted(files)
            if not self._images:
                raise FrameSourceError(f"目录里没有图片: {self.path}")
        else:
            ext = os.path.splitext(self.path)[1].lower()
            if ext in (".png", ".jpg", ".jpeg", ".bmp"):
                self._single = cv2.imread(self.path)
                if self._single is None:
                    raise FrameSourceError(f"读不了图片: {self.path}")
            else:
                self._video = cv2.VideoCapture(self.path)
                if not self._video.isOpened():
                    raise FrameSourceError(f"打不开视频: {self.path}")

    def _pace(self):
        """按 replay fps 节流，模拟真实相机节奏。"""
        now = time.time()
        wait = 1.0 / self.fps - (now - self._last_t)
        if wait > 0:
            time.sleep(wait)
        self._last_t = time.time()

    def capture(self):
        self._pace()
        if self._single is not None:
            if not self.loop and self._idx > 0:
                raise FrameSourceError("单图回放结束（loop=false）")
            self._idx += 1
            return self._single.copy(), time.time()

        if self._images is not None:
            if self._idx >= len(self._images):
                if not self.loop:
                    raise FrameSourceError("图片目录回放结束（loop=false）")
                self._idx = 0
            frame = cv2.imread(self._images[self._idx])
            self._idx += 1
            if frame is None:
                return np.zeros((self.height_default()), np.uint8), time.time()
            return frame, time.time()

        ok, frame = self._video.read()
        if not ok:
            if not self.loop:
                raise FrameSourceError("视频回放结束（loop=false）")
            self._video.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._video.read()
            if not ok:
                raise FrameSourceError("视频读不出帧")
        return frame, time.time()

    @staticmethod
    def height_default():
        return (480, 640, 3)

    def close(self):
        if self._video is not None:
            self._video.release()
        self._images = self._video = self._single = None

    def __str__(self):
        return f"file({self.path})"


class CvSource:
    """USB / 板载 V4L2 摄像头（cv2.VideoCapture）。"""

    def __init__(self, index: int = 0, width: int = 640, height: int = 480):
        self.index = int(index)
        self.width, self.height = int(width), int(height)
        self._cap = None

    def open(self):
        self._cap = cv2.VideoCapture(self.index)
        if not self._cap.isOpened():
            raise FrameSourceError(f"打不开摄像头 index={self.index}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def capture(self):
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise FrameSourceError("摄像头读帧失败")
        return frame, time.time()

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __str__(self):
        return f"cv({self.index}, {self.width}x{self.height})"


def make_source(kind: str, **kwargs):
    """按名字建帧源。kind: picamera2 | file | cv"""
    kind = kind.lower()
    if kind in ("picamera2", "picam", "pi"):
        return PicameraSource(**kwargs)
    if kind in ("file", "replay"):
        return FileSource(**kwargs)
    if kind in ("cv", "webcam", "usb"):
        return CvSource(**kwargs)
    raise FrameSourceError(f"未知帧源类型: {kind}（可选 picamera2 / file / cv）")
