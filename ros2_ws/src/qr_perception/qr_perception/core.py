# -*- coding: utf-8 -*-
"""QR 解码核心 —— 纯 Python，不依赖 ROS（M3-2）。

设计要点（见 docs/给队友看/QR感知方案.md）：
  - 输入一帧 BGR 图像，输出 QrResult 列表；识别不到返回空列表，绝不抛异常。
  - 解码优先级：OpenCV QRCodeDetector（首选）→ 2x 放大重试 → pyzbar（兜底，装了才用）。
  - 深度重试（放大 / pyzbar）不是每帧都做：没有码的画面占大头，全开会让帧率掉到 15fps 以下。
    用 deep_every_n 控制：每 N 帧做一次全流水线，其余帧只跑最便宜的 OpenCV 原图。
  - 解码前灰度化（省一半算力）；结果按码的面积从大到小排（最大的 = 最近的 = 最该信的）。

本模块可以在任何机器上单独 import 与单元测试（tools/qr_eval.py 就是这样用的）。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import cv2
import numpy as np

# pyzbar 是可选兜底（apt: libzbar0；pip: pyzbar）。装不上不影响主流程。
try:
    from pyzbar import pyzbar  # type: ignore

    _HAVE_PYZBAR = True
except Exception:  # ImportError 或 libzbar 缺失时的动态链接错误
    pyzbar = None
    _HAVE_PYZBAR = False


@dataclass
class QrResult:
    """单个二维码的解码结果（与 ROS 无关的纯数据结构）。"""

    text: str                 # 完整解码文本，如 "RACKA_4X6M"
    center: tuple             # (u, v) 像素中心
    corners: list             # 4 个角点 [(u, v), ...]，顺时针
    stamp: float              # 时间戳（秒，wall clock）
    area_px: float = 0.0      # 四边形面积（像素²），用于"取最近/最大的码"
    source: str = "opencv"    # 哪个解码器出的结果：opencv / pyzbar


def _polygon_area(pts: np.ndarray) -> float:
    """鞋带公式算四边形面积。"""
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


class QrDecoder:
    """帧 → QrResult 列表。线程不安全（每路相机一个实例）。

    参数
    ----
    deep_every_n : int
        每 N 帧做一次"深度"尝试（2x 放大 + pyzbar）；1 = 每帧全开（最慢最稳）。
        找到码的帧总是立即返回，不会白跑后面的阶段。
    use_pyzbar : bool
        是否启用 pyzbar 兜底（没装则自动视为 False）。
    min_text_len : int
        短于该长度的解码结果丢弃（过滤噪声误检，比赛码最短是 "END"=3）。
    """

    def __init__(self, deep_every_n: int = 2, use_pyzbar: bool = True,
                 min_text_len: int = 3, upscale: float = 2.0):
        self.deep_every_n = max(1, int(deep_every_n))
        self.use_pyzbar = use_pyzbar and _HAVE_PYZBAR
        self.min_text_len = min_text_len
        self.upscale = upscale
        self._det = cv2.QRCodeDetector()
        self._frame_count = 0

    # ---------- 各解码阶段（返回 QrResult 列表） ----------

    def _decode_opencv(self, gray: np.ndarray, stamp: float) -> list:
        """OpenCV QRCodeDetector，多码模式，失败退回单码模式。"""
        try:
            ok, infos, pts, _ = self._det.detectAndDecodeMulti(gray)
        except cv2.error:
            return []
        results = []
        if ok and pts is not None:
            for text, quad in zip(infos, pts):
                if not text or len(text) < self.min_text_len:
                    continue
                quad = np.asarray(quad, dtype=np.float64).reshape(-1, 2)
                results.append(QrResult(
                    text=str(text),
                    center=(float(quad[:, 0].mean()), float(quad[:, 1].mean())),
                    corners=[(float(p[0]), float(p[1])) for p in quad],
                    stamp=stamp,
                    area_px=_polygon_area(quad),
                    source="opencv",
                ))
        if not results:  # 老版本/个别场景 multi 会漏，退回单码接口
            try:
                text, quad, _ = self._det.detectAndDecode(gray)
            except cv2.error:
                text, quad = "", None
            if text and len(text) >= self.min_text_len and quad is not None:
                quad = np.asarray(quad, dtype=np.float64).reshape(-1, 2)
                results.append(QrResult(
                    text=str(text),
                    center=(float(quad[:, 0].mean()), float(quad[:, 1].mean())),
                    corners=[(float(p[0]), float(p[1])) for p in quad],
                    stamp=stamp,
                    area_px=_polygon_area(quad),
                    source="opencv",
                ))
        return results

    def _decode_pyzbar(self, gray: np.ndarray, stamp: float, scale: float = 1.0) -> list:
        """pyzbar 兜底。scale>1 时角点坐标除回原尺度。"""
        results = []
        try:
            for sym in pyzbar.decode(gray):
                text = sym.data.decode("utf-8", errors="replace")
                if not text or len(text) < self.min_text_len:
                    continue
                pts = np.asarray(sym.polygon, dtype=np.float64).reshape(-1, 2) / scale
                results.append(QrResult(
                    text=text,
                    center=(float(pts[:, 0].mean()), float(pts[:, 1].mean())),
                    corners=[(float(p[0]), float(p[1])) for p in pts],
                    stamp=stamp,
                    area_px=_polygon_area(pts),
                    source="pyzbar",
                ))
        except Exception:
            return []
        return results

    # ---------- 对外主入口 ----------

    def decode(self, frame_bgr: np.ndarray, stamp: float | None = None) -> list:
        """解码一帧。识别不到返回 []；任何内部异常都吞掉（M3-8：无码不崩）。"""
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        if stamp is None:
            stamp = time.time()
        self._frame_count += 1
        # 第 1 帧即深度帧；之后每 N 帧一次（N=1 时每帧都深度）
        deep = ((self._frame_count - 1) % self.deep_every_n) == 0

        try:
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        except cv2.error:
            return []

        # 阶段 1：OpenCV 原图（每帧必跑，最便宜）
        results = self._decode_opencv(gray, stamp)
        if results:
            return self._dedupe_sort(results)

        # 以下为深度阶段：只在深度帧跑，避免空画面拖垮帧率
        if not deep:
            return []

        # 阶段 2：OpenCV 放大重试（小码 / 远码）
        if self.upscale and self.upscale > 1.0:
            big = cv2.resize(gray, None, fx=self.upscale, fy=self.upscale,
                             interpolation=cv2.INTER_LINEAR)
            results = self._decode_opencv(big, stamp)
            if results:
                for r in results:  # 坐标除回原尺度
                    r.corners = [(u / self.upscale, v / self.upscale) for u, v in r.corners]
                    r.center = (r.center[0] / self.upscale, r.center[1] / self.upscale)
                    r.area_px /= self.upscale * self.upscale
                return self._dedupe_sort(results)

        # 阶段 3/4：pyzbar 原图 + 放大
        if self.use_pyzbar:
            results = self._decode_pyzbar(gray, stamp)
            if results:
                return self._dedupe_sort(results)
            if self.upscale and self.upscale > 1.0:
                results = self._decode_pyzbar(big, stamp, scale=self.upscale)
                if results:
                    return self._dedupe_sort(results)

        return []

    def _dedupe_sort(self, results: list) -> list:
        """同一文本只留一个（优先 OpenCV），按面积降序。"""
        best: dict = {}
        for r in results:
            cur = best.get(r.text)
            if cur is None or (r.source == "opencv" and cur.source != "opencv") \
                    or (r.source == cur.source and r.area_px > cur.area_px):
                best[r.text] = r
        return sorted(best.values(), key=lambda r: r.area_px, reverse=True)

    @staticmethod
    def pyzbar_available() -> bool:
        return _HAVE_PYZBAR
