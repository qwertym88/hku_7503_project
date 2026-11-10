# -*- coding: utf-8 -*-
"""多帧投票去抖（M3-5）—— 纯 Python，不依赖 ROS。

要解决的两个问题：
  1. 单帧误码不上报：同一个文本要累计 confirm_frames 次 sighting 才确认。
     计数只累加不清零（除非该文本消失了 absent_frames 帧）—— 这样间歇性识别
     （比如深度重试每 2 帧才跑一次、码在画面边缘时隐时现）也能攒够票；
     而孤立的单帧噪声误读永远到不了 confirm_frames。
     注：若同一个错误文本被反复解码（系统性误读，如画面里有张海报），
     投票帮不上忙也不该帮——那对系统来说就是"看到了这个码"。
  2. 同一个码不刷屏：确认过一次后进入静默；连续 absent_frames 帧没再看到它，
     静默解除（下次攒够票会重新上报）—— 覆盖"同一条码扫两遍"的场景
     （比如取货失败回退后重新对准同一个货架）。
"""

from __future__ import annotations


class MultiFrameVoter:
    """texts-in → confirmed-texts-out 的有状态滤波器。"""

    def __init__(self, confirm_frames: int = 2, absent_frames: int = 15):
        """
        参数
        ----
        confirm_frames : 累计票数达到该值才确认（1 = 不投票，来一个发一个）。
        absent_frames  : 该码连续多少帧没出现后，清零其票数并解除静默（允许重报）。
        """
        self.confirm_frames = max(1, int(confirm_frames))
        self.absent_frames = max(1, int(absent_frames))
        self._votes: dict = {}        # text -> 累计票数
        self._absent: dict = {}       # text -> 连续没看到的帧数
        self._confirmed: set = set()  # 已确认且仍在静默中的文本
        self._frame = 0

    def update(self, texts: list) -> list:
        """喂入一帧的全部解码文本，返回本帧新确认（应当发布）的文本列表。

        通常返回 []（没码 / 票不够 / 已在静默期）；确认时返回 [text]。
        """
        self._frame += 1
        seen = set(texts)

        # 1) 计票 + 缺席计数
        for t in seen:
            self._votes[t] = self._votes.get(t, 0) + 1
            self._absent[t] = 0
        for t in list(self._votes.keys()):
            if t not in seen:
                self._absent[t] = self._absent.get(t, 0) + 1
                # 消失够久：票数清零 + 静默解除（两个语义共用同一个阈值）
                if self._absent[t] >= self.absent_frames:
                    self._votes.pop(t, None)
                    self._absent.pop(t, None)
                    self._confirmed.discard(t)

        # 2) 确认：票够且不在静默期。一帧最多确认一个（票最高的）。
        candidates = [
            (self._votes[t], t) for t in seen
            if self._votes.get(t, 0) >= self.confirm_frames and t not in self._confirmed
        ]
        if not candidates:
            return []
        _, winner = max(candidates)
        self._confirmed.add(winner)
        return [winner]

    # ---------- 调试辅助 ----------

    def state(self) -> dict:
        """当前内部状态（给日志/调试用）。"""
        return {
            "frame": self._frame,
            "votes": dict(self._votes),
            "confirmed": set(self._confirmed),
        }
