#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QR 取流预览 + 解码标注（M3-1/M3-4 验收工具，不需要 ROS）。

用法：
  # 树莓派接显示器：实时预览，框出二维码并显示解码文本
  python3 tools/qr_preview.py

  # 树莓派 SSH 无显示器（headless）：只打印 + 存截图
  python3 tools/qr_preview.py --no-window --save tests/qr_shots/

  # 固定对焦（0.4 m 停靠距离 → 2.5 屈光度）+ 锁曝光防拖影
  python3 tools/qr_preview.py --focus 2.5 --exposure-us 8000

  # 本机无相机：回放图片目录 / 视频调试
  python3 tools/qr_preview.py --path tests/qr_dataset/RACKA_4X6M

验收对应：M3-1（≥15fps 出帧）、M3-4（6 类码识别 + 截图归档 tests/qr_shots/）。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

# 让脚本在没 source ROS 环境时也能找到解码核心（core/voting/sources 均不依赖 ROS）
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ros2_ws", "src", "qr_perception"))

from qr_perception.core import QrDecoder          # noqa: E402
from qr_perception.sources import (CvSource, FileSource,     # noqa: E402
                                   FrameSourceError, PicameraSource)
from qr_perception.voting import MultiFrameVoter  # noqa: E402

GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
RED = (0, 0, 255)


def draw_results(frame, results):
    """在帧上画角点框 + 文本。"""
    for r in results:
        pts = np.array(r.corners, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(frame, [pts], True, GREEN, 2)
        u, v = int(r.center[0]), int(r.center[1])
        cv2.circle(frame, (u, v), 4, RED, -1)
        label = f"{r.text} [{r.source}]"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        ty = max(0, v - 10)
        cv2.rectangle(frame, (u - tw // 2 - 4, ty - th - 4), (u + tw // 2 + 4, ty), (0, 0, 0), -1)
        cv2.putText(frame, label, (u - tw // 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2)
    return frame


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--path", help="离线回放：图片目录 / 视频文件 / 单张图片")
    src.add_argument("--cv", type=int, metavar="IDX", help="用 V4L2 摄像头 IDX（默认自动：有 picamera2 用 Pi，否则试 cv:0）")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--focus", type=float, default=None, metavar="DIOPTERS",
                    help="固定对焦屈光度(1/米)，如 0.4m → 2.5；缺省=自动对焦")
    ap.add_argument("--exposure-us", type=int, default=None, help="锁曝光时间(µs)")
    ap.add_argument("--fps", type=float, default=10.0, help="回放节奏（仅 --path）")
    ap.add_argument("--confirm", type=int, default=2, help="投票确认帧数")
    ap.add_argument("--deep-every-n", type=int, default=2, help="深度重试间隔帧数")
    ap.add_argument("--no-window", action="store_true", help="无显示器模式（SSH）")
    ap.add_argument("--save", metavar="DIR", help="确认到新码时把标注帧存到 DIR（M3-4 截图归档）")
    ap.add_argument("--max-frames", type=int, default=0, help="处理 N 帧后退出（0=一直跑）")
    args = ap.parse_args()

    # ---- 建帧源 ----
    try:
        if args.path:
            source = FileSource(args.path, fps=args.fps)
        elif args.cv is not None:
            source = CvSource(args.cv, args.width, args.height)
        else:
            try:
                source = PicameraSource(args.width, args.height,
                                        focus_diopters=args.focus,
                                        exposure_time_us=args.exposure_us)
            except FrameSourceError:
                print("picamera2 不可用，回退 cv:0")
                source = CvSource(0, args.width, args.height)
        source.open()
    except FrameSourceError as e:
        print(f"[错误] {e}")
        return 1

    if args.save:
        os.makedirs(args.save, exist_ok=True)

    decoder = QrDecoder(deep_every_n=args.deep_every_n)
    voter = MultiFrameVoter(confirm_frames=args.confirm)
    print(f"帧源: {source} | pyzbar: {'on' if decoder.pyzbar_available() else 'off'} "
          f"| 窗口: {'off' if args.no_window else 'on'}"
          + (f" | 截图目录: {args.save}" if args.save else ""))
    print("按 Ctrl+C 退出" + ("" if args.no_window else "，窗口焦点在预览窗时按 q 退出"))

    n, t0, fps_disp = 0, time.time(), 0.0
    exit_code = 0
    try:
        while True:
            frame, stamp = source.capture()
            results = decoder.decode(frame, stamp)
            confirmed = voter.update([r.text for r in results])

            for text in confirmed:
                print(f"[确认] {text}")
                if args.save:
                    out = draw_results(frame.copy(), results)
                    safe = "".join(c if c.isalnum() else "_" for c in text)
                    path = os.path.join(args.save, f"{safe}_{int(stamp * 1000)}.png")
                    cv2.imwrite(path, out)
                    print(f"        已存 {path}")

            n += 1
            now = time.time()
            if now - t0 >= 1.0:
                fps_disp = n / (now - t0)
                n, t0 = 0, now

            if not args.no_window:
                out = draw_results(frame.copy(), results)
                cv2.putText(out, f"{fps_disp:5.1f} fps", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 2)
                cv2.imshow("qr_preview (q=quit)", out)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            if args.max_frames and decoder._frame_count >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    except FrameSourceError as e:
        print(f"[错误] {e}")
        exit_code = 1
    finally:
        source.close()
        if not args.no_window:
            cv2.destroyAllWindows()
        if fps_disp:
            print(f"最后实测帧率: {fps_disp:.1f} fps")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
