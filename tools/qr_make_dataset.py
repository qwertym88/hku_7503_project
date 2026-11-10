#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从官方样例图生成离线难例数据集（M3-11 采集工具，不需要相机）。

输入：tests/qr_dataset/<类别>/official.png（从需求 PDF 附录提取的 6 张官方样例）
输出：同目录下的合成变体 —— 旋转 / 透视倾斜 / 模糊 / 运动拖影 / 明暗 / 低分辨率 / 噪声，
      模拟"距离 / 角度 / 光照 / 行进模糊"四个维度（对应《仿真与离线测试方案》§2.2）。

  常规变体（文件名不带前缀）：合格解码器应当 100% 通过 —— 用作回归基线。
  极端变体（hard_ 前缀）：只做参考统计，不计入 qr_eval.py 的达标率。

用法：
  python3 tools/qr_make_dataset.py                 # 生成到 tests/qr_dataset/
  python3 tools/qr_make_dataset.py --out 其他目录 --seed 7
"""

from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

CANVAS_W, CANVAS_H = 640, 480   # 模拟相机帧大小


def paste_on_canvas(qr: np.ndarray, w: int = CANVAS_W, h: int = CANVAS_H) -> np.ndarray:
    """把（变换后的）QR 贴到模拟相机帧的画布中央，四周留白。"""
    canvas = np.full((h, w, 3), 235, dtype=np.uint8)  # 浅灰背景，模拟场地地面
    hh, ww = qr.shape[:2]
    y0, x0 = max(0, (h - hh) // 2), max(0, (w - ww) // 2)
    hh, ww = min(hh, h), min(ww, w)
    canvas[y0:y0 + hh, x0:x0 + ww] = qr[:hh, :ww]
    return canvas


def rotate(img: np.ndarray, deg: float) -> np.ndarray:
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), deg, 1.0)
    cos, sin = abs(M[0, 0]), abs(M[0, 1])
    nw, nh = int(h * sin + w * cos), int(h * cos + w * sin)
    M[0, 2] += nw / 2 - w / 2
    M[1, 2] += nh / 2 - h / 2
    return cv2.warpAffine(img, M, (nw, nh), borderValue=(255, 255, 255))


def perspective(img: np.ndarray, strength: float) -> np.ndarray:
    """把矩形 QR 的四角按 strength 比例内缩一角，模拟斜视角。"""
    h, w = img.shape[:2]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    off = strength * min(w, h)
    dst = np.float32([[0, 0], [w, 0], [w - off, h], [off * 0.3, h]])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, M, (w, h), borderValue=(255, 255, 255))


def motion_blur(img: np.ndarray, k: int) -> np.ndarray:
    """水平方向运动模糊（模拟行进拖影）。"""
    kernel = np.zeros((k, k), np.float32)
    kernel[k // 2, :] = 1.0 / k
    return cv2.filter2D(img, -1, kernel)


def rescale(img: np.ndarray, factor: float) -> np.ndarray:
    """缩小再放大 —— 模拟"更远距离时码占的像素更少"。"""
    h, w = img.shape[:2]
    small = cv2.resize(img, (max(8, int(w * factor)), max(8, int(h * factor))),
                       interpolation=cv2.INTER_AREA)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)


def add_noise(img: np.ndarray, sigma: int) -> np.ndarray:
    rng = np.random.default_rng(42)
    noise = rng.normal(0, sigma, img.shape).astype(np.float32)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def variants(official: np.ndarray) -> dict:
    """生成 变体名 → 图。常规 8 个 + 极端 3 个。"""
    out = {}
    out["rot10.png"] = paste_on_canvas(rotate(official, 10))
    out["rotm10.png"] = paste_on_canvas(rotate(official, -10))
    out["tilt.png"] = paste_on_canvas(perspective(official, 0.18))
    out["blur3.png"] = paste_on_canvas(cv2.GaussianBlur(official, (3, 3), 0))
    out["motion8.png"] = paste_on_canvas(motion_blur(official, 8))
    out["dark05.png"] = paste_on_canvas(np.clip(official * 0.5, 0, 255).astype(np.uint8))
    out["bright15.png"] = paste_on_canvas(np.clip(official * 1.5, 0, 255).astype(np.uint8))
    out["small05.png"] = paste_on_canvas(rescale(official, 0.5))
    # ---- 极端样例（hard_ 前缀，qr_eval 单独统计）----
    out["hard_tilt35.png"] = paste_on_canvas(perspective(official, 0.35))
    out["hard_small025.png"] = paste_on_canvas(rescale(official, 0.25))
    out["hard_blur7.png"] = paste_on_canvas(cv2.GaussianBlur(official, (7, 7), 0))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join("tests", "qr_dataset"),
                    help="数据集根目录（默认 tests/qr_dataset/）")
    ap.add_argument("--overwrite", action="store_true", help="覆盖已存在的变体")
    args = ap.parse_args()

    root = args.out
    if not os.path.isdir(root):
        print(f"[错误] {root} 不存在（需要先放入 <类别>/official.png）")
        return 2

    classes = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
    if not classes:
        print(f"[错误] {root} 下没有类别目录")
        return 2

    n_written = 0
    for cls in classes:
        src = os.path.join(root, cls, "official.png")
        if not os.path.isfile(src):
            print(f"[跳过] {cls}/official.png 不存在")
            continue
        official = cv2.imread(src)
        if official is None:
            print(f"[跳过] {cls}/official.png 读不了")
            continue
        for name, img in variants(official).items():
            dst = os.path.join(root, cls, name)
            if os.path.exists(dst) and not args.overwrite:
                continue
            cv2.imwrite(dst, img)
            n_written += 1
        print(f"[OK] {cls}: 变体已生成")
    print(f"\n共写入 {n_written} 张变体。评测：python3 tools/qr_eval.py {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
