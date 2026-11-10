#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线图像测试集批量评测（M3-11 / M3-6 回归工具，不需要相机、不需要 ROS）。

数据集布局（目录名 = 该目录下所有图的期望解码文本）：
  tests/qr_dataset/
  ├─ START/           official.png rot10.png blur3.png ...
  ├─ RACKA_4X6M/      ...
  ├─ ...
  └─ END/

  文件名以 hard_ 开头的是"已知很难"的极端样例：单独统计、不计入达标率
  （它们用来回答"极限在哪"，不是用来卡回归的）。

用法：
  python3 tools/qr_eval.py tests/qr_dataset/               # 期望 ≥95% 才退出码 0
  python3 tools/qr_eval.py tests/qr_dataset/ -v            # 打印每张图的结果
  python3 tools/qr_eval.py tests/qr_dataset/ --threshold 1.0

改了解码参数后跑一遍，达标率变差就说明改坏了 —— 这就是 M3 的"仿真替代品"。
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ros2_ws", "src", "qr_perception"))

from qr_perception.core import QrDecoder  # noqa: E402

IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".bmp")


def eval_dataset(root: str, decoder: QrDecoder, verbose: bool):
    classes = sorted(d for d in os.listdir(root)
                     if os.path.isdir(os.path.join(root, d)))
    if not classes:
        print(f"[错误] {root} 下没有类别目录")
        return None

    rows, total = [], {"ok": 0, "wrong": 0, "miss": 0}
    hard = {"ok": 0, "wrong": 0, "miss": 0}
    for cls in classes:
        d = os.path.join(root, cls)
        files = sorted(f for f in os.listdir(d) if f.lower().endswith(IMAGE_EXTS))
        ok = wrong = miss = 0
        for f in files:
            path = os.path.join(d, f)
            frame = cv2.imread(path)
            results = decoder.decode(frame) if frame is not None else []
            texts = [r.text for r in results]
            if cls in texts:
                verdict = "ok"
                ok += 1
            elif texts:
                verdict = f"wrong(={texts[0]!r})"
                wrong += 1
            else:
                verdict = "miss"
                miss += 1
            is_hard = f.startswith("hard_")
            bucket = hard if is_hard else total
            bucket[verdict.split("(")[0]] += 1
            if verbose:
                print(f"  {'[hard]' if is_hard else '      '} {cls}/{f:<28} {verdict}")
        rows.append((cls, len(files), ok, wrong, miss))

    return rows, total, hard


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", help="数据集根目录，如 tests/qr_dataset/")
    ap.add_argument("--threshold", type=float, default=0.95,
                    help="达标率阈值（默认 0.95，对应 M3-6 的 ≥95%%）")
    ap.add_argument("--deep-every-n", type=int, default=1,
                    help="离线评测默认每帧全流水线（=1），最准")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印每张图的结果")
    args = ap.parse_args()

    if not os.path.isdir(args.dataset):
        print(f"[错误] 数据集目录不存在: {args.dataset}")
        return 2

    decoder = QrDecoder(deep_every_n=args.deep_every_n)
    print(f"评测 {args.dataset} | pyzbar: {'on' if decoder.pyzbar_available() else 'off'}\n")

    ret = eval_dataset(args.dataset, decoder, args.verbose)
    if ret is None:
        return 2
    rows, total, hard = ret

    print(f"{'类别':<14} {'张数':>4} {'正确':>4} {'误读':>4} {'漏读':>4} {'通过率':>8}")
    print("-" * 50)
    for cls, n, ok, wrong, miss in rows:
        rate = ok / n * 100 if n else 0.0
        print(f"{cls:<14} {n:>4} {ok:>4} {wrong:>4} {miss:>4} {rate:>7.1f}%")
    print("-" * 50)

    n_all = total["ok"] + total["wrong"] + total["miss"]
    rate = total["ok"] / n_all * 100 if n_all else 0.0
    print(f"常规样例合计   {n_all:>4} {total['ok']:>4} {total['wrong']:>4} {total['miss']:>4} {rate:>7.1f}%")
    n_hard = hard["ok"] + hard["wrong"] + hard["miss"]
    if n_hard:
        hrate = hard["ok"] / n_hard * 100
        print(f"极端样例(参考) {n_hard:>4} {hard['ok']:>4} {hard['wrong']:>4} {hard['miss']:>4} {hrate:>7.1f}%")

    passed = rate >= args.threshold * 100
    print(f"\n达标率 {rate:.1f}%（阈值 {args.threshold * 100:.0f}%）: {'✅ PASS' if passed else '❌ FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
