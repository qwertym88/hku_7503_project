# -*- coding: utf-8 -*-
"""qr_perception —— M3 感知包：picamera2 取流 + 纯 Python QR 解码 + 薄 ROS 封装 → /qr。

core.py / voting.py / sources.py 均不依赖 ROS，可单独 import 做离线测试；
qr_node.py 是唯一的 ROS 文件（薄封装）。
"""
