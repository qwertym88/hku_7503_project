# -*- coding: utf-8 -*-
"""ROS 薄封装（M3-3）：帧源 → 解码核心 → 投票 → /qr。

刻意保持"薄"：本文件不做任何解码/去抖算法，只做参数接线与消息发布，
算法全部在 ROS 无关的 core.py / voting.py / sources.py 里（换平台可整体搬走）。

话题（接口契约见 docs/给队友看/模块接口说明.md）：
  发 /qr               std_msgs/String    确认后的完整码文本；没识别到【不发】
  发 /camera/image_raw sensor_msgs/Image  仅供感知自用与 RViz（非契约接口，可关）

常用参数（ros2 run qr_perception qr_perception --ros-args -p ...）：
  source: picamera2 | rpicam | file | cv  帧源（默认 picamera2；rpicam 用官方 rpicam-vid，
                                          Ubuntu 24.04 无需装 picamera2；本机无相机用 file 回放）
  file_path: str                    source=file 时的图片目录/视频/单图
  width/height: 640/480             取流分辨率
  focus_diopters: double            固定对焦屈光度（1/米）；null=自动对焦
  exposure_time_us: int             锁曝光（µs）；null=自动曝光
  confirm_frames: 2                 连续确认帧数（多帧投票）
  absent_frames: 15                 消失多少帧后允许重报同一码
  publish_image: true               是否发 /camera/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import sources
from .core import QrDecoder
from .voting import MultiFrameVoter


def bgr_to_image(frame, frame_id: str, stamp) -> Image:
    """BGR ndarray → sensor_msgs/Image（bgr8）。不引 cv_bridge，少一个依赖。"""
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height, msg.width = frame.shape[0], frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = frame.reshape(-1).tobytes()
    return msg


class QrPerceptionNode(Node):

    def __init__(self):
        super().__init__("qr_perception")

        # ---- 参数（全部有默认值，直接 ros2 run 也能跑） ----
        self.declare_parameter("source", "picamera2")
        self.declare_parameter("file_path", "")
        self.declare_parameter("replay_fps", 10.0)
        self.declare_parameter("replay_loop", True)
        self.declare_parameter("cv_index", 0)
        self.declare_parameter("camera_framerate", 30)    # rpicam-vid 取流帧率
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("focus_diopters", 0.0)      # <=0 视为不锁焦
        self.declare_parameter("exposure_time_us", 0)      # <=0 视为自动曝光
        self.declare_parameter("deep_every_n", 2)
        self.declare_parameter("use_pyzbar", True)
        self.declare_parameter("confirm_frames", 2)
        self.declare_parameter("absent_frames", 15)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("target_fps", 0.0)          # >0 时限制循环频率

        p = lambda name: self.get_parameter(name).value
        kind = str(p("source"))
        common = {"width": int(p("width")), "height": int(p("height"))}
        if kind in ("file", "replay"):
            src = sources.FileSource(path=str(p("file_path")), fps=float(p("replay_fps")),
                                     loop=bool(p("replay_loop")))
        elif kind in ("cv", "webcam", "usb"):
            src = sources.CvSource(index=int(p("cv_index")), **common)
        elif kind in ("rpicam", "rpicam-vid", "rpicamvid"):
            focus = float(p("focus_diopters"))
            exposure = int(p("exposure_time_us"))
            src = sources.RpicamVidSource(
                framerate=int(p("camera_framerate")),
                **common,
                focus_diopters=focus if focus > 0 else None,
                exposure_time_us=exposure if exposure > 0 else None)
        else:
            focus = float(p("focus_diopters"))
            exposure = int(p("exposure_time_us"))
            src = sources.PicameraSource(
                **common,
                focus_diopters=focus if focus > 0 else None,
                exposure_time_us=exposure if exposure > 0 else None)

        self.decoder = QrDecoder(deep_every_n=int(p("deep_every_n")),
                                 use_pyzbar=bool(p("use_pyzbar")))
        self.voter = MultiFrameVoter(confirm_frames=int(p("confirm_frames")),
                                     absent_frames=int(p("absent_frames")))
        self.publish_image = bool(p("publish_image"))
        self.frame_id = str(p("frame_id"))
        self.target_fps = float(p("target_fps"))

        self.qr_pub = self.create_publisher(String, "/qr", 10)
        self.img_pub = self.create_publisher(Image, "/camera/image_raw", 10) \
            if self.publish_image else None

        self._src = src
        self._n_frames = 0
        self._n_decoded = 0
        self._t_last_log = self.get_clock().now()

    def on_frame(self, frame, stamp_wall):
        """处理一帧：解码 → 投票 → 发布。返回本帧确认的文本列表。"""
        results = self.decoder.decode(frame, stamp_wall)
        confirmed = self.voter.update([r.text for r in results])

        for text in confirmed:
            msg = String()
            msg.data = text
            self.qr_pub.publish(msg)
            self.get_logger().info(f"/qr -> '{text}'")

        if self.img_pub is not None and self.img_pub.get_subscription_count() > 0:
            self.img_pub.publish(bgr_to_image(frame, self.frame_id,
                                              self.get_clock().now().to_msg()))

        self._n_frames += 1
        if results:
            self._n_decoded += 1
        now = self.get_clock().now()
        if (now - self._t_last_log).nanoseconds > 10_000_000_000:  # 每 10s 一行状态
            rate = self._n_decoded / max(1, self._n_frames) * 100.0
            self.get_logger().info(
                f"状态: {self._n_frames} 帧, {rate:.0f}% 帧有码, "
                f"pyzbar={'on' if self.decoder.pyzbar_available() else 'off'}")
            self._n_frames = self._n_decoded = 0
            self._t_last_log = now
        return confirmed

    def run(self):
        self.get_logger().info(f"帧源: {self._src}")
        self._src.open()
        period = 1.0 / self.target_fps if self.target_fps > 0 else 0.0
        reopen_left = 3  # 断流自动重连次数（瞬时故障不杀节点）
        try:
            while rclpy.ok():
                try:
                    frame, stamp = self._src.capture()
                except sources.FrameSourceError as e:
                    if "回放结束" in str(e):
                        self.get_logger().info("回放结束，节点退出")
                        break
                    if reopen_left <= 0:
                        raise
                    reopen_left -= 1
                    self.get_logger().warn(f"取流中断，尝试重连({3 - reopen_left}/3): {e}")
                    import time
                    time.sleep(2.0)
                    self._src.open()  # 各帧源的 open() 幂等
                    self.get_logger().info("重连成功")
                    continue
                reopen_left = 3  # 成功跑一帧后恢复重连配额
                self.on_frame(frame, stamp)
                if period:
                    import time
                    time.sleep(period)
        except sources.FrameSourceError as e:
            self.get_logger().error(f"取流失败（重连后仍不行）: {e}")
        finally:
            self._src.close()


def main(args=None):
    rclpy.init(args=args)
    node = QrPerceptionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
