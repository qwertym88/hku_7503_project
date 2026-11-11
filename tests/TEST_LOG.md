# 测试记录表

> 规则见《单模块测试说明》末尾：**没有证据（截图/日志/bag）的测试等于没测。**
> 每次测试追加一行；`证据` 列写仓库内路径或终端输出摘要。

| 日期 | 模块 | 测试项 | 结果 | 证据 | 测试人 |
|---|---|---|---|---|---|
| 2026-11-10 | M3 | 离线解码回归：官方 6 码 + 合成难例 72 张（旋转/倾斜/模糊/明暗/低分辨率/噪声） | 72/72 通过（含 18 张极端样例） | `tests/qr_dataset/`；`python3 tools/qr_eval.py tests/qr_dataset/` 输出 100% PASS | M3 |
| 2026-11-10 | M3 | 多帧投票去抖单元测试（单帧误读不上报 / 不刷屏 / 消失重报 / 间歇识别 / 多码择优） | 8/8 通过 | `colcon test --packages-select qr_perception`：Ran 8 tests, OK | M3 |
| 2026-11-10 | M3 | ROS 薄封装：图片目录回放 6 类码 → `/qr`（本机，无相机） | 6 类码各发布**恰好一次**（回放循环 3+ 圈无刷屏）；`ros2 topic echo /qr` 收到 `std_msgs/String` | `ros2 launch qr_perception qr_perception.launch.py source:=file file_path:=/tmp/qr_replay` 节点日志 | M3 |
| 2026-11-10 | M3 | `/camera/image_raw` 内部图像流（bgr8，frame_id=camera） | `ros2 topic echo /camera/image_raw --once` 正常出图 | 终端输出 header.frame_id=camera | M3 |
| 2026-11-10 | M3 | 打印样张可扫性（`tests/qr_print_sheet.png` 整张一次解出 6 码） | 6/6 | `qr_make_dataset.py --sheet` + 解码验证脚本输出 | M3 |
| 2026-11-11 | M3 | **Pi 实拍**：rpicam-vid 取流 640x480@30，400 帧连续流稳定性 | 400 帧零断流，整循环 **16.9 fps**（≥15 达标，M3-1） | Pi 终端输出 `最后实测帧率: 16.9 fps`；关键修复：rpicam-vid 默认 `-t 5000` 只跑 5s，必须 `-t 0` | M3 |
| 2026-11-11 | M3 | **Pi 实拍**：6 类 QR 实机识别（手机/打印样张举码，多轮） | 6/6 全部识别，每码多轮确认；投票去抖正常（同码不刷屏、重现重报） | `tests/qr_shots/*.png`（6 张提交件，标注后仍可复验解码）；会话日志 62 次截图/确认 | M3 |
| 2026-11-11 | M3 | **跨机全栈**：Pi 跑节点（回放源）→ 本机 `ros2 topic echo /qr`（ROS_DOMAIN_ID=42，局域网 DDS） | 6 类码各恰好 1 条消息，跨机去抖正确 | 本机终端输出（data: START … data: END） | M3 |
| 2026-11-11 | M3 | Pi 离线回归（Pi 的 cv2 4.6 环境） | 72/72 = 100% PASS | `tools/setup_pi_qr.sh` 第 4 步输出 | M3 |

## 已知问题与注意事项（M3 实测教训）

- **Ubuntu 24.04 没有 `python3-picamera2`**（Pi OS 专属包）。取流用 `rpicam` 帧源（rpicam-vid 子进程，零依赖）；
  想走 picamera2 需 `sudo apt install python3-libcamera` + `pip install --user --break-system-packages picamera2==0.3.10`（版本必须配 noble 的 libcamera 0.2.0）。
- **rpicam-vid 默认 `-t 5000`（只跑 5 秒）**，长跑必须加 `-t 0`。曾误判为"管道背压卡死"。
- Pi5 + Ubuntu 的相机管线长跑（>10 分钟）偶发 `imx708_stop_streaming failed`，重启取流进程即恢复；
  节点已带断流自动重连（3 次）。比赛单轮 3 分钟，风险低。
- pyzbar 兜底在 Pi 上暂不可用（pip 装了但缺 `libzbar0` 系统库，需 sudo）；OpenCV 主路 100% 覆盖测试集，不阻塞。
- `deep_every_n` 实测（Pi5，640x480 空画面）：2→13fps，4→15fps，已设默认 4。识别到码时总是走轻量路径（19ms），确认延迟 ~2 帧。
