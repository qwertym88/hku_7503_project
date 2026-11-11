> **2026-11-11范围调整**：QR实现和六码截图保留；固定路线mission不依赖/qr。M3主责转为[场地测量、参数记录与验收证据](../../../docs/给队友看/分工与任务拆解.md)。下文的距离/锁焦等旧待办不再作为本轮主路门槛；正式码随机串不同仍须重拍提交图。

# qr_perception —— 相机 + QR 解码（M3）

## 架构（已实现）
```text
Camera Module 3 → picamera2 取流 ─┐
                                  ├─► 纯 Python 解码核心（OpenCV 首选 / pyzbar 兜底）─► QrResult
（ROS 无关，可图片回放调试）      ─┘              + 多帧投票去抖                    │
                                                     薄 ROS 封装 qr_node.py ───────┤→ /qr (String)
                                                                                 └→ /camera/image_raw (Image)
```

| 文件 | 职责 | 依赖 ROS? |
|---|---|---|
| `qr_perception/core.py` | 解码核心：`QrResult` 数据结构 + `QrDecoder`（OpenCV `detectAndDecodeMulti` → 2x 放大重试 → pyzbar 兜底，深度尝试每 N 帧一次控帧率） | ❌ |
| `qr_perception/voting.py` | 多帧投票去抖：攒够 `confirm_frames` 票才上报、消失 `absent_frames` 帧后允许重报 | ❌ |
| `qr_perception/sources.py` | 帧源：`PicameraSource`（picamera2，支持锁焦/锁曝光）/ `FileSource`（图片目录/视频回放）/ `CvSource`（USB 摄像头） | ❌ |
| `qr_perception/qr_node.py` | 薄封装：帧源→解码→投票→发布 `/qr`；不引 cv_bridge，手拼 `sensor_msgs/Image` | ✅ 唯一 |
| `test/test_voting.py` | 投票逻辑 8 个场景单元测试（`colcon test`） | ❌ |

## 接口（契约见《模块接口说明》）
- 发 `/qr`（`std_msgs/String`）：多帧确认后发完整码（如 `RACKA_4X6M`）；**没识别到不发**；同一码持续在视野内只发一次，消失后再出现会重发。
- 发 `/camera/image_raw`（`sensor_msgs/Image`，bgr8）：仅供 RViz/调试，**非契约接口**，且只在有订阅者时才发（省带宽）。

## 运行
```bash
# 树莓派真机（picamera2；锁焦 2.5 屈光度 ≈ 0.4m 停靠距离）
ros2 launch qr_perception qr_perception.launch.py focus_diopters:=2.5

# 本机无相机：图片目录回放（整条链路含 ROS 发布可测）
ros2 launch qr_perception qr_perception.launch.py source:=file file_path:=tests/qr_dataset/RACKA_4X6M

# 验证
ros2 topic echo /qr
```

## 依赖
- `python3-opencv`（apt，必装）
- `python3-picamera2`（apt，仅树莓派）
- `libzbar0` + `pyzbar`（可选兜底；没装自动跳过，日志会显示 `pyzbar=off`）

## 测试（不需要相机 / 不需要 ROS）
```bash
python3 tools/qr_eval.py tests/qr_dataset/        # 离线数据集回归（72 张，2026-11-10 实测 100%）
python3 tools/qr_preview.py --path tests/qr_dataset/START   # 预览 + 标注 + 截图
colcon test --packages-select qr_perception       # 单元测试
```

## 待办
- [x] M3-1 取流（Pi 实测 rpicam-vid 640x480，400 帧零断流 16.9fps ≥15 达标；picamera2/file/cv 备选）
- [x] M3-2 纯 Python 解码核心（OpenCV 首选，pyzbar 兜底；离线 72/72=100%）
- [x] M3-3 ROS 薄封装 → `/qr`（跨机验证：Pi 发布本机订阅，6 类码各恰好一条）
- [x] M3-4/9 六类码实拍识别 + 截图归档 `tests/qr_shots/`（2026-11-11，62 次确认，提交件可复验）
- [x] M3-5 多帧投票去抖（8 场景单测通过）
- [x] M3-11 离线图像测试集 + `qr_eval.py`（官方 6 码 + 合成难例 72 张）
- [ ] M3-6 近距离/角度识别率实测表（模板 `tests/qr_distance_test.md` 已备好，装车后按停靠位姿填）
- [ ] M3-7 按实际扫码距离锁焦（`focus_diopters`/`--focus` 参数已留好，装车后标定）

## 实测教训（2026-11-11，详见 `tests/TEST_LOG.md`）
- Ubuntu 24.04 无 `python3-picamera2` → rpicam-vid 帧源（零依赖）为主路
- rpicam-vid 默认 `-t 5000` 只跑 5 秒 → 必须 `-t 0`
- `deep_every_n=4`（Pi5 实测 15fps；=2 只有 13fps）

## 参考实现（已核实）
- [ros-drivers/zbar_ros](https://github.com/ros-drivers/zbar_ros)：**薄封装范本**，整包节点仅 162 行，CI 覆盖 jazzy。
- [ZengWenJian123/qrcode_recognition](https://github.com/ZengWenJian123/qrcode_recognition)：**50 行纯 Python** 解码范本。
