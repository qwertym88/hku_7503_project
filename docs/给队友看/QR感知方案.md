# QR 感知方案（M3 必读）

> 结论先行：
> 1. **你的判断是对的，而且有成熟先例**——"核心算法独立于 ROS + 外层薄封装"正是主流做法。
> 2. **不必用 `camera_ros`**：Pi 5 + Camera Module 3 用 **`picamera2`** 就能独立取流。
> 3. 推荐架构：**纯 Python 核心（picamera2 + OpenCV/pyzbar）→ ~50 行 ROS 封装 → `/qr`**。

---

## 1. 参考项目（均已实际 clone 核实）

### 1.1 薄封装范本 ⭐

| 项目 | ★ | 语言 | 是什么 | 对我们的价值 |
|---|---|---|---|---|
| [ros-drivers/zbar_ros](https://github.com/ros-drivers/zbar_ros) | 64 | C++ | "Basic ROS 2 **wrapper** for the zbar library"，输入 `image` 话题、输出 `barcode` 话题 | **"核心库 + 薄包装"的官方范本**：整个节点只有 **162 行**；**CI 覆盖 jazzy** |
| [Venom-Algorithm/zbar_ros](https://github.com/Venom-Algorithm/zbar_ros) | 0 | C++ | 感知层单包：输入 `Image`、输出 `BarcodeDetections`，**不发布 TF**，header 继承 `stamp/frame_id` | 更贴近我们：**还带 `target_pose_estimator.cpp`**（二维码位姿估计） |
| [ZengWenJian123/qrcode_recognition](https://github.com/ZengWenJian123/qrcode_recognition) | 41 | Python | **50 行**纯 Python：`cv2.VideoCapture(0)` + `pyzbar.decode()`，还给出**中心点与最小外接矩形** | **"ROS 无关核心"的极简范本**，可直接借鉴结构 |

### 1.2 RoboMaster 视觉生态

| 项目 | ★ | 语言 | 是什么 |
|---|---|---|---|
| [chenjunnn/rm_vision](https://github.com/chenjunnn/rm_vision) | 238 | Python | **RoboMaster 视觉 ROS 2 框架**（`bringup`/`config`/`launch` 分层） |
| [FaterYU/rm_vision](https://github.com/FaterYU/rm_vision) | 181 | Python | RoboMaster 视觉框架（另一支） |
| [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) | 698 | C | 同济 SuperPower 25 赛季视觉框架 |
| [Bright-Crest/rm_eng_2024](https://github.com/Bright-Crest/rm_eng_2024) | 2 | C | **RoboMaster 工程机器人 ROS2 工作区** |

> RoboMaster 的工程机器人赛季任务里包含现场识别码/标签的环节，这也是"移动平台快速扫码"这个场景的来源。*(规则细节以官方规则手册为准，本次未核对原始规则文本。)*

### 1.3 解码库与算法

| 项目 | ★ | 说明 |
|---|---|---|
| [NaturalHistoryMuseum/pyzbar](https://github.com/NaturalHistoryMuseum/pyzbar) | 833 | `libzbar` 的 Python 绑定，QR + 一维码，返回 `polygon` 角点 |
| [Eric-Canas/QReader](https://github.com/Eric-Canas/QReader) | 432 | 面向**难码/模糊码**的鲁棒读取（基于 YOLO 检测 + 解码，偏重） |
| [Asadullah-Dal17/QR-detection-and-Distance-Estimation](https://github.com/Asadullah-Dal17/QR-detection-and-Distance-Estimation) | 51 | QR 检测 + **距离估计**（可用于对准） |
| [jenly1314/WeChatQRCode](https://github.com/jenly1314/WeChatQRCode) | 896 | OpenCV `wechat_qrcode` 引擎的移植（说明该引擎很流行） |

---

## 2. 架构：核心与 ROS 解耦

```
┌───────────────────────── 纯 Python，无 ROS ─────────────────────────┐
│  picamera2 取流 → 解码(OpenCV/pyzbar) → QrResult(text, corners, ...) │
│         ↑ 可离线单测、可用图片回放、换相机/换库都不动 ROS           │
└────────────────────────────────┬────────────────────────────────────┘
                                 │  ~50 行薄封装
                    ┌────────────▼─────────────┐
                    │  ROS 节点：发布 /qr      │
                    └──────────────────────────┘
```

**三条好处**：① 没硬件也能用图/视频调试；② 解码算法可单独写单元测试；③ 万一要换解码库，只改核心不动 ROS。

> 这正是 `zbar_ros` 的形态（zbar 是核心，162 行节点是壳），也是 `qrcode_recognition` 的形态（50 行纯 Python）。

---

## 3. 相机取流：`picamera2` vs `camera_ros`

| | **picamera2**（推荐） | camera_ros |
|---|---|---|
| 安装 | `sudo apt install python3-picamera2`（Pi OS **预装**） | 需源码编译 `libcamera` + `camera_ros` |
| 形态 | **纯 Python 库，无 ROS** | **ROS 节点**，天生耦合 ROS |
| 控制 | 可直接设曝光/增益/**固定对焦** | 需通过 ROS 参数绕 |
| 出错面 | 小 | 编译链 + ROS 版本 + 相机时序，坑多 |
| 适用 | **我们的核心层** | 已决定不用 |

**说明**：`picamera2` 是树莓派官方的 libcamera Python API，**不支持 Buster 及更早**的系统，Pi OS Lite 需手动装；我们的 Pi 5 用 64 位 Pi OS（非 Lite）即可直接用。

**结论：M3 用 `picamera2`，不引入 `camera_ros`。** 这也顺带消掉了原计划里"M3-1 camera_ros 编译失败"这个高风险项。

---

## 4. 解码库对比

| 方案 | 依赖 | 优点 | 缺点 | 建议 |
|---|---|---|---|---|
| **OpenCV `QRCodeDetector`** | 只需 `python3-opencv` | 零额外依赖、有 `detectAndDecodeMulti`、返回角点 | 对模糊/畸变一般 | **首选** |
| **`pyzbar`** | `libzbar0` + pip | 快、稳、返回 `polygon` | 中文/多码一般 | **次选/交叉验证** |
| `wechat_qrcode` | `opencv-contrib` | 难码强 | 重、额外依赖 | 精度不够时再上 |
| `QReader` | 较重（YOLO） | 最难码 | 依赖重、树莓派上慢 | 不用 |

**推荐：先 OpenCV，再 pyzbar 兜底；两个都不行再考虑 wechat_qrcode。**（前两个都零/低成本）

---

## 5. 移动平台"快速扫码"的工程要点

| # | 要点 | 做法 |
|---|---|---|
| 1 | **固定对焦** | Camera Module 3 是自动对焦，扫近距码时会来回"拉风箱"。**按实际扫码距离固定 focus**，既快又稳 |
| 2 | **降曝光防运动模糊** | 车速 ~0.2–0.3 m/s，曝光时间要短；宁可暗一点也不要拖影 |
| 3 | **分辨率取舍** | 检测用 640×480 或 1280×720 即可；不要直接上全分辨率，帧率会掉 |
| 4 | **多帧投票去抖** | 连续 N 帧解码一致才上报（`/qr` 去重就靠这个），单帧误码不上报 |
| 5 | **ROI / 跳帧** | 只在画面中下部（货架可能出现区域）检测；或每 2 帧检测一次 |
| 6 | **灰度化** | 解码前转灰度，省一半算力 |
| 7 | **（可选）测距对准** | 用 QR 像素尺寸估距离，低速微调停靠——**保底不需要**，坐标已知 |
| 8 | **照明** | 场地灯光/反光未知，先实测；必要时补光 |

---

## 6. 接口与封装

**核心层返回一个简单结构**（不涉及 ROS）：

```python
@dataclass
class QrResult:
    text: str          # "RACKA_4X6M"
    center: tuple      # (u, v) 像素中心，可用于对准
    corners: list      # 四个角点
    stamp: float       # 时间戳
```

**ROS 封装节点**（~50 行）：把 `text` 发到 `/qr`（`std_msgs/String`），保持我们在《模块接口说明》里的契约。

> ⚠️ **若想直接用 `zbar_ros`**：它发布的 `barcode` 话题用的是**自定义消息** `zbar_ros_interfaces/Symbol`，与我们的 `/qr`（`String`）契约不符。
> 两个选择：① **自己写 50 行 String 节点**（推荐，最简单）；② 用 `zbar_ros` 跑通后再写一个小 shim 转成 `String`，但那就多了一个自定义消息依赖。
> **保底：选 ①。**

---

## 7. M3 任务重构（替换原 M3-1/M3-2）

| 任务 | 周次 | 交付物 | 验收 | 变化 |
|---|---|---|---|---|
| **M3-1 取流（picamera2）** | W1 | 能稳定出帧的脚本 | ≥15 fps，无丢帧 | 🔄 **原计划是编译 camera_ros，改为 picamera2** |
| **M3-2 核心解码（纯 Python）** | W1–W2 | `QrResult` 解码函数 | 用**图片回放**即可测，不依赖 ROS | 🔄 明确"ROS 无关" |
| M3-3 ROS 薄封装 → `/qr` | W2 | ~50 行节点，发 `String` | `ros2 topic echo /qr` 有输出 | 结构明确 |
| M3-4 6 类 QR 识别 + 截图 | W2 | 6 张截图归档 | 100% 解码 | — |
| M3-5 多帧投票去抖 | W3 | 去重逻辑 | 同一码不刷屏 | — |
| M3-6 距离/角度识别率 | W3–W4 | 测试表 | 常见停靠位姿 ≥95% | — |
| M3-7 相机安装配合（含**固定对焦**） | W3 | 视场/焦距确认 | 与 M1-6 支架匹配 | ➕ 固定对焦 |
| M3-8 健壮性（光照/模糊） | W4 | 失败返回空 | 无码不崩 | — |
| M3-9 截图集合交付 | W5 | 6 张截图 | 符合提交要求 | — |
| **M3-10（可选）** 试 `pyzbar`/`zbar_ros` 作为备选 | W2 | 对比记录 | 两条路都跑通 | ➕ 新增 |

---

## 8. 待确认

| # | 事项 | 找谁 |
|---|---|---|
| 1 | 二维码在货架上的**尺寸、位置、朝向** | 老师/需求 |
| 2 | 相机安装高度与俯角（决定固定焦距） | M1 + M3 |
| 3 | 场地光照条件（是否反光、是否需补光） | 现场实测 |
| 4 | 扫码时的**典型停靠距离** | M4 停靠点标定后 |

---

## 9. 参考链接

- zbar_ros（薄封装范本，含 jazzy CI）：https://github.com/ros-drivers/zbar_ros
- Venom zbar_ros（感知层 + 位姿估计）：https://github.com/Venom-Algorithm/zbar_ros
- 纯 Python 极简范本：https://github.com/ZengWenJian123/qrcode_recognition
- RoboMaster ROS2 视觉框架：https://github.com/chenjunnn/rm_vision
- picamera2（官方）：https://github.com/raspberrypi/picamera2
- pyzbar：https://github.com/NaturalHistoryMuseum/pyzbar
- 关联文档：《[模块接口说明](模块接口说明.md)》《[底盘参考实现](底盘参考实现.md)》

*所有仓库均于 2026-10-23 实际 clone 或 API 核实。*
