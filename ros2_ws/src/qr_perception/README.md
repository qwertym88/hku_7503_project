# qr_perception —— 相机 + QR 解码（M3）

## 职责
```text
Camera Module 3 → picamera2 取流 ─┐
                                  ├─► 纯 Python 解码核心（OpenCV / pyzbar）─► QrResult
（ROS 无关，可图片回放调试）      ─┘                                            │
                                                            ~50 行薄封装 ──────┘─► /qr (String)
```

**架构要点（见《[QR 感知方案](../../../docs/给队友看/QR感知方案.md)》）**：
- **不用 `camera_ros`**，用官方 `picamera2` 取流（Pi OS 预装，无需编译）。
- **解码核心不依赖 ROS**：输入一帧图，输出 `QrResult(text, center, corners, stamp)`，可离线单测。
- 外层只保留一个薄节点，把字符串发到 `/qr`。

## 接口
- 发 `/qr`（`std_msgs/String`）：识别到发完整码（如 `RACKA_4X6M`），识别不到不发。
- 内部发 `/camera/image_raw` 仅供自用与 RViz（**不算契约接口**）。

## 待办
- [ ] `picamera2` 取流脚本（**W1**，≥15fps 不丢帧，可存图回放）
- [ ] 纯 Python 解码核心（OpenCV `QRCodeDetector` 首选，`pyzbar` 兜底）
- [ ] ~50 行 ROS 薄封装 → `/qr` + 多帧投票去重去抖
- [ ] 6 类 QR 识别 + 截图归档到 `tests/qr_shots/`
- [ ] 相机按扫码距离**固定对焦**（自动对焦会"拉风箱"）

## 参考实现（已核实）
- [ros-drivers/zbar_ros](https://github.com/ros-drivers/zbar_ros)：**薄封装范本**，整包节点仅 162 行，CI 覆盖 jazzy。
- [ZengWenJian123/qrcode_recognition](https://github.com/ZengWenJian123/qrcode_recognition)：**50 行纯 Python** 解码范本。

当前为空壳。测试方法见《[单模块测试说明](../../../docs/给队友看/单模块测试说明.md)》。
