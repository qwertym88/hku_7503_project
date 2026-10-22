# qr_perception —— 相机 + QR 解码（M3）

## 职责
```text
Camera Module 3 → camera_ros → /camera/image_raw → OpenCV QRCodeDetector → /qr (String)
```

## 接口
- 发 `/qr`（`std_msgs/String`）：识别到发完整码（如 `RACKA_4X6M`），识别不到不发。
- 内部发 `/camera/image_raw` 仅供自用与 RViz。

## 待办
- [ ] `libcamera` + `camera_ros` 源码编译（**W1 难点**）
- [ ] 解码节点 + 去重去抖
- [ ] 6 类 QR 识别 + 截图归档到 `tests/qr_shots/`
- [ ] 兜底：OpenCV 直连 V4L2

当前为空壳。测试方法见《[单模块测试说明](../../../docs/给队友看/单模块测试说明.md)》。
