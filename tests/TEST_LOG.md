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
