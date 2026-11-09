# maps —— 建图结果（M4）

存放 `slam_toolbox` / `map_saver` 产出的地图：

```text
maps/
├─ dase7503_field.pgm/yaml   ★ 比赛场地正式地图（待进场地建，见下）
├─ test_desk.pgm/yaml        桌面链路验证图（2026-11-09，仅证明管道通，不是场地）
└─ field_YYYYMMDD/           每次重建单独存目录，便于回退
```

- 建图命令：`ros2 launch robot_navigation slam.launch.py` 后
  `ros2 run nav2_map_server map_saver_cli -f maps/dase7503_field`（详见《单模块测试说明》M4）。
- **实测数据为准**，不要直接改 yaml 里的分辨率来"凑"尺寸。
- 地图文件进 Git；生成的 bag / 日志不进（见 `.gitignore`）。
