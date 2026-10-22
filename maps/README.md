# maps —— 建图结果（M4）

存放 `slam_toolbox` / `map_saver` 产出的地图：

```text
maps/
├─ field.pgm
├─ field.yaml
└─ field_YYYYMMDD/    每次重建单独存目录，便于回退
```

- 建图命令见《[运行说明](../docs/给队友看/运行说明.md)》第 7 节。
- **实测数据为准**，不要直接改 yaml 里的分辨率来"凑"尺寸。
- 地图文件进 Git；生成的 bag / 日志不进（见 `.gitignore`）。
