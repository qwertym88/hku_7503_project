# 图纸

这个目录放**坐标的依据**：场地工程图 / 俯视图。它必须随代码一起走，否则换设备后没人知道
`sim/scripts/generate_field_world.py` 里那些硬编码坐标是从哪来的。

## 应当放什么

| 文件 | 说明 |
|---|---|
| `1.png` | 场地俯视工程图（横向 **3020 × 2000 mm**）。`sim/README.md` 与坐标注释都引用它 |

## 现在缺什么

⚠️ **`1.png` 尚未入库**（此前只在维护者本机）。请把它复制到本目录：

```bash
cp <你本机上的 1.png> docs/图纸/1.png
git add docs/图纸/1.png && git commit -m "docs: 场地工程图入库"
```

## 图纸的原始来源

原始 PDF 已在仓库里（无需另外找）：

- `requirements/2. DASE7503 Group Project_QR Code and Gamefield.pdf`
  —— 3 页，A4 竖版（595×842 pt），工程图嵌在页内，`1.png` 是从中裁出的图。
- `requirements/1. DASE7503 Group Project_2026-27_1B.pdf` —— 需求与规则
- `requirements/3. Group Projects Materials.pdf` —— 器材清单
- `requirements/4. DASE7503_Appendix.Samples of QR codes.pdf` —— 二维码样例

## 已从图纸读出的关键尺寸（供交叉核对）

- 场地外框：横向 **3020**、纵向 **2000** mm（**横向**矩形）
- Destination 区：800 × 1400 mm，贴左墙（`x 0.00~0.80`，`y 0.30~1.70`）
- Start 区：600 × 800 mm，贴右墙（`x 2.42~3.02`，`y 0.60~1.40`）
- 货架：平面 **140 × 140** mm，高 **232.5** mm，四立柱 + 三层板
- 货架中心：A `(1.20, 1.58)`、B `(1.85, 0.51)`、C `(1.85, 1.72)`、D `(1.35, 0.37)`

> 这些数值一旦与图纸不符，**改 `sim/scripts/generate_field_world.py` 里的 `ZONES` / `RACKS` 两张表**
> 然后重新生成世界文件；**不要手改 `sim/worlds/*.sdf`**。
