# 机械组（M1）工作目录

放 SolidWorks 图纸、导出件与加工文件。

```text
cad/
├─ assembly/       总装（.SLDASM）
├─ parts/          零件（.SLDPRT）
├─ drawings/       2D 工程图 / 激光切割 DXF
├─ print/          3D 打印 STL（举升机构、支架）
└─ exports/        pack-and-go zip、STEP、渲染图
```

## 规则
- **文件名用英文**，避免中文路径导致 SolidWorks 引用丢失。
- 每周导出一次 `exports/pack-and-go-YYYYMMDD.zip`（提交要求）。
- 尺寸红线：整车 **≤ 300 × 300 × 180 mm**（含举升收拢态）。

## 关键任务
举升机构见《[分工与任务拆解](../docs/给队友看/分工与任务拆解.md)》M1-3，**10-31 前必须打样验证**。
