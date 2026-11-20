#!/usr/bin/env python3
"""生成 DASE7503 比赛场地 的 Gazebo (SDF) 世界文件。

场地布局以用户于 2026-11-20 手工确认的 Harmonic SDF 为准。

当前坐标系：
  - 单位：米 (m)，+Z 向上；原点在原图纸右下角（旧坐标 3.02, 0）。
  - +X = 原 +Y，+Y = 原 -X，保持右手系。
  - x_new = y_old，y_new = 3.02 - x_old；yaw_new = yaw_old - π/2。
  - 场地范围 x ∈ [0, 2.00]，y ∈ [0, 3.02]。
  - Start 中心 (1.00, 0.30)，Destination 中心 (1.00, 2.62)。
  - 货架的物理摆放保持不变；通道沿新 ±Y，二维码朝新 +Y。

用法：
  python3 sim/scripts/generate_field_world.py                    # Harmonic 版（Jazzy/24.04）
  python3 sim/scripts/generate_field_world.py --target fortress  # Fortress 版（Humble/22.04）
  python3 sim/scripts/generate_field_world.py --check            # 只做几何自检 + 俯视图
  python3 sim/scripts/generate_field_world.py --stdout           # 只打印不写文件
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

# --------------------------------------------------------------------------
# 1. 场地 / 围墙 / 货架基础尺寸
# --------------------------------------------------------------------------
FIELD_X = 2.00          # 新 X 跨度（原 Y）
FIELD_Y = 3.02          # 新 Y 跨度（原 X）
WALL_T = 0.05           # 围墙厚度
WALL_H = 0.50           # 围墙高度

FLOOR_T = 0.002         # 场地地板厚度（仅视觉，略高于地面避免 z-fighting）
ZONE_T = 0.002          # 区域标记厚度（仅视觉，无碰撞）
ZONE_Z = FLOOR_T + 0.001

# 2026-11-19：按实物近似为两片侧墙；35 cm 指两内侧面之间的净空。
# 新坐标下侧墙沿 Y：X 向厚 2.5 cm、Y 向长 15 cm，通道沿 ±Y。
# 两片墙分别位于通道 X 负/正侧；没有底板、层板或跨通道的碰撞体。
RACK_GAP = 0.35
RACK_WALL_T = 0.025
RACK_WALL_L = 0.15
RACK_H = 0.23
QR_CENTER_Z = 0.25      # 二维码中心距地高度；不是在货架顶部再加 25 cm
QR_SIZE = 0.05          # 二维码边长 5 cm
SIM_ROOT = Path(__file__).resolve().parents[1]

# --------------------------------------------------------------------------
# 2. 区域（新坐标：中心 + X/Y 尺寸）
# --------------------------------------------------------------------------
ZONES = {
    "destination": dict(cx=1.00, cy=2.62, w=1.40, h=0.80),
    "start":       dict(cx=1.00, cy=0.30, w=0.80, h=0.60),
}

# --------------------------------------------------------------------------
# 3. 货架（由用户确认的 SDF 布局转换，2026-11-20）
# --------------------------------------------------------------------------
# 新坐标：cx=旧 y，cy=3.02-旧 x。二维码网格原始正面 -Y，yaw=π 后朝新 +Y。
RACKS = {
    "a": dict(cx=0.550, cy=1.895, qr_yaw=math.pi),
    "b": dict(cx=1.360, cy=1.195, qr_yaw=math.pi),
    "c": dict(cx=1.670, cy=1.695, qr_yaw=math.pi),
    "d": dict(cx=0.410, cy=1.095, qr_yaw=math.pi),
}

# 起终点标志柱：新场景坐标，尺寸单位 m；高度“20”按 20 cm 处理。
MARKER_DIAMETER = 0.10
MARKER_H = 0.20
MARKER_QR_SIZE = 0.05
MARKER_QR_CENTER_Z = MARKER_H + MARKER_QR_SIZE / 2  # 下边缘位于柱顶
# 网格正面原为 -Y，yaw=π/4 后法向为 (1,-1)/sqrt(2)。
MARKERS = {
    "s": dict(cx=0.60, cy=0.60, texture="Start.png", qr_yaw=math.pi / 4),
    "e": dict(cx=0.30, cy=2.22, texture="End.png", qr_yaw=math.pi / 4),
}

# --------------------------------------------------------------------------
# 4. 颜色
# --------------------------------------------------------------------------
COL_GREY_WALL = ("0.45 0.45 0.48 1", "0.55 0.55 0.58 1")
COL_GREY_FLOOR = ("0.62 0.62 0.64 1", "0.72 0.72 0.74 1")
COL_GREEN = ("0.02 0.45 0.10 1", "0.10 0.75 0.20 1")     # Destination
COL_BLUE = ("0.02 0.15 0.50 1", "0.15 0.40 0.90 1")      # Start
COL_WOOD = ("0.42 0.26 0.10 1", "0.85 0.62 0.32 1")      # 货架（木色/黄）

ZONE_COLORS = {"destination": COL_GREEN, "start": COL_BLUE}
ZONE_LABELS = {"destination": "Destination 绿", "start": "Start 蓝"}

SDF_VERSION = {"harmonic": "1.10", "fortress": "1.9"}

# Gazebo 新版（Harmonic, gz-sim8）与 Fortress（ign-gazebo6）的系统插件命名不同：
#   Harmonic: filename="gz-sim-physics-system"          name="gz::sim::systems::Physics"
#   Fortress: filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"
PLUGIN_NS = {"harmonic": ("gz-sim", "gz::sim::systems"),
             "fortress": ("ignition-gazebo", "ignition::gazebo::systems")}


# --------------------------------------------------------------------------
# 5. 由"中心 + 尺寸"得到矩形，便于自检与画图
# --------------------------------------------------------------------------
def rect_of(cx: float, cy: float, w: float, h: float) -> dict:
    return dict(cx=cx, cy=cy, w=w, h=h,
                x0=cx - w / 2, x1=cx + w / 2,
                y0=cy - h / 2, y1=cy + h / 2)


def collect_rects() -> list[tuple[str, float, float, float, float, bool]]:
    """(名称, x0, x1, y0, y1, 是否有碰撞)"""
    items = []
    for key, z in ZONES.items():
        r = rect_of(z["cx"], z["cy"], z["w"], z["h"])
        items.append(
            (f"{key}_zone", r["x0"], r["x1"], r["y0"], r["y1"], False))
    for key, k in RACKS.items():
        for side, sign in (("west", -1), ("east", 1)):
            cx = k["cx"] + sign * (RACK_GAP + RACK_WALL_T) / 2
            r = rect_of(cx, k["cy"], RACK_WALL_T, RACK_WALL_L)
            items.append((f"rack_{key}_{side}", r["x0"], r["x1"],
                          r["y0"], r["y1"], True))
    for key, marker in MARKERS.items():
        r = rect_of(marker["cx"], marker["cy"], MARKER_DIAMETER, MARKER_DIAMETER)
        items.append((f"marker_{key}", r["x0"], r["x1"], r["y0"], r["y1"], True))
    return items


# --------------------------------------------------------------------------
# 6. SDF 片段生成
# --------------------------------------------------------------------------
def sdf_header(target: str) -> str:
    pretty = {"harmonic": "Gazebo Harmonic (gz-sim8), SDF 1.10",
              "fortress": "Gazebo Fortress (ign-gazebo6), SDF 1.9"}[target]
    return f"""<?xml version="1.0" ?>
<!--
  DASE7503 比赛场地 —— Gazebo 世界文件（右下角原点的新坐标系）
  *** 本文件由 sim/scripts/generate_field_world.py 自动生成，请勿手改 ***
  改坐标请改脚本里的 ZONES / RACKS / MARKERS 三张表，然后重新生成。

  坐标系：原点在旧图右下角；新 +X=旧 +Y，新 +Y=旧 -X，+Z 天向。
  转换：新 x=旧 y，新 y=3.02-旧 x；场地 X={FIELD_X:.2f} m，Y={FIELD_Y:.2f} m。
  Start=(1.00,0.30)，Destination=(1.00,2.62)；Rack 布局来自用户确认的 SDF。
  目标格式：{pretty}
-->
<sdf version="{SDF_VERSION[target]}">
  <world name="dase7503_field">
"""


SDF_TAIL = """  </world>
</sdf>
"""


def plugin_block(target: str) -> str:
    f, n = PLUGIN_NS[target]
    return f"""    <!-- 必需的系统插件：缺任何一个都会表现为
         「没有物理 / 生成不了模型 / 看不到画面 / 雷达无数据」 -->
    <plugin filename='{f}-physics-system' name='{n}::Physics'/>
    <plugin filename='{f}-user-commands-system' name='{n}::UserCommands'/>
    <plugin filename='{f}-scene-broadcaster-system' name='{n}::SceneBroadcaster'/>
    <plugin filename='{f}-sensors-system' name='{n}::Sensors'>
      <render_engine>ogre2</render_engine>
    </plugin>
"""


def material(ambient: str, diffuse: str, indent: str) -> str:
    return (f"{indent}<material>\n"
            f"{indent}  <ambient>{ambient}</ambient>\n"
            f"{indent}  <diffuse>{diffuse}</diffuse>\n"
            f"{indent}  <specular>0.1 0.1 0.1 1</specular>\n"
            f"{indent}</material>\n")


def box_model(name: str, size, pose, ambient, diffuse, *, static=True, collision=True) -> str:
    sx, sy, sz = size
    px, py, pz, rr, pp, yy = pose
    out = [f"    <model name='{name}'>"]
    if static:
        out.append("      <static>true</static>")
    out.append("      <pose>{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}</pose>".format(
        px, py, pz, rr, pp, yy))
    out.append("      <link name='link'>")
    if collision:
        out.append("        <collision name='collision'>")
        out.append("          <geometry>")
        out.append(
            "            <box><size>{:.6f} {:.6f} {:.6f}</size></box>".format(sx, sy, sz))
        out.append("          </geometry>")
        out.append("        </collision>")
    out.append("        <visual name='visual'>")
    out.append("          <geometry>")
    out.append(
        "            <box><size>{:.6f} {:.6f} {:.6f}</size></box>".format(sx, sy, sz))
    out.append("          </geometry>")
    out.append(material(ambient, diffuse, "          ").rstrip("\n"))
    out.append("        </visual>")
    out.append("      </link>")
    out.append("    </model>")
    return "\n".join(out) + "\n"


def rack_model(key: str, rack: dict, resource_base: str) -> str:
    """两片实体侧墙 + 悬空二维码（仅视觉），保留通道完整碰撞净空。"""
    cx, cy = rack["cx"], rack["cy"]
    out = [f"    <model name='rack_{key}'>\n      <static>true</static>\n"
           f"      <pose>{cx:.6f} {cy:.6f} 0 0 0 0</pose>\n"]
    for side, sign in (("west", -1), ("east", 1)):
        dx = sign * (RACK_GAP + RACK_WALL_T) / 2
        geometry = (f"<geometry><box><size>{RACK_WALL_T:.6f} "
                    f"{RACK_WALL_L:.6f} {RACK_H:.6f}</size></box></geometry>")
        out.append(f"      <link name='{side}_wall'>\n"
                   f"        <pose>{dx:.6f} 0 {RACK_H / 2:.6f} 0 0 0</pose>\n"
                   f"        <collision name='collision'>{geometry}</collision>\n"
                   f"        <visual name='visual'>\n          {geometry}\n"
                   + material(*COL_WOOD, "          ") +
                   "        </visual>\n      </link>\n")
    out.append(qr_panel_link(resource_base, f"{key.upper()}.png",
                             QR_SIZE, QR_CENTER_Z, rack["qr_yaw"]))
    out.append("    </model>\n")
    return "".join(out)


def qr_panel_link(resource_base: str, texture: str, size: float,
                  center_z: float, yaw: float) -> str:
    """共用双面二维码网格：局部 XY 原点、正面原始法向 -Y，仅视觉。"""
    return (f"      <link name='qr_panel'>\n"
            f"        <pose>0 0 {center_z:.6f} 0 0 {yaw:.6f}</pose>\n"
            "        <visual name='qr_image'>\n"
            "          <cast_shadows>false</cast_shadows>\n"
            "          <geometry><mesh>\n"
            f"            <uri>{resource_base}/meshes/qr_panel.dae</uri>\n"
            f"            <scale>{size:.6f} 1 {size:.6f}</scale>\n"
            "          </mesh></geometry>\n"
            "          <material>\n"
            "            <ambient>1 1 1 1</ambient>\n"
            "            <diffuse>1 1 1 1</diffuse>\n"
            "            <specular>0 0 0 1</specular>\n"
            "            <pbr><metal>\n"
            f"              <albedo_map>{resource_base}/materials/textures/{texture}</albedo_map>\n"
            "              <metalness>0</metalness><roughness>1</roughness>\n"
            "            </metal></pbr>\n"
            "          </material>\n"
            "        </visual>\n      </link>\n")


def marker_model(key: str, marker: dict, resource_base: str) -> str:
    """固定圆柱有碰撞，5 cm 二维码竖直安装在柱顶、无碰撞。"""
    geometry = (f"<geometry><cylinder><radius>{MARKER_DIAMETER / 2:.6f}</radius>"
                f"<length>{MARKER_H:.6f}</length></cylinder></geometry>")
    return (f"    <model name='marker_{key}'>\n      <static>true</static>\n"
            f"      <pose>{marker['cx']:.6f} {marker['cy']:.6f} 0 0 0 0</pose>\n"
            "      <link name='pillar'>\n"
            f"        <pose>0 0 {MARKER_H / 2:.6f} 0 0 0</pose>\n"
            f"        <collision name='collision'>{geometry}</collision>\n"
            "        <visual name='visual'>\n"
            f"          {geometry}\n"
            + material(*COL_GREY_WALL, "          ") +
            "        </visual>\n      </link>\n"
            + qr_panel_link(resource_base, marker["texture"], MARKER_QR_SIZE,
                            MARKER_QR_CENTER_Z, marker["qr_yaw"]) +
            "    </model>\n")


# --------------------------------------------------------------------------
# 7. 组装世界
# --------------------------------------------------------------------------
def build_world(target: str = "harmonic", resource_base: str = "..") -> str:
    p = [sdf_header(target)]

    # ---- 物理 / 插件 ----
    p.append("""    <gravity>0 0 -9.8</gravity>

    <physics default='0' name='default_physics' type='ode'>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

""")
    p.append(plugin_block(target))

    # ---- 光照（位置随场地中心走）----
    p.append(f"""
    <light type='directional' name='sun'>
      <cast_shadows>true</cast_shadows>
      <pose>0 3.02 6 0 0 0</pose>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0.25 0.45 -0.86</direction>
    </light>

    <light type='point' name='fill_light'>
      <pose>{FIELD_X / 2:.6f} {FIELD_Y / 2:.6f} 2.20 0 0 0</pose>
      <diffuse>0.55 0.55 0.55 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>12</range><constant>0.6</constant><linear>0.08</linear><quadratic>0.01</quadratic>
      </attenuation>
    </light>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.72 0.75 0.80 1</background>
      <shadows>true</shadows>
    </scene>
""")

    # ---- 地面 ----
    p.append("""    <model name='ground_plane'>
      <static>true</static>
      <pose>0 3.02 0 0 0 -1.570796326795</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        </collision>
        <visual name='visual'>
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
""" + material(*COL_GREY_FLOOR, "          ").rstrip("\n") + """
        </visual>
      </link>
    </model>
""")

    # ---- 场地地板（仅视觉）----
    p.append(box_model(
        "field_floor", (FIELD_X, FIELD_Y, FLOOR_T),
        (FIELD_X / 2, FIELD_Y / 2, FLOOR_T / 2, 0, 0, 0),
        *COL_GREY_FLOOR, collision=False))

    # ---- 四面围墙（贴在场地外侧，内表面正好在场界线）----
    p.append(
        "    <!-- ===== 围墙：高度 {:.2f} m，内表面即场界线 ===== -->\n".format(WALL_H))
    half = WALL_T / 2
    walls = {
        # 新南/北墙对应旧东/西墙，长 2.00 m
        "wall_south": ((FIELD_X, WALL_T, WALL_H),
                       (FIELD_X / 2, -half, WALL_H / 2, 0, 0, 0)),
        "wall_north": ((FIELD_X, WALL_T, WALL_H),
                       (FIELD_X / 2, FIELD_Y + half, WALL_H / 2, 0, 0, 0)),
        # 新西/东墙对应旧南/北墙，保留墙角搭接长度
        "wall_west": ((WALL_T, FIELD_Y + 2 * WALL_T, WALL_H),
                      (-half, FIELD_Y / 2, WALL_H / 2, 0, 0, 0)),
        "wall_east": ((WALL_T, FIELD_Y + 2 * WALL_T, WALL_H),
                      (FIELD_X + half, FIELD_Y / 2, WALL_H / 2, 0, 0, 0)),
    }
    for name, (size, pose) in walls.items():
        p.append(box_model(name, size, pose, *COL_GREY_WALL))

    # ---- 区域（仅视觉：机器人要能开过去）----
    p.append("    <!-- ===== 区域标记：无碰撞，仅视觉 ===== -->\n")
    for key, z in ZONES.items():
        r = rect_of(z["cx"], z["cy"], z["w"], z["h"])
        p.append("    <!-- {}: 中心 ({:.3f}, {:.3f})  x {:.3f}~{:.3f}  y {:.3f}~{:.3f} -->\n".format(
            ZONE_LABELS[key], z["cx"], z["cy"], r["x0"], r["x1"], r["y0"], r["y1"]))
        p.append(box_model(f"{key}_zone", (z["w"], z["h"], ZONE_T),
                           (z["cx"], z["cy"], ZONE_Z, 0, 0, 0),
                           *ZONE_COLORS[key], collision=False))

    # ---- 开放式货架（仅侧墙有碰撞）与真实二维码贴图 ----
    p.append(f"    <!-- ===== 货架：净空 {RACK_GAP:.2f} m；两侧墙 "
             f"X/Y/Z={RACK_WALL_T:.3f}/{RACK_WALL_L:.2f}/{RACK_H:.2f} m；通道沿 Y ===== -->\n")
    for key, k in RACKS.items():
        p.append(rack_model(key, k, resource_base))

    p.append("    <!-- ===== 起终点圆柱 S/E + 柱顶二维码，正面朝 (1,-1) ===== -->\n")
    for key, marker in MARKERS.items():
        p.append(marker_model(key, marker, resource_base))

    p.append(SDF_TAIL)
    return "".join(p)


# --------------------------------------------------------------------------
# 8. 自检（含重叠告警与净空提示）+ 俯视图
# --------------------------------------------------------------------------
def audit() -> int:
    items = collect_rects()
    hard = 0
    print("=" * 100)
    print("几何自检 —— 右下角原点的新坐标 + 2026-11-20 用户确认布局")
    print(f"场地 {FIELD_X:.2f} (X/横向) x {FIELD_Y:.2f} (Y/纵向)  [新坐标布局]")
    print("=" * 100)
    print(f"  {'对象':<17}{'x 范围':<21}{'y 范围':<21}{'尺寸':<12}"
          f"{'距左':>7}{'距右':>7}{'距底':>7}{'距顶':>7}")
    for name, x0, x1, y0, y1, solid in items:
        ok = (x0 >= -1e-9 and y0 >= -1e-9 and x1 <=
              FIELD_X + 1e-9 and y1 <= FIELD_Y + 1e-9)
        if not ok:
            hard += 1
        tag = "" if ok else "  ← 出界!"
        print(f"  {name:<17}[{x0:6.3f},{x1:6.3f}]     [{y0:6.3f},{y1:6.3f}]     "
              f"{x1-x0:.2f}x{y1-y0:.2f}   {x0:6.3f} {FIELD_X-x1:6.3f} "
              f"{y0:6.3f} {FIELD_Y-y1:6.3f}{tag}")

    print("-" * 100)
    for asset in [SIM_ROOT / "meshes" / "qr_panel.dae", *[
            SIM_ROOT / "materials" / "textures" / f"{key.upper()}.png" for key in RACKS], *[
            SIM_ROOT / "materials" / "textures" / m["texture"] for m in MARKERS.values()]]:
        if not asset.is_file():
            print(f"  [✗ 缺少二维码资源] {asset}")
            hard += 1
    print(f"  [i] 货架通道沿 ±Y（X 向净宽 {RACK_GAP:.3f} m）；"
          f"二维码中心高 {QR_CENTER_Z:.3f} m，边长 {QR_SIZE:.3f} m（仅视觉）")
    solids = [it for it in items if it[5]]
    racks = [it for it in solids if it[0].startswith("rack_")]
    zones = [it for it in items if not it[5]]

    # 静态障碍不允许重叠；圆柱保守使用外接正方形。
    for i in range(len(solids)):
        for j in range(i + 1, len(solids)):
            a, b = solids[i], solids[j]
            if a[1] < b[2] and b[1] < a[2] and a[3] < b[4] and b[3] < a[4]:
                print(f"  [✗ 实体包围盒重叠] {a[0]} <-> {b[0]}")
                hard += 1
    print(f"  [{'OK' if hard == 0 else '✗'}] 侧墙与圆柱包围盒无重叠；"
          f"全部实体在 {FIELD_X:.2f} x {FIELD_Y:.2f} 边界内")

    # 区域之间 / 区域与货架
    warn = 0
    for i in range(len(zones)):
        for j in range(i + 1, len(zones)):
            a, b = zones[i], zones[j]
            ox = min(a[2], b[2]) - max(a[1], b[1])
            oy = min(a[4], b[4]) - max(a[3], b[3])
            if ox > 1e-9 and oy > 1e-9:
                warn += 1
                print(
                    f"  [⚠ 区域重叠] {a[0]} <-> {b[0]}：x 重叠 {ox:.3f} m, y 重叠 {oy:.3f} m")
            else:
                print(f"  [OK] {a[0]} <-> {b[0]} 不重叠"
                      f"（X 分离 {max(b[1] - a[2], a[1] - b[2]):.3f} m，"
                      f"Y 分离 {max(b[3] - a[4], a[3] - b[4]):.3f} m）")
    for z in zones:
        for r in racks:
            if z[1] < r[2] and r[1] < z[2] and z[3] < r[4] and r[3] < z[4]:
                warn += 1
                print(f"  [⚠ 区域压到货架] {z[0]} <-> {r[0]}")
    if warn == 0:
        print("  [OK] 区域之间、区域与货架之间【均无重叠】")
    else:
        print(f"  [⚠] 共有 {warn} 处区域相关重叠 —— 区域无碰撞故不影响仿真")

    # 货架到最近两面墙的净空（提示"贴得近但没重叠"的位置）
    print("  [i] 货架到最近的两面墙的净空：")
    for name, x0, x1, y0, y1, _ in racks:
        gaps = {"左墙": x0, "右墙": FIELD_X - x1, "底墙": y0, "顶墙": FIELD_Y - y1}
        near = sorted(gaps.items(), key=lambda kv: kv[1])[:2]
        print(f"        {name:<10} " +
              "，".join(f"距{k} {v:.3f} m" for k, v in near))
    return hard


def ascii_map(target_rows: int = 30, max_width: int = 78) -> str:
    """按场地长宽比自适应画俯视图。"""
    width = max(24, min(max_width, int(
        round(target_rows * 2 * FIELD_X / FIELD_Y))))
    rows = max(6, int(round(width * FIELD_Y / FIELD_X / 2)))
    grid = [[" "] * width for _ in range(rows)]

    def put(x0, x1, y0, y1, ch):
        c0 = int(round(x0 / FIELD_X * (width - 1)))
        c1 = int(round(x1 / FIELD_X * (width - 1)))
        r0 = int(round((1 - y1 / FIELD_Y) * (rows - 1)))
        r1 = int(round((1 - y0 / FIELD_Y) * (rows - 1)))
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                grid[r][c] = ch

    for name, x0, x1, y0, y1, solid in collect_rects():
        ch = {"destination_zone": "G", "start_zone": "s", "marker_s": "S", "marker_e": "E"}.get(
            name, name.split("_")[1].upper())
        put(x0, x1, y0, y1, ch)

    top = "+" + "-" * width + "+"
    body = "\n".join("|" + "".join(r) + "|" for r in grid)
    return (f"{top}\n{body}\n{top}\n"
            f"  上方 = 场地顶部(y={FIELD_Y:.2f})，下方 = 场地底部(y=0)，左 = x=0，右 = x={FIELD_X:.2f}\n"
            f"  G=Destination绿（高 Y 侧）  s=Start蓝（低 Y 侧）  S/E=圆柱  A/B/C/D=货架\n"
            f"  场地 {FIELD_X:.2f}(X/横向) x {FIELD_Y:.2f}(Y/纵向) m，新坐标")


def summary() -> str:
    L = []
    L.append("场地: {:.2f} (X/横向) x {:.2f} (Y/纵向) m  —— 右下角原点的新坐标，围墙高 {:.2f} m".format(
        FIELD_X, FIELD_Y, WALL_H))
    for key, z in ZONES.items():
        r = rect_of(z["cx"], z["cy"], z["w"], z["h"])
        L.append("{:<12} 中心 ({:.2f}, {:.2f})  {}x{} m  x {:.3f}~{:.3f}  y {:.3f}~{:.3f}".format(
            key, z["cx"], z["cy"], z["w"], z["h"], r["x0"], r["x1"], r["y0"], r["y1"]))
    for key, k in RACKS.items():
        L.append("RACK {:<8} 中心 ({:.3f}, {:.3f})；净空 {:.2f} m，两片 {:.3f}x{:.2f}x{:.2f} m 侧墙".format(
            key.upper(), k["cx"], k["cy"], RACK_GAP, RACK_WALL_T, RACK_WALL_L, RACK_H))
    for key, marker in MARKERS.items():
        L.append(f"圆柱 {key.upper()} 中心 ({marker['cx']:.3f}, {marker['cy']:.3f})；"
                 f"直径 {MARKER_DIAMETER:.2f} m，高 {MARKER_H:.2f} m；"
                 f"二维码 {MARKER_QR_SIZE:.2f} m，中心高 {MARKER_QR_CENTER_Z:.3f} m，朝 (1,-1)")
    return "\n".join(L)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", choices=["harmonic", "fortress"], default="harmonic",
                    help="harmonic = Gazebo Harmonic/gz-sim8（Jazzy，默认）；"
                         "fortress = Gazebo Fortress/ign-gazebo6（Humble）")
    ap.add_argument("--stdout", action="store_true", help="只打印，不写文件")
    ap.add_argument("--check", action="store_true", help="只做自检")
    ap.add_argument("--out", default=None, help="输出路径")
    args = ap.parse_args()

    print(summary())
    print()
    hard = audit()
    print()
    print(ascii_map())
    if args.check:
        raise SystemExit(1 if hard else 0)
    if hard:
        raise SystemExit("几何或资源自检失败，未生成世界文件。")
    default_name = ("dase7503_field.sdf" if args.target == "harmonic"
                    else "dase7503_field_fortress.sdf")
    out = Path(args.out).resolve() if args.out else SIM_ROOT / "worlds" / default_name
    resource_base = os.path.relpath(SIM_ROOT, out.parent)
    text = build_world(args.target, resource_base)
    if args.stdout:
        print(text)
        return
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"\n已写入: {out}  ({len(text)} bytes)  [target={args.target}, "
          f"SDF {SDF_VERSION[args.target]}]")


if __name__ == "__main__":
    main()
