#!/usr/bin/env python3
"""生成 DASE7503 比赛场地 的 Gazebo (SDF) 世界文件。

坐标【全部硬编码为图纸给定的中心坐标】——不再做约束推导，避免推偏。

坐标系（与图纸一致）：
  - 单位：米 (m)
  - 原点 (0,0) = 场地左下角
  - +X 向右，+Y 向上，+Z 向上（Gazebo 天向）
  - 场地范围：x ∈ [0, 3.02]，y ∈ [0, 2.00]
  - 围墙贴在场地【外侧】，内表面正好压在 x=0/3.02、y=0/2.0 上，不占用场地面积

用法：
  python3 sim/scripts/generate_field_world.py                    # Harmonic 版（Jazzy/24.04）
  python3 sim/scripts/generate_field_world.py --target fortress  # Fortress 版（Humble/22.04）
  python3 sim/scripts/generate_field_world.py --check            # 只做几何自检 + 俯视图
  python3 sim/scripts/generate_field_world.py --stdout           # 只打印不写文件
"""

from __future__ import annotations

import argparse
import os

# --------------------------------------------------------------------------
# 1. 场地 / 围墙 / 货架基础尺寸
# --------------------------------------------------------------------------
FIELD_X = 3.02          # 长（X）
FIELD_Y = 2.00          # 宽（Y）
WALL_T = 0.05           # 围墙厚度
WALL_H = 0.50           # 围墙高度

FLOOR_T = 0.002         # 场地地板厚度（仅视觉，略高于地面避免 z-fighting）
ZONE_T = 0.002          # 区域标记厚度（仅视觉，无碰撞）
ZONE_Z = FLOOR_T + 0.001

RACK_S = 0.14           # 货架长/宽
RACK_H = 0.2325         # 货架高
POST = 0.012            # 货架立柱截面
POST_H = RACK_H
SHELF_T = 0.008         # 层板厚度
SHELF_Z = (0.020, RACK_H / 2, RACK_H - 0.012)   # 三层板的中心高度

# --------------------------------------------------------------------------
# 2. 区域（图纸给定【中心坐标】+ 尺寸，硬编码）
# --------------------------------------------------------------------------
#  destination：宽 0.8、高 1.4，靠左上角。高 1.4 与"距顶墙 0.8"在 2.0 宽的场地里
#               不可能同时满足（0.8+1.4+? > 2.0），按图纸要求【顶部贴紧 Y=2.0】
#               → x 0.80~1.60，y 0.60~2.00，中心 (1.20, 1.30)
#  start      ：宽 0.6、高 0.8，【底部居中】且紧贴底边
#               → x 1.21~1.81，y 0.00~0.80，中心 (1.51, 0.40)
ZONES = {
    "destination": dict(cx=1.20, cy=1.30, w=0.80, h=1.40),
    "start":       dict(cx=1.51, cy=0.40, w=0.60, h=0.80),
}

# --------------------------------------------------------------------------
# 3. 货架（图纸给定【中心坐标】，硬编码）
# --------------------------------------------------------------------------
#  A：Destination 右缘 +0.4，靠上           → (2.07, 1.58)
#  C：距右墙 0.6，距顶墙 0.21                → (2.35, 1.72)
#  B：在 C 正下方（同 X），距右墙 0.6，距底墙 0.44 → (2.35, 0.51)
#  D：距左墙 0.45，距底墙 0.30               → (0.52, 0.37)
RACKS = {
    "a": dict(cx=2.07, cy=1.58),
    "b": dict(cx=2.35, cy=0.51),
    "c": dict(cx=2.35, cy=1.72),
    "d": dict(cx=0.52, cy=0.37),
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
        items.append((f"{key}_zone", r["x0"], r["x1"], r["y0"], r["y1"], False))
    for key, k in RACKS.items():
        r = rect_of(k["cx"], k["cy"], RACK_S, RACK_S)
        items.append((f"rack_{key}", r["x0"], r["x1"], r["y0"], r["y1"], True))
    return items


# --------------------------------------------------------------------------
# 6. SDF 片段生成
# --------------------------------------------------------------------------
def sdf_header(target: str) -> str:
    pretty = {"harmonic": "Gazebo Harmonic (gz-sim8), SDF 1.10",
              "fortress": "Gazebo Fortress (ign-gazebo6), SDF 1.9"}[target]
    return f"""<?xml version="1.0" ?>
<!--
  DASE7503 比赛场地 —— Gazebo 世界文件
  *** 本文件由 sim/scripts/generate_field_world.py 自动生成，请勿手改 ***
  改坐标请改脚本里的 ZONES / RACKS 两张表，然后重新生成。

  坐标系：原点在场地左下角，+X 向右(长 3.02)，+Y 向上(宽 2.00)，+Z 天向。
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
        out.append("            <box><size>{:.6f} {:.6f} {:.6f}</size></box>".format(sx, sy, sz))
        out.append("          </geometry>")
        out.append("        </collision>")
    out.append("        <visual name='visual'>")
    out.append("          <geometry>")
    out.append("            <box><size>{:.6f} {:.6f} {:.6f}</size></box>".format(sx, sy, sz))
    out.append("          </geometry>")
    out.append(material(ambient, diffuse, "          ").rstrip("\n"))
    out.append("        </visual>")
    out.append("      </link>")
    out.append("    </model>")
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------
# 7. 组装世界
# --------------------------------------------------------------------------
def build_world(target: str = "harmonic") -> str:
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

    # ---- 光照 ----
    p.append("""
    <light type='directional' name='sun'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 6 0 0 0</pose>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.25 -0.86</direction>
    </light>

    <light type='point' name='fill_light'>
      <pose>1.51 1.00 2.20 0 0 0</pose>
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
    p.append("    <!-- ===== 围墙：高度 {:.2f} m，内表面即场界线 ===== -->\n".format(WALL_H))
    half = WALL_T / 2
    walls = {
        "wall_south": ((FIELD_X + 2 * WALL_T, WALL_T, WALL_H),
                       (FIELD_X / 2, -half, WALL_H / 2, 0, 0, 0)),
        "wall_north": ((FIELD_X + 2 * WALL_T, WALL_T, WALL_H),
                       (FIELD_X / 2, FIELD_Y + half, WALL_H / 2, 0, 0, 0)),
        "wall_west": ((WALL_T, FIELD_Y, WALL_H),
                      (-half, FIELD_Y / 2, WALL_H / 2, 0, 0, 0)),
        "wall_east": ((WALL_T, FIELD_Y, WALL_H),
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

    # ---- 货架 ----
    p.append("    <!-- ===== 货架：{} x {} x {} m，多层结构 ===== -->\n".format(
        RACK_S, RACK_S, RACK_H))
    for key, k in RACKS.items():
        cx, cy = k["cx"], k["cy"]
        p.append("    <!-- RACK {}: 中心 ({:.3f}, {:.3f}) -->\n".format(key.upper(), cx, cy))
        p.append(f"    <model name='rack_{key}'>\n      <static>true</static>\n"
                 f"      <pose>{cx:.6f} {cy:.6f} 0 0 0 0</pose>\n")
        # 四根立柱
        off = RACK_S / 2 - POST / 2
        for i, (dx, dy) in enumerate(((-off, -off), (-off, off), (off, -off), (off, off))):
            p.append(f"      <link name='post_{i}'>\n"
                     f"        <pose>{dx:.6f} {dy:.6f} {POST_H / 2:.6f} 0 0 0</pose>\n"
                     f"        <collision name='collision'>\n          <geometry>\n"
                     f"            <box><size>{POST} {POST} {POST_H}</size></box>\n"
                     f"          </geometry>\n        </collision>\n"
                     f"        <visual name='visual'>\n          <geometry>\n"
                     f"            <box><size>{POST} {POST} {POST_H}</size></box>\n"
                     f"          </geometry>\n"
                     + material(*COL_WOOD, "          ").rstrip("\n") +
                     "\n        </visual>\n      </link>\n")
        # 三层层板
        for i, z in enumerate(SHELF_Z):
            p.append(f"      <link name='shelf_{i}'>\n"
                     f"        <pose>0 0 {z:.6f} 0 0 0</pose>\n"
                     f"        <collision name='collision'>\n          <geometry>\n"
                     f"            <box><size>{RACK_S} {RACK_S} {SHELF_T}</size></box>\n"
                     f"          </geometry>\n        </collision>\n"
                     f"        <visual name='visual'>\n          <geometry>\n"
                     f"            <box><size>{RACK_S} {RACK_S} {SHELF_T}</size></box>\n"
                     f"          </geometry>\n"
                     + material(*COL_WOOD, "          ").rstrip("\n") +
                     "\n        </visual>\n      </link>\n")
        p.append("    </model>\n")

    p.append(SDF_TAIL)
    return "".join(p)


# --------------------------------------------------------------------------
# 8. 自检（含"区域互相重叠"的显式告警）+ 俯视图
# --------------------------------------------------------------------------
def audit() -> int:
    items = collect_rects()
    hard = 0
    print("=" * 96)
    print("几何自检 —— 所有坐标均为图纸给定值（硬编码，无推导）")
    print("=" * 96)
    print(f"  {'对象':<16}{'x 范围':<20}{'y 范围':<20}{'尺寸':<16}"
          f"{'距左':>7}{'距右':>7}{'距底':>7}{'距顶':>7}")
    for name, x0, x1, y0, y1, solid in items:
        ok = (x0 >= -1e-9 and y0 >= -1e-9 and x1 <= FIELD_X + 1e-9 and y1 <= FIELD_Y + 1e-9)
        if not ok:
            hard += 1
        tag = "" if ok else "  ← 出界!"
        print(f"  {name:<16}[{x0:6.3f},{x1:6.3f}]      [{y0:6.3f},{y1:6.3f}]      "
              f"{x1-x0:.2f}x{y1-y0:.2f}     {x0:6.3f} {FIELD_X-x1:6.3f} "
              f"{y0:6.3f} {FIELD_Y-y1:6.3f}{tag}")

    print("-" * 96)
    racks = [it for it in items if it[5]]
    zones = [it for it in items if not it[5]]

    # 货架之间：不允许重叠
    for i in range(len(racks)):
        for j in range(i + 1, len(racks)):
            a, b = racks[i], racks[j]
            if a[1] < b[2] and b[1] < a[2] and a[3] < b[4] and b[3] < a[4]:
                print(f"  [✗ 货架重叠] {a[0]} <-> {b[0]}")
                hard += 1
    print(f"  [{'OK' if hard == 0 else '✗'}] 货架之间无重叠；全部实体在 3.02 x 2.00 边界内")

    # 区域之间：图纸给定值，若重叠只告警（区域无碰撞，不影响物理）
    warn = 0
    for i in range(len(zones)):
        for j in range(i + 1, len(zones)):
            a, b = zones[i], zones[j]
            ox = min(a[2], b[2]) - max(a[1], b[1])
            oy = min(a[4], b[4]) - max(a[3], b[3])
            if ox > 1e-9 and oy > 1e-9:
                warn += 1
                print(f"  [⚠ 区域重叠] {a[0]} <-> {b[0]}："
                      f"x 重叠 {ox:.3f} m, y 重叠 {oy:.3f} m"
                      f"（重叠区 x[{max(a[1],b[1]):.3f},{min(a[2],b[2]):.3f}] "
                      f"y[{max(a[3],b[3]):.3f},{min(a[4],b[4]):.3f}]）")
    for z in zones:
        for r in racks:
            if z[1] < r[2] and r[1] < z[2] and z[3] < r[4] and r[3] < z[4]:
                warn += 1
                print(f"  [⚠ 区域压到货架] {z[0]} <-> {r[0]}")
    if warn == 0:
        print("  [OK] 区域之间、区域与货架之间无重叠")
    else:
        print(f"  [⚠] 共有 {warn} 处区域重叠 —— 这是【图纸给定坐标的必然结果】，"
              f"区域无碰撞故不影响仿真；如需消除，见 sim/README.md §2")
    return hard


def ascii_map(width: int = 74) -> str:
    items = collect_rects()
    rows = int(width * FIELD_Y / FIELD_X / 2) + 1
    grid = [[" "] * width for _ in range(rows)]

    def put(x0, x1, y0, y1, ch):
        c0 = int(round(x0 / FIELD_X * (width - 1)))
        c1 = int(round(x1 / FIELD_X * (width - 1)))
        r0 = int(round((1 - y1 / FIELD_Y) * (rows - 1)))
        r1 = int(round((1 - y0 / FIELD_Y) * (rows - 1)))
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                grid[r][c] = ch

    for name, x0, x1, y0, y1, solid in items:
        ch = {"destination_zone": "G", "start_zone": "S"}.get(name, name[-1].upper())
        put(x0, x1, y0, y1, ch)

    top = "+" + "-" * (width - 2) + "+"
    body = "\n".join("|" + "".join(r) + "|" for r in grid)
    return (f"{top}\n{body}\n{top}\n"
            f"  (0,0)=左下角  G=Destination绿  S=Start蓝  A/B/C/D=货架  "
            f"场地 {FIELD_X} x {FIELD_Y} m)")


def summary() -> str:
    L = []
    L.append("场地: {:.2f} (X) x {:.2f} (Y) m, 围墙高 {:.2f} m".format(FIELD_X, FIELD_Y, WALL_H))
    for key, z in ZONES.items():
        r = rect_of(z["cx"], z["cy"], z["w"], z["h"])
        L.append("{:<12} 中心 ({:.2f}, {:.2f})  {}x{} m  x {:.3f}~{:.3f}  y {:.3f}~{:.3f}".format(
            key, z["cx"], z["cy"], z["w"], z["h"], r["x0"], r["x1"], r["y0"], r["y1"]))
    for key, k in RACKS.items():
        L.append("RACK {:<8} 中心 ({:.2f}, {:.2f})".format(key.upper(), k["cx"], k["cy"]))
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

    text = build_world(args.target)
    if args.stdout:
        print(text)
        return
    default_name = ("dase7503_field.sdf" if args.target == "harmonic"
                    else "dase7503_field_fortress.sdf")
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "..", "worlds", default_name)
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"\n已写入: {out}  ({len(text)} bytes)  [target={args.target}, "
          f"SDF {SDF_VERSION[args.target]}]")


if __name__ == "__main__":
    main()
