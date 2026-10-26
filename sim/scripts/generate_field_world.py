#!/usr/bin/env python3
"""生成 DASE7503 比赛场地 的 Gazebo (SDF) 世界文件。

单一事实来源：本文件里的 FIELD / ZONES / RACKS 三张表。
world 文件由它生成，所以改尺寸只要改这里，不会出现"手改 SDF 改漏一处"。

坐标系（与图纸一致）：
  - 单位：米 (m)
  - 原点 (0,0) = 场地左下角
  - +X 向右，+Y 向上，+Z 向上（Gazebo 天向）
  - 场地范围：x ∈ [0, 3.02]，y ∈ [0, 2.0]
  - 围墙贴在场地【外侧】，内表面正好压在 x=0/3.02、y=0/2.0 上，所以不占用场地面积

用法：
  python3 sim/scripts/generate_field_world.py           # 写入 sim/worlds/dase7503_field.sdf
  python3 sim/scripts/generate_field_world.py --stdout  # 只打印到屏幕
  python3 sim/scripts/generate_field_world.py --check   # 只做几何自检 + 俯视图
"""

from __future__ import annotations

import argparse
import os

# --------------------------------------------------------------------------
# 1. 场地与围墙
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
SHELF_Z = (0.020, 0.2325 / 2, RACK_H - 0.012)   # 三层板的中心高度

# --------------------------------------------------------------------------
# 2. 区域（图纸给定"位置"，尺寸冲突时按 §4 规则解析）
# --------------------------------------------------------------------------
# 图纸原值：Destination 0.8x1.4，距左墙 0.8，距顶墙 0.8
#           Start       0.6x0.8，贴右墙 0.0，距底墙 0.6
ZONES_RAW = {
    "destination": dict(w=0.80, h=1.40, from_west=0.80, from_north=0.80),
    "start":       dict(w=0.60, h=0.80, from_east=0.00, from_south=0.60),
}


def resolve_destination() -> dict:
    """Destination 区：保留两个锚定偏移(0.8/0.8)，只压缩冲突的高度 1.4→1.2。

    理由：图纸明确给了"距左墙 0.8、距顶墙 0.8"这两个【位置】约束，
    而 0.8 + 1.4 = 2.2 > 2.0 冲突的是【尺寸】。按"位置优先"处理。

    备选方案（若你更想保住 1.4 的高度）：把下面改成
        y1 = FIELD_Y            # 顶边贴顶墙
        y0 = FIELD_Y - h        # 则距顶墙变成 0.6
    生成器会自动重算 Rack A 的位置（它依赖 Destination 的右边缘，不受影响）。
    """
    w = ZONES_RAW["destination"]["w"]
    h = ZONES_RAW["destination"]["h"]
    x0 = ZONES_RAW["destination"]["from_west"]
    y1 = FIELD_Y - ZONES_RAW["destination"]["from_north"]
    h_max = y1  # 下边界不能低于 y=0
    h_fit = min(h, h_max)
    return dict(x0=x0, x1=x0 + w, y0=y1 - h_fit, y1=y1,
                w=w, h=h_fit, h_raw=h, clipped=(h_fit < h - 1e-9))


def resolve_start() -> dict:
    w = ZONES_RAW["start"]["w"]
    h = ZONES_RAW["start"]["h"]
    x1 = FIELD_X - ZONES_RAW["start"]["from_east"]
    y0 = ZONES_RAW["start"]["from_south"]
    return dict(x0=x1 - w, x1=x1, y0=y0, y1=y0 + h, w=w, h=h,
                h_raw=h, clipped=False)


# --------------------------------------------------------------------------
# 3. 货架（按图纸的相对位置关系解算中心坐标）
# --------------------------------------------------------------------------
def rack_center(*, from_west=None, from_east=None, from_south=None, from_north=None,
                right_of_x=None, right_gap=None) -> tuple[float, float]:
    """由"距墙距离"或"某处右侧留白"推出货架中心 (cx, cy)。"""
    if from_west is not None:
        cx = from_west + RACK_S / 2
    elif from_east is not None:
        cx = FIELD_X - from_east - RACK_S / 2
    elif right_of_x is not None:
        cx = right_of_x + right_gap + RACK_S / 2
    else:
        raise ValueError("需要给出 X 方向约束")

    if from_south is not None:
        cy = from_south + RACK_S / 2
    elif from_north is not None:
        cy = FIELD_Y - from_north - RACK_S / 2
    else:
        raise ValueError("需要给出 Y 方向约束")
    return cx, cy


def build_racks(dest: dict) -> dict:
    racks = {}
    # A：距顶墙 0.35，位于 Destination 右边缘再往右 0.4
    racks["a"] = rack_center(from_north=0.35, right_of_x=dest["x1"], right_gap=0.40)
    # C：距顶墙 0.21，距右墙 0.6
    racks["c"] = rack_center(from_north=0.21, from_east=0.60)
    # B：在 C 正下方（同 X），距右墙 0.6，距底墙 0.44
    racks["b"] = rack_center(from_south=0.44, from_east=0.60)
    # D：距左墙 0.45，距底墙 0.30
    racks["d"] = rack_center(from_west=0.45, from_south=0.30)
    assert abs(racks["b"][0] - racks["c"][0]) < 1e-9, "B 必须在 C 正下方（同 X）"
    return racks


# --------------------------------------------------------------------------
# 4. 颜色
# --------------------------------------------------------------------------
COL_GREY_WALL = ("0.45 0.45 0.48 1", "0.55 0.55 0.58 1")
COL_GREY_FLOOR = ("0.62 0.62 0.64 1", "0.72 0.72 0.74 1")
COL_GREEN = ("0.02 0.45 0.10 1", "0.10 0.75 0.20 1")     # Destination
COL_BLUE = ("0.02 0.15 0.50 1", "0.15 0.40 0.90 1")      # Start
COL_WOOD = ("0.42 0.26 0.10 1", "0.85 0.62 0.32 1")      # 货架（木色/黄）

SDF_VERSION = {"harmonic": "1.10", "fortress": "1.9"}

# Gazebo 新版（Harmonic, gz-sim8）与 Fortress（ign-gazebo6）的系统插件命名不同：
#   Harmonic: filename="gz-sim-physics-system"      name="gz::sim::systems::Physics"
#   Fortress: filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"
PLUGIN_NS = {"harmonic": ("gz-sim", "gz::sim::systems"),
             "fortress": ("ignition-gazebo", "ignition::gazebo::systems")}


def sdf_header(target: str) -> str:
    pretty = {"harmonic": "Gazebo Harmonic (gz-sim8), SDF 1.10",
              "fortress": "Gazebo Fortress (ign-gazebo6), SDF 1.9"}[target]
    return f"""<?xml version="1.0" ?>
<!--
  DASE7503 比赛场地 —— Gazebo 世界文件
  *** 本文件由 sim/scripts/generate_field_world.py 自动生成，请勿手改 ***
  改尺寸/位置请改脚本里的 FIELD / ZONES_RAW / build_racks()，然后重新生成。

  坐标系：原点在场地的左下角，+X 向右(长 3.02)，+Y 向上(宽 2.0)，+Z 天向。
  目标格式：{pretty}
-->
<sdf version="{SDF_VERSION[target]}">
  <world name="dase7503_field">
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

SDF_TAIL = """  </world>
</sdf>
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


def rect_pose(r: dict):
    """由矩形 (x0,x1,y0,y1) 得到中心与尺寸。"""
    return ((r["x0"] + r["x1"]) / 2, (r["y0"] + r["y1"]) / 2,
            r["x1"] - r["x0"], r["y1"] - r["y0"])


def build_world(target: str = "harmonic") -> str:
    dest = resolve_destination()
    start = resolve_start()
    racks = build_racks(dest)
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
    name_cn = {"destination": "Destination 绿", "start": "Start 蓝"}
    for key, r in (("destination", dest), ("start", start)):
        cx, cy, w, h = rect_pose(r)
        col = COL_GREEN if key == "destination" else COL_BLUE
        p.append("    <!-- {}: x {:.3f}~{:.3f}, y {:.3f}~{:.3f} -->\n".format(
            name_cn[key], r["x0"], r["x1"], r["y0"], r["y1"]))
        p.append(box_model(f"{key}_zone", (w, h, ZONE_T),
                           (cx, cy, ZONE_Z, 0, 0, 0), *col, collision=False))

    # ---- 货架 ----
    p.append("    <!-- ===== 货架：{} x {} x {} m，多层结构 ===== -->\n".format(
        RACK_S, RACK_S, RACK_H))
    for key in ("a", "b", "c", "d"):
        cx, cy = racks[key]
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
# 5. 几何自检与俯视图
# --------------------------------------------------------------------------
def collect_rects():
    dest = resolve_destination()
    start = resolve_start()
    racks = build_racks(dest)
    items = [("destination", dest["x0"], dest["x1"], dest["y0"], dest["y1"], False),
             ("start", start["x0"], start["x1"], start["y0"], start["y1"], False)]
    for k in ("a", "b", "c", "d"):
        cx, cy = racks[k]
        items.append((f"rack_{k}", cx - RACK_S / 2, cx + RACK_S / 2,
                      cy - RACK_S / 2, cy + RACK_S / 2, True))
    return items


def check() -> int:
    items = collect_rects()
    bad = 0
    print("=" * 74)
    print("几何自检")
    print("=" * 74)
    for name, x0, x1, y0, y1, _ in items:
        ok = (x0 >= -1e-9 and y0 >= -1e-9 and x1 <= FIELD_X + 1e-9 and y1 <= FIELD_Y + 1e-9)
        flag = "OK " if ok else "出界"
        if not ok:
            bad += 1
        print(f"  [{flag}] {name:<12} x {x0:6.3f}~{x1:6.3f}   y {y0:6.3f}~{y1:6.3f}"
              f"   ({x1-x0:.3f} x {y1-y0:.3f} m)")

    print("-" * 74)
    solids = [it for it in items if it[5]]
    for i in range(len(solids)):
        for j in range(i + 1, len(solids)):
            a, b = solids[i], solids[j]
            if a[1] < b[2] and b[1] < a[2] and a[3] < b[4] and b[3] < a[4]:
                print(f"  [重叠] {a[0]} <-> {b[0]}")
                bad += 1
    print(f"  [{'OK ' if bad == 0 else '问题'}] 货架之间无重叠；全部实体在边界内")
    return bad


def ascii_map(width: int = 74) -> str:
    """按实际比例画俯视图，方便肉眼核对布局。"""
    items = collect_rects()
    rows = int(width * FIELD_Y / FIELD_X / 2) + 1
    grid = [[" "] * width for _ in range(rows)]

    def put(x0, x1, y0, y1, ch):
        c0 = int(x0 / FIELD_X * (width - 1))
        c1 = int(x1 / FIELD_X * (width - 1))
        r0 = int((1 - y1 / FIELD_Y) * (rows - 1))
        r1 = int((1 - y0 / FIELD_Y) * (rows - 1))
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                grid[r][c] = ch

    for name, x0, x1, y0, y1, solid in items:
        ch = {"destination": "G", "start": "S"}.get(name, name[-1].upper())
        put(x0, x1, y0, y1, ch)

    top = "+" + "-" * (width - 2) + "+"
    body = "\n".join("|" + "".join(r) + "|" for r in grid)
    return (f"{top}\n{body}\n{top}\n"
            f"  (0,0)=左下角  G=Destination绿  S=Start蓝  A/B/C/D=货架  "
            f"场地 {FIELD_X} x {FIELD_Y} m)")


def summary() -> str:
    dest = resolve_destination()
    start = resolve_start()
    racks = build_racks(dest)
    L = []
    L.append("场地: {:.2f} (X) x {:.2f} (Y) m, 围墙高 {:.2f} m".format(FIELD_X, FIELD_Y, WALL_H))
    L.append("Destination: x {:.3f}~{:.3f}, y {:.3f}~{:.3f}".format(
        dest["x0"], dest["x1"], dest["y0"], dest["y1"]))
    if dest["clipped"]:
        L.append("   ! 高度由 {:.2f} 压缩为 {:.2f}（0.8+1.4=2.2 > 2.0 冲突，保位置约束）"
                 .format(dest["h_raw"], dest["h"]))
    L.append("Start:       x {:.3f}~{:.3f}, y {:.3f}~{:.3f}".format(
        start["x0"], start["x1"], start["y0"], start["y1"]))
    for k in ("a", "b", "c", "d"):
        cx, cy = racks[k]
        L.append("RACK {}:      center ({:.3f}, {:.3f})".format(k.upper(), cx, cy))
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
    bad = check()
    print()
    print(ascii_map())
    if args.check:
        raise SystemExit(1 if bad else 0)

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
