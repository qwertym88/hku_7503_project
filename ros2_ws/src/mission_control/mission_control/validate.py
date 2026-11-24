"""Offline route/feedback rehearsal and SVG route review."""

import argparse
import json
import math
from pathlib import Path
from xml.sax.saxutils import escape

from .core import Mission
from .route import action_samples, load_config, pose, rack_polygons


def rehearse(cfg, dt=0.02):
    mission = Mission(cfg)
    p = list(pose(cfg['start_pose']))
    velocity = (0., 0.)
    now = 0.
    mission.start(now, p)
    while mission.state == 'RUNNING' and now < cfg['control']['mission_timeout_s'] + dt:
        a = mission.action
        if a.kind in ('photo', 'lift', 'lower') and not mission.effect_done:
            mission.acknowledge(a.id, True, 'offline mock effect; no camera or Gazebo')
        v, w = mission.tick(now, p, velocity)
        mid_yaw = p[2] + w*dt/2
        p[0] += v*math.cos(mid_yaw)*dt
        p[1] += v*math.sin(mid_yaw)*dt
        p[2] += w*dt
        velocity = v, w
        now += dt
    return mission, now


def write_svg(cfg, actions, path):
    scale = 220
    margin = 35
    field = cfg['scene']['field']
    height = field['y_max'] - field['y_min']
    def point(p):
        return f'{margin+(p[0]-field["x_min"])*scale:.2f},{margin+(field["y_max"]-p[1])*scale:.2f}'
    def polygon(points, color, opacity=1.):
        return f'<polygon points="{" ".join(point(p) for p in points)}" fill="{color}" opacity="{opacity}"/>'
    width = field['x_max'] - field['x_min']
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width*scale+2*margin}" height="{height*scale+2*margin}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<rect x="{margin}" y="{margin}" width="{width*scale}" height="{height*scale}" fill="#eee" stroke="black"/>']
    d = cfg['scene']['destination']
    out.append(polygon([(d['x_min'],d['y_min']), (d['x_max'],d['y_min']),
                        (d['x_max'],d['y_max']), (d['x_min'],d['y_max'])], '#cdeccd'))
    for name, m in cfg['scene']['markers'].items():
        x, y = point((m['x'],m['y'])).split(',')
        out.append(f'<circle cx="{x}" cy="{y}" r="{m["radius"]*scale}" fill="#555"/><text x="{x}" y="{y}" fill="white">{name}</text>')
    for key, r in cfg['racks'].items():
        for poly in rack_polygons(cfg, pose(r['pose'])):
            out.append(polygon(poly, '#986b32'))
        for name, color in (('photo_pose', '#d62728'), ('drop_pose', '#1f77b4')):
            x, y = point(pose(r[name])).split(',')
            out.append(f'<circle cx="{x}" cy="{y}" r="4" fill="{color}"/><text x="{float(x)+5}" y="{y}" font-size="12">{key} {name.split("_")[0]}</text>')
    for a in actions:
        if a.kind in ('drive', 'curve'):
            color = '#1f77b4' if a.state in ('CLEAR_RACK','TO_DROP') else '#d62728'
            points = action_samples(a, linear_step=0.02, angular_step=0.05) if a.kind == 'curve' else (a.start, a.target)
            out.append(f'<polyline points="{" ".join(map(point, points))}" fill="none" stroke="{color}" stroke-width="1.5" opacity="0.7"><title>{escape(a.id)}</title></polyline>')
    out.append('</svg>')
    Path(path).write_text('\n'.join(out), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config')
    parser.add_argument('--svg', help='Write a route review SVG')
    parser.add_argument('--actions', help='Write expanded fixed actions as JSON')
    args = parser.parse_args()
    cfg = load_config(args.config)
    mission, elapsed = rehearse(cfg)
    if args.svg:
        write_svg(cfg, mission.actions, args.svg)
    if args.actions:
        Path(args.actions).write_text(json.dumps([a.__dict__ for a in mission.actions], indent=2), encoding='utf-8')
    print(json.dumps(dict(version=cfg['version'], actions=len(mission.actions),
                          result=mission.state, reason=mission.reason,
                          ideal_rehearsal_s=round(elapsed, 2), placed=mission.placed,
                          evidence='offline ideal feedback and mocked photo/cargo, not Gazebo or real robot'),
                     ensure_ascii=False, indent=2))
    if mission.state != 'DONE':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
