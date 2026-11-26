"""Plan fixed sim waypoints with heading-aware A* and exact drive sweeps."""

import argparse
from copy import deepcopy
from heapq import heappop, heappush
from itertools import count, product
import math
from pathlib import Path

import yaml

from .clearance import SceneClearance, curve_violation, drive_violation, turn_violation
from .curve import tangent_paths
from .field_map import rasterize
from .route import angle, cargo_offset, compose, load_config, marker_scan_pose, pose, qr_approach_side, validate_geometry


# Each edge is an in-place turn followed by a forward drive, like the executor.
GRID_STEP = 0.10
OFFSETS = [(i, j) for i, j in product(range(-2, 3), repeat=2) if math.gcd(i, j) == 1]
TRACKING_RESERVE = 0.012


def link(scene, start, target, minimum, speed, turn_speed):
    distance = math.dist(start[:2], target[:2])
    yaw = math.atan2(target[1] - start[1], target[0] - start[0]) if distance > 1e-8 else start[2]
    incoming = (*target[:2], yaw)
    if turn_violation(scene, start, yaw, minimum):
        return None
    if distance > 1e-8 and drive_violation(scene, (*start[:2], yaw), incoming, minimum):
        return None
    final_turn = 0.
    if len(target) == 3:
        if turn_violation(scene, incoming, target[2], minimum):
            return None
        final_turn = abs(angle(target[2] - yaw))
    turning = abs(angle(yaw - start[2])) + final_turn
    # Approximate the stop and acceleration overhead incurred at each corner.
    cost = distance / speed + turning / turn_speed
    if distance > 1e-8:
        cost += 0.30
    if turning > 1e-8:
        cost += 0.40
    return incoming, cost


def waypoints(scene, start, goal, minimum, speed, turn_speed):
    if scene.violation(start, minimum) or scene.violation(goal, minimum):
        return None

    serial = count()
    queue = [(0., next(serial), 0., start)]
    costs = {start: 0.}
    parents = {}
    best_goal = math.inf
    end = None
    while queue:
        estimate, _, cost, current = heappop(queue)
        if estimate >= best_goal:
            break
        if costs[current] != cost:
            continue
        direct = link(scene, current, goal, minimum, speed, turn_speed)
        if direct and cost + direct[1] < best_goal:
            best_goal = cost + direct[1]
            end = current
            if current == start:
                return []
        gx, gy = (round(value / GRID_STEP) for value in current[:2])
        neighbors = ((round((gx + dx) * GRID_STEP, 6), round((gy + dy) * GRID_STEP, 6))
                     for dx, dy in OFFSETS)
        for xy in neighbors:
            remaining = math.dist(xy, goal[:2]) / speed
            if cost + math.dist(xy, current[:2]) / speed + remaining >= best_goal:
                continue
            step = link(scene, current, xy, minimum, speed, turn_speed)
            if step is None:
                continue
            arrival, travel = step
            arrival = (*arrival[:2], round(arrival[2], 10))
            candidate = cost + travel
            if candidate < costs.get(arrival, math.inf):
                costs[arrival] = candidate
                parents[arrival] = current
                heappush(queue, (candidate + remaining, next(serial), candidate, arrival))
    if end is None:
        return None

    points = [goal[:2]]
    while end != start:
        points.append(end[:2])
        end = parents[end]
    points.reverse()
    return shortcut(scene, start, goal, points, minimum, speed, turn_speed)


def shortcut(scene, start, goal, points, minimum, speed, turn_speed):
    result = []
    i = 0
    while i < len(points) - 1:
        for j in range(len(points) - 1, i - 1, -1):
            target = goal if j == len(points) - 1 else points[j]
            step = link(scene, start, target, minimum, speed, turn_speed)
            if step is None:
                continue
            if j == len(points) - 1:
                return result
            successor = goal if j + 1 == len(points) - 1 else points[j + 1]
            if link(scene, step[0], successor, minimum, speed, turn_speed):
                start = step[0]
                result.append(points[j])
                i = j + 1
                break
        else:
            return points[:-1]
    return result


def as_pose(p):
    return dict(zip(('x', 'y', 'yaw'), p))


def round_corners(points, headings, radius):
    """Replace interior corners with circular fillets on the original legs."""
    trims = [0.]
    arcs = []
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    for i, (before, after) in enumerate(zip(headings, headings[1:])):
        delta = angle(after - before)
        trim = min(radius * math.tan(abs(delta) / 2), 0.44 * min(lengths[i:i + 2]))
        trims.append(trim)
        if trim > 1e-8:
            r = trim / math.tan(abs(delta) / 2)
            arcs.append((r * abs(delta), math.copysign(1 / r, delta)))
        else:
            arcs.append((0., 0.))
    trims.append(0.)
    segments = []
    for i, length in enumerate(lengths):
        segments.append((length - trims[i] - trims[i + 1], 0.))
        if i < len(arcs) and arcs[i][0] > 0:
            if abs(arcs[i][1]) > 10.:
                return ()
            segments.append(arcs[i])
    return tuple(segments)


def smooth_leg(scene, start, goal, waypoints, minimum):
    points = [start[:2], *waypoints, goal[:2]]
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(points, points[1:])]
    candidates = []

    def add(initial, final, segments):
        if not any(curvature for _, curvature in segments):
            return
        turning = abs(angle(initial - start[2])) + abs(angle(final - goal[2]))
        stops = sum(abs(angle(a - b)) > 1e-8 for a, b in ((initial, start[2]), (final, goal[2])))
        cost = sum(length + 0.025 * length * abs(k) for length, k in segments) + 0.25 * stops + 0.20 * turning
        candidates.append((cost, (*start[:2], initial), (*goal[:2], final), segments))

    for radius in (0.24, 0.17, 0.12):
        add(headings[0], headings[-1], round_corners(points, headings, radius))
    starts = (start[2], headings[0], *(headings[0] + d for d in (-0.70, -0.35, 0.35, 0.70)))
    ends = (goal[2], headings[-1], *(headings[-1] + d for d in (-0.70, -0.35, 0.35, 0.70)))
    for initial, final in product(starts, ends):
        for radius in (0.11, 0.16, 0.24, 0.35, 0.50):
            for segments in tangent_paths((*start[:2], initial), (*goal[:2], final), radius):
                add(initial, final, segments)
    for _, initial, final, segments in sorted(candidates):
        if (not turn_violation(scene, start, initial[2], minimum)
                and not turn_violation(scene, final, goal[2], minimum)
                and not curve_violation(scene, initial, segments, minimum)):
            return dict(start=as_pose(initial), end=as_pose(final), segments=[list(s) for s in segments])
    return None


def plan_route(source, clearance=0.10, *, smooth=True):
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError('Planning clearance must be positive and finite')
    cfg = deepcopy(source)
    cfg['version'] += f'-offline-{clearance * 100:.1f}cm'
    if smooth:
        cfg['version'] += '-curves'
    cfg['planning'] = {'clearance_m': clearance}
    cfg['transit_points'] = {}
    racks = {key: pose(rack['pose']) for key, rack in cfg['racks'].items()}
    p = pose(cfg['start_pose'])
    minimum = clearance + TRACKING_RESERVE
    control = cfg['control']
    markers = cfg.get('marker_scans')
    if markers is not None:
        start_scan = marker_scan_pose(cfg, 'START')
        hit = turn_violation(SceneClearance(cfg, racks), p, start_scan[2], minimum)
        if hit:
            raise ValueError(f'START scan: not enough clearance from {hit}')
        p = start_scan
    for key in cfg['order']:
        rack = cfg['racks'][key]
        original = source['racks'][key]
        empty = SceneClearance(cfg, racks, docking=key if rack.get('photo_in_docking_zone') else None)
        pickup = compose(racks[key], pose(original['pickup_offset']))
        drop = pose(original['drop_pose'])
        exit_pose = compose(drop, (-original['exit_distance_m'], 0., 0.))
        failures = set()
        headings = (pickup[2],)
        if qr_approach_side(cfg, original) is None:
            headings += (angle(pickup[2] + math.pi),)
        for yaw in headings:
            pickup = (*pickup[:2], yaw)
            rack['pickup_offset']['yaw'] = angle(yaw - racks[key][2])
            for enter_distance in dict.fromkeys((original['enter_distance_m'], 0.35, 0.40, 0.30, 0.20, 0.16)):
                photo = compose(pickup, (-enter_distance, 0., 0.))
                rack['photo_pose'] = as_pose(photo)
                rack['enter_distance_m'] = enter_distance
                loaded = SceneClearance(cfg, racks, key)
                dropped = compose(drop, cargo_offset(rack))
                after_drop = SceneClearance(cfg, {**racks, key: dropped})
                hit = (empty.violation(photo, minimum)
                       or drive_violation(empty, photo, pickup, minimum, docking=key)
                       or drive_violation(after_drop, drop, exit_pose, minimum, docking=key)
                       or after_drop.violation(exit_pose, minimum))
                if hit:
                    failures.add(hit)
                    continue
                to_photo = waypoints(empty, p, photo, minimum, control['cruise_mps'], control['turn_rps'])
                if to_photo is None:
                    failures.add('no approach to photo pose')
                    continue
                preferred = original['clear_distance_m'] * (1 if original['clear_direction'] == 'forward' else -1)
                departures = dict.fromkeys((preferred, 0.10, 0.06, 0.15, 0.20, 0.25, 0.30, 0.35,
                                             -0.10, -0.20, -0.25, -0.30, -0.35))
                for departure in departures:
                    clear = compose(pickup, (departure, 0., 0.))
                    hit = drive_violation(loaded, pickup, clear, minimum)
                    if hit:
                        failures.add(hit)
                        continue
                    to_drop = waypoints(loaded, clear, drop, minimum, control['loaded_mps'], control['turn_rps'])
                    if to_drop is None:
                        failures.add('no loaded route to drop pose')
                        continue
                    rack['clear_distance_m'] = abs(departure)
                    rack['clear_direction'] = 'forward' if departure > 0 else 'backward'
                    break
                else:
                    continue
                break
            else:
                continue
            break
        else:
            raise ValueError(f'Rack {key}: no route at {minimum:.3f} m planning clearance '
                             f'({", ".join(sorted(failures))})')

        for name, points in (('to_photo', to_photo), ('to_drop', to_drop)):
            rack[name] = []
            rack.pop(f'{name}_curve', None)
            for i, point in enumerate(points, 1):
                label = f'{key.lower()}_{name}_{i:02d}'
                cfg['transit_points'][label] = list(point)
                rack[name].append(label)
        if smooth:
            for name, scene, start, goal, points in (
                ('to_photo', empty, p, photo, to_photo), ('to_drop', loaded, clear, drop, to_drop),
            ):
                curve = smooth_leg(scene, start, goal, points, minimum)
                if curve:
                    rack[f'{name}_curve'] = curve
        racks[key] = dropped
        p = exit_pose
    if markers is not None:
        end_scan = marker_scan_pose(cfg, 'END')
        empty = SceneClearance(cfg, racks)
        points = waypoints(empty, p, end_scan, minimum, control['cruise_mps'], control['turn_rps'])
        if points is None:
            raise ValueError(f'END scan: no route with {minimum:.3f} m planning clearance')
        markers['to_end'] = []
        markers.pop('to_end_curve', None)
        for i, point in enumerate(points, 1):
            label = f'to_end_{i:02d}'
            cfg['transit_points'][label] = list(point)
            markers['to_end'].append(label)
        if smooth and math.dist(p[:2], end_scan[:2]) > 1e-8:
            curve = smooth_leg(empty, p, end_scan, points, minimum)
            if curve:
                markers['to_end_curve'] = curve
    validate_geometry(cfg)
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--clearance', type=float, default=0.10)
    parser.add_argument('--polyline', action='store_true', help='Export the original stop-and-turn route')
    parser.add_argument('--map', type=Path, help='Write the static field map YAML and PGM')
    args = parser.parse_args()
    try:
        cfg = plan_route(load_config(args.config), args.clearance, smooth=not args.polyline)
    except ValueError as exc:
        parser.exit(1, f'{exc}\n')
    args.output.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding='utf-8')
    if args.map:
        rasterize(cfg['scene']).save(args.map)


if __name__ == '__main__':
    main()
