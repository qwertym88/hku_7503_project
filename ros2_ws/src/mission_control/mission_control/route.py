"""Configuration, fixed route compilation and planar collision checks (no ROS)."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from .curve import advance, samples


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose(value):
    if any(isinstance(value[k], bool) or not isinstance(value[k], (int,float)) or not math.isfinite(value[k]) for k in ('x','y','yaw')):
        raise ValueError('Pose x/y/yaw must be finite numbers')
    return (float(value['x']), float(value['y']), float(value['yaw']))


def compose(parent, child):
    x, y, yaw = parent
    dx, dy, da = child
    return (x + math.cos(yaw) * dx - math.sin(yaw) * dy,
            y + math.sin(yaw) * dx + math.cos(yaw) * dy, angle(yaw + da))


def relative(parent, child):
    dx, dy = child[0] - parent[0], child[1] - parent[1]
    return (math.cos(parent[2]) * dx + math.sin(parent[2]) * dy,
            -math.sin(parent[2]) * dx + math.cos(parent[2]) * dy,
            angle(child[2] - parent[2]))


def pickup_pose(rack):
    return compose(pose(rack['photo_pose']), (rack['enter_distance_m'], 0, 0))


def cargo_offset(rack):
    return relative(pickup_pose(rack), pose(rack['pose']))


def rectangle(p, length, width, padding=0.0):
    return [compose(p, (x, y, 0))[:2]
            for x, y in ((-length / 2 - padding, -width / 2 - padding),
                         (length / 2 + padding, -width / 2 - padding),
                         (length / 2 + padding, width / 2 + padding),
                         (-length / 2 - padding, width / 2 + padding))]


def overlap(a, b):
    """Separating axis test for convex rectangles, including contact."""
    for poly in (a, b):
        for i, point in enumerate(poly):
            nxt = poly[(i + 1) % len(poly)]
            axis = (point[1] - nxt[1], nxt[0] - point[0])
            pa = [x * axis[0] + y * axis[1] for x, y in a]
            pb = [x * axis[0] + y * axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def rack_polygons(cfg, p):
    g = cfg['scene']['rack']
    shift = (g['gap'] + g['wall_thickness']) / 2
    return [rectangle(compose(p, (sign * shift, 0, 0)),
                      g['wall_thickness'], g['depth']) for sign in (-1, 1)]


def inside(poly, bounds):
    return all(bounds['x_min'] <= x <= bounds['x_max'] and
               bounds['y_min'] <= y <= bounds['y_max'] for x, y in poly)


def collision(cfg, robot_pose, rack_poses, carrying=None):
    g = cfg['scene']
    if g.get('racks_passable', False):
        rack_poses = {}
    body = g['robot']
    moving = [rectangle(robot_pose, body['length'], body['width'], g['clearance_m'])]
    if carrying:
        # Full cargo rectangle is conservative while transporting. Stationary
        # racks retain their open channel so entering/exiting remains possible.
        cargo_pose = compose(robot_pose, cargo_offset(cfg['racks'][carrying]))
        moving.append(rectangle(cargo_pose, g['rack']['gap'] + 2 * g['rack']['wall_thickness'],
                                g['rack']['depth'], g['clearance_m']))
    for poly in moving:
        if not inside(poly, g['field']):
            return 'field boundary'
        for key, p in rack_poses.items():
            if key != carrying and any(overlap(poly, wall) for wall in rack_polygons(cfg, p)):
                return f'rack {key}'
        for name, m in g['markers'].items():
            # Conservative square enclosing the cylindrical marker.
            if overlap(poly, rectangle((m['x'], m['y'], 0), 2*m['radius'], 2*m['radius'])):
                return f'marker {name}'
    return None


def _finite_tree(value, path='config'):
    if isinstance(value, dict):
        for k, v in value.items():
            _finite_tree(v, f'{path}.{k}')
    elif isinstance(value, list):
        for i, v in enumerate(value):
            _finite_tree(v, f'{path}[{i}]')
    elif value is None or isinstance(value, (int, float)) and not math.isfinite(value):
        raise ValueError(f'{path}: null/non-finite value')


def qr_approach_side(cfg, rack):
    return rack.get('qr_approach_side', 'negative_y' if cfg.get('qr_faces_negative_y') else None)


def validate_config(cfg):
    _finite_tree(cfg)
    if not isinstance(cfg['simulation_only'], bool) or cfg['frame'] != 'field':
        raise ValueError('Expected a boolean simulation_only and frame: field')
    if not isinstance(cfg.get('qr_faces_negative_y', False), bool):
        raise ValueError('qr_faces_negative_y must be a boolean')
    if sorted(cfg['order']) != list('ABCD') or set(cfg['racks']) != set('ABCD'):
        raise ValueError('order must contain each of A/B/C/D exactly once')
    if not isinstance(cfg['version'], str) or not cfg['version'].strip():
        raise ValueError('version must be a nonempty string')
    for group in ('control', 'timing', 'simulation'):
        for key, value in cfg[group].items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
                raise ValueError(f'{group}.{key} must be a positive number')
    c = cfg['control']
    photo_timeout = c.get('photo_timeout_s', c['effect_timeout_s'])
    if 'photo_first_frame_timeout_s' in c and c['photo_first_frame_timeout_s'] > photo_timeout:
        raise ValueError('First-frame timeout cannot exceed the photo action timeout')
    if max(c['cruise_mps'], c['loaded_mps'], c['approach_mps']) > 0.5 or c['turn_rps'] > 1.5:
        raise ValueError('Configured speed exceeds ideal_base limits (0.5m/s, 1.5rad/s)')
    for name, bounds in ((k, cfg['scene'][k]) for k in ('field', 'destination')):
        if bounds['x_min'] >= bounds['x_max'] or bounds['y_min'] >= bounds['y_max']:
            raise ValueError(f'{name}: invalid bounds')
    for section in ('robot', 'rack'):
        if any(not isinstance(v, (float, int)) or v <= 0 for v in cfg['scene'][section].values()):
            raise ValueError(f'scene.{section}: positive dimensions required')
    if 'planning' in cfg:
        clearance = cfg['planning']['clearance_m']
        if isinstance(clearance, bool) or not isinstance(clearance, (int, float)) or clearance <= 0:
            raise ValueError('planning.clearance_m must be positive and finite')
    if cfg['scene']['clearance_m'] < 0:
        raise ValueError('clearance_m must be nonnegative')
    if not isinstance(cfg['scene'].get('racks_passable', False), bool):
        raise ValueError('scene.racks_passable must be a boolean')
    if set(cfg['scene']['markers']) != {'S', 'E'}:
        raise ValueError('scene.markers must describe the existing S and E models')
    if any(m['radius'] <= 0 for m in cfg['scene']['markers'].values()):
        raise ValueError('marker radius must be positive')
    pose(cfg['start_pose'])
    for name, point in cfg['transit_points'].items():
        if len(point) != 2 or not all(isinstance(v, (int, float)) for v in point):
            raise ValueError(f'transit_points.{name}: expected [x, y]')

    def check_leg(label, leg, field):
        if not isinstance(leg[field], list):
            raise ValueError(f'{label}.{field}: must be an explicit list')
        for name in leg[field]:
            if name not in cfg['transit_points']:
                raise ValueError(f'{label}.{field}: unknown point {name}')
        curve = f'{field}_curve'
        if curve in leg:
            spec = leg[curve]
            if not cfg.get('planning') or not isinstance(spec, dict):
                raise ValueError(f'{label}.{curve}: curve requires a planning clearance and a mapping')
            pose(spec['start'])
            pose(spec['end'])
            if not spec['segments'] or any(
                len(segment) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in segment)
                or segment[0] <= 0 for segment in spec['segments']
            ):
                raise ValueError(f'{label}.{curve}: expected positive arc lengths and signed curvatures')

    if 'marker_scans' in cfg:
        markers = cfg['marker_scans']
        xy = markers['end_xy']
        if len(xy) != 2 or not all(type(v) in (int, float) for v in xy):
            raise ValueError('marker_scans.end_xy: expected [x, y]')
        check_leg('marker_scans', markers, 'to_end')

    models = set()
    for key, r in cfg['racks'].items():
        if r['model'] != f'rack_{key.lower()}' or r['model'] in models:
            raise ValueError(f'{key}: model name must match rack_{key.lower()}')
        models.add(r['model'])
        for field in ('pose', 'photo_pose', 'drop_pose', 'pickup_offset'):
            pose(r[field])
        if not isinstance(r.get('photo_in_docking_zone', False), bool):
            raise ValueError(f'{key}.photo_in_docking_zone must be a boolean')
        if 'qr_approach_side' in r and r['qr_approach_side'] not in ('positive_y', 'negative_y'):
            raise ValueError(f'{key}.qr_approach_side: expected positive_y or negative_y')
        side = qr_approach_side(cfg, r)
        if side is not None:
            photo = pose(r['photo_pose'])
            side_sign = 1 if side == 'positive_y' else -1
            if (side_sign * (photo[1] - r['pose']['y']) <= 0
                    or abs(angle(photo[2] + side_sign * math.pi / 2)) > 1e-6):
                raise ValueError(f'{key}: photograph from the configured {side} side facing the rack')
        for field in ('enter_distance_m', 'clear_distance_m', 'exit_distance_m'):
            if not isinstance(r[field], (int, float)) or r[field] <= 0:
                raise ValueError(f'{key}.{field} must be positive')
        if r.get('clear_direction', 'backward') not in ('forward', 'backward'):
            raise ValueError(f'{key}.clear_direction must be forward or backward')
        for field in ('to_photo', 'to_drop'):
            check_leg(key, r, field)
        pickup = pickup_pose(r)
        delta = relative(pose(r['pose']), pickup)
        offset = pose(r['pickup_offset'])
        if math.hypot(delta[0]-offset[0],delta[1]-offset[1]) > cfg['simulation']['cargo_alignment_m'] or abs(angle(delta[2]-offset[2])) > cfg['simulation']['cargo_alignment_rad']:
            raise ValueError(f'{key}: ENTER does not reach configured pickup_offset')
        if abs(math.cos(delta[2])) > math.sin(cfg['simulation']['cargo_alignment_rad']):
            raise ValueError(f'{key}: pickup heading must follow the rack channel (local ±Y)')
        drop = compose(pose(r['drop_pose']), cargo_offset(r))
        g = cfg['scene']['rack']
        footprint = rectangle(drop, g['gap'] + 2*g['wall_thickness'], g['depth'])
        if not inside(footprint, cfg['scene']['destination']):
            raise ValueError(f'{key}: final rack footprint outside destination')
    return cfg


def load_config(path):
    try:
        with Path(path).open(encoding='utf-8') as f:
            return validate_config(yaml.safe_load(f))
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError(f'Incomplete/invalid route configuration: {exc}') from exc


def marker_scan_pose(cfg, label):
    xy = pose(cfg['start_pose'])[:2] if label == 'START' else cfg['marker_scans']['end_xy']
    marker = cfg['scene']['markers']['S' if label == 'START' else 'E']
    return (*xy, math.atan2(marker['y'] - xy[1], marker['x'] - xy[0]))


@dataclass(frozen=True)
class Action:
    id: str
    rack: str
    state: str
    kind: str
    start: tuple
    target: tuple
    value: float
    speed: float
    timeout: float
    segments: tuple = ()


def compile_route(cfg):
    """Expand fixed waypoint legs into separate turn/drive actions, no planner."""
    actions = []
    current = pose(cfg['start_pose'])
    c = cfg['control']

    def add(key, state, kind, target=None, value=0., speed=0., segments=()):
        nonlocal current
        timeout = c['segment_timeout_s'] if kind in ('turn', 'drive', 'curve') else c['effect_timeout_s']
        if kind == 'photo':
            timeout = c.get('photo_timeout_s', timeout)
        elif kind == 'wait':
            timeout = value + c['effect_timeout_s']
        a = Action(f'{key}_{state}_{len(actions):03d}', key, state, kind,
                   current, target or current, value, speed, timeout, segments)
        actions.append(a)
        current = a.target

    def turn(key, state, yaw):
        diff = angle(yaw - current[2])
        if abs(diff) > 1e-8:
            add(key, state, 'turn', (*current[:2], angle(yaw)), diff, c['turn_rps'])

    def drive(key, state, distance, speed):
        add(key, state, 'drive', compose(current, (distance, 0, 0)), distance, speed)

    def route(key, state, leg, field, final, speed):
        spec = leg.get(f'{field}_curve')
        if spec:
            start, end = pose(spec['start']), pose(spec['end'])
            turn(key, state, start[2])
            segments = tuple(tuple(segment) for segment in spec['segments'])
            actual = current
            for length, curvature in segments:
                actual = advance(actual, length, curvature)
            if (math.dist(current[:2], start[:2]) > 1e-6 or math.dist(actual[:2], final[:2]) > 1e-6
                    or math.dist(actual[:2], end[:2]) > 1e-6 or abs(angle(actual[2] - end[2])) > 1e-6):
                raise ValueError(f'{key}.{field}_curve: arc geometry does not connect the declared poses')
            add(key, state, 'curve', end, sum(length for length, _ in segments), speed, segments)
            turn(key, state, final[2])
            return
        points = leg[field]
        for point in [cfg['transit_points'][name] for name in points] + [final[:2]]:
            dx, dy = point[0] - current[0], point[1] - current[1]
            length = math.hypot(dx, dy)
            if length > 1e-8:
                turn(key, state, math.atan2(dy, dx))
                drive(key, state, length, speed)
        turn(key, state, final[2])

    markers = cfg.get('marker_scans')
    if markers is not None:
        turn('START', 'TO_START', marker_scan_pose(cfg, 'START')[2])
        add('START', 'DWELL', 'wait', value=cfg['timing']['photo_dwell_s'])
        add('START', 'SCAN_START', 'photo')

    for key in cfg['order']:
        r = cfg['racks'][key]
        route(key, 'TO_PHOTO', r, 'to_photo', pose(r['photo_pose']), c['cruise_mps'])
        add(key, 'DWELL', 'wait', value=cfg['timing']['photo_dwell_s'])
        add(key, 'PHOTO', 'photo')
        drive(key, 'ENTER', r['enter_distance_m'], c['approach_mps'])
        add(key, 'LIFT', 'lift')
        add(key, 'LIFT_WAIT', 'wait', value=cfg['timing']['lift_wait_s'])
        clear_sign = 1 if r.get('clear_direction', 'backward') == 'forward' else -1
        drive(key, 'CLEAR_RACK', clear_sign*r['clear_distance_m'], c['approach_mps'])
        route(key, 'TO_DROP', r, 'to_drop', pose(r['drop_pose']), c['loaded_mps'])
        add(key, 'LOWER', 'lower')
        add(key, 'LOWER_WAIT', 'wait', value=cfg['timing']['lower_wait_s'])
        drive(key, 'EXIT', -r['exit_distance_m'], c['approach_mps'])
    if markers is not None:
        route('END', 'TO_END', markers, 'to_end', marker_scan_pose(cfg, 'END'), c['cruise_mps'])
        add('END', 'DWELL', 'wait', value=cfg['timing']['photo_dwell_s'])
        add('END', 'SCAN_END', 'photo')
    return actions


def action_samples(action, linear_step=0.005, angular_step=0.015):
    if action.kind == 'curve':
        yield from (p for _, p, _ in samples(action.start, action.segments, linear_step, angular_step))
        return
    distance = math.hypot(action.target[0]-action.start[0], action.target[1]-action.start[1])
    turn = angle(action.target[2]-action.start[2])
    n = max(1, math.ceil(distance / linear_step), math.ceil(abs(turn) / angular_step))
    for i in range(n + 1):
        t = i/n
        yield (action.start[0] + (action.target[0]-action.start[0])*t,
               action.start[1] + (action.target[1]-action.start[1])*t,
               angle(action.start[2] + turn*t))


def validate_geometry(cfg, actions=None):
    from .clearance import SceneClearance, action_violation, clearance_setting

    actions = compile_route(cfg) if actions is None else actions
    minimum = clearance_setting(cfg)
    racks = {key: pose(r['pose']) for key, r in cfg['racks'].items()}
    stationary = []
    for key, p in racks.items():
        for polygon in rack_polygons(cfg,p):
            if not inside(polygon,cfg['scene']['field']):
                raise ValueError(f'Initial rack {key} outside field')
            for name, other in stationary:
                if overlap(polygon,other):
                    raise ValueError(f'Initial racks overlap: {key}/{name}')
            stationary.append((key,polygon))
            for name,m in cfg['scene']['markers'].items():
                if overlap(polygon,rectangle((m['x'],m['y'],0),2*m['radius'],2*m['radius'])):
                    raise ValueError(f'Initial rack {key} overlaps marker {name}')
    carrying = None
    placed = {}
    for action in actions:
        if minimum:
            hit = action_violation(SceneClearance(cfg, racks, carrying), action, minimum)
            if hit:
                raise ValueError(f'{action.id}: less than {minimum:.3f} m clearance from {hit}')
        for p in action_samples(action):
            hit = collision(cfg, p, racks, carrying)
            if hit:
                raise ValueError(f'{action.id}: collision with {hit} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})')
        if action.kind == 'lift':
            carrying = action.rack
        elif action.kind == 'lower':
            dropped = compose(action.target, cargo_offset(cfg['racks'][action.rack]))
            g = cfg['scene']['rack']
            polygon = rectangle(dropped, g['gap']+2*g['wall_thickness'], g['depth'])
            for name, other in placed.items():
                if overlap(polygon, other):
                    raise ValueError(f'Placed cargo overlap: {name}/{action.rack}')
            placed[action.rack] = polygon
            racks[action.rack] = dropped
            carrying = None
    return actions
