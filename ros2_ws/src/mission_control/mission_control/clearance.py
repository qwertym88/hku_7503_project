"""Metric separation from rack walls and cylindrical markers in the sim field."""

from dataclasses import dataclass
import math

from .curve import advance
from .route import angle, cargo_offset, compose, overlap, rectangle


@dataclass(frozen=True)
class Box:
    pose: tuple
    length: float
    width: float

    @property
    def radius(self):
        return math.hypot(self.length, self.width) / 2

    def distance_to_point(self, x, y):
        px, py, yaw = self.pose
        dx, dy = x - px, y - py
        cos, sin = math.cos(yaw), math.sin(yaw)
        return math.hypot(max(0., abs(cos * dx + sin * dy) - self.length / 2),
                          max(0., abs(-sin * dx + cos * dy) - self.width / 2))

    def distance_to_box(self, other):
        a = rectangle(self.pose, self.length, self.width)
        b = rectangle(other.pose, other.length, other.width)
        if overlap(a, b):
            return 0.
        return min(min(self.distance_to_point(*point) for point in b),
                   min(other.distance_to_point(*point) for point in a))


def rack_walls(scene, rack_pose):
    rack = scene['rack']
    offset = (rack['gap'] + rack['wall_thickness']) / 2
    return [Box(compose(rack_pose, (sign * offset, 0., 0.)),
                rack['wall_thickness'], rack['depth']) for sign in (-1, 1)]


def moving_boxes(cfg, p, carrying):
    body = cfg['scene']['robot']
    yield Box(p, body['length'], body['width'])
    if carrying:
        rack = cfg['scene']['rack']
        yield Box(compose(p, cargo_offset(cfg['racks'][carrying])),
                  rack['gap'] + 2 * rack['wall_thickness'], rack['depth'])


class SceneClearance:
    def __init__(self, cfg, racks, carrying=None, *, docking=None):
        scene = cfg['scene']
        self.bounds = scene['field']
        self.wall_margin = scene['clearance_m']
        self.markers = scene['markers']
        if scene.get('racks_passable', False):
            racks = {}
        self.walls = {key: rack_walls(scene, p) for key, p in racks.items() if key != carrying}
        self.cfg = cfg
        self.carrying = carrying
        self.docking = docking

    def violation(self, p, minimum, *, docking=None, sweep=0., uncertainty=0.):
        """Check a pose or the entire longitudinal sweep about its midpoint."""
        docking = docking or self.docking
        for body in moving_boxes(self.cfg, p, self.carrying):
            # Project the robot's translation onto this box's own axes. For
            # this field's aligned body and transverse rack this is exact, and
            # an enclosing rectangle stays conservative for any oblique cargo.
            relative_yaw = p[2] - body.pose[2]
            box = Box(body.pose, body.length + sweep * abs(math.cos(relative_yaw)),
                      body.width + sweep * abs(math.sin(relative_yaw))) if sweep else body
            x, y, yaw = box.pose
            cos, sin = abs(math.cos(yaw)), abs(math.sin(yaw))
            hx = (cos * box.length + sin * box.width) / 2
            hy = (sin * box.length + cos * box.width) / 2
            walls = self.bounds
            if min(x - hx - walls['x_min'], walls['x_max'] - x - hx,
                   y - hy - walls['y_min'], walls['y_max'] - y - hy) < self.wall_margin + uncertainty:
                return 'field boundary'
            for key, marker in self.markers.items():
                if box.distance_to_point(marker['x'], marker['y']) < marker['radius'] + minimum + uncertainty:
                    return f'marker {key}'
            for key, rack in self.walls.items():
                margin = (self.wall_margin if key == docking else minimum) + uncertainty
                for wall in rack:
                    if (math.hypot(x - wall.pose[0], y - wall.pose[1]) < box.radius + wall.radius + margin
                            and box.distance_to_box(wall) < margin):
                        return f'rack {key}'
        return None


def docking_rack(action, cfg):
    if (action.state in ('ENTER', 'LIFT', 'LOWER_WAIT', 'EXIT')
            or action.state in ('TO_PHOTO', 'DWELL', 'PHOTO')
            and cfg['racks'].get(action.rack, {}).get('photo_in_docking_zone')):
        return action.rack
    return None


def clearance_setting(cfg):
    return cfg.get('planning', {}).get('clearance_m', 0.)


def turn_violation(scene, start, yaw, minimum, *, docking=None):
    delta = angle(yaw - start[2])
    steps = max(1, math.ceil(abs(delta) / 0.07))
    radius = max(math.dist(start[:2], box.pose[:2]) + box.radius
                 for box in moving_boxes(scene.cfg, start, scene.carrying))
    # A corner moves at most this far from its nearest tested heading.
    uncertainty = radius * abs(delta) / (2 * steps)
    for i in range(steps + 1):
        hit = scene.violation((*start[:2], start[2] + delta * i / steps), minimum,
                              docking=docking, uncertainty=uncertainty)
        if hit:
            return hit
    return None


def drive_violation(scene, start, end, minimum, *, docking=None):
    midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2, start[2])
    return scene.violation(midpoint, minimum, docking=docking, sweep=math.dist(start[:2], end[:2]))


def curve_violation(scene, start, segments, minimum, *, docking=None):
    radius = max(math.dist(start[:2], box.pose[:2]) + box.radius
                 for box in moving_boxes(scene.cfg, start, scene.carrying))
    for length, curvature in segments:
        if curvature == 0:
            end = advance(start, length, curvature)
            hit = drive_violation(scene, start, end, minimum, docking=docking)
        else:
            # Bound the distance a point on the body can move from the midpoint of each interval.
            steps = max(1, math.ceil(length * (1 + radius * abs(curvature)) / 0.012))
            ds = length / steps
            uncertainty = ds * (1 + radius * abs(curvature)) / 2
            hit = None
            for i in range(steps):
                hit = scene.violation(advance(start, (i + .5) * ds, curvature), minimum, docking=docking,
                                      uncertainty=uncertainty)
                if hit:
                    break
            end = advance(start, length, curvature)
        if hit:
            return hit
        start = end
    return None


def action_violation(scene, action, minimum):
    docking = docking_rack(action, scene.cfg)
    if action.kind == 'drive':
        return drive_violation(scene, action.start, action.target, minimum, docking=docking)
    if action.kind == 'turn':
        return turn_violation(scene, action.start, action.target[2], minimum, docking=docking)
    if action.kind == 'curve':
        return curve_violation(scene, action.start, action.segments, minimum, docking=docking)
    return scene.violation(action.target, minimum, docking=docking)
