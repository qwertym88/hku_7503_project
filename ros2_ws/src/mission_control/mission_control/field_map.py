"""Rasterize fixed field geometry and optionally overlay stationary rack walls."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from .clearance import rack_walls


@dataclass
class FieldMap:
    resolution: float
    origin: tuple
    width: int
    height: int
    cells: list

    def save(self, yaml_file):
        yaml_file = Path(yaml_file)
        image = yaml_file.with_suffix('.pgm')
        pixels = bytes(254 if value == 0 else 0
                       for row in range(self.height - 1, -1, -1)
                       for value in self.cells[row * self.width:(row + 1) * self.width])
        image.write_bytes(f'P5\n{self.width} {self.height}\n255\n'.encode('ascii') + pixels)
        yaml_file.write_text(yaml.safe_dump({
            'image': image.name, 'resolution': self.resolution, 'origin': [*self.origin, 0.],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25, 'mode': 'trinary',
        }, sort_keys=False), encoding='utf-8')


def rasterize(scene, racks=None, resolution=0.01):
    field = scene['field']
    xmin, xmax, ymin, ymax = (field[k] for k in ('x_min', 'x_max', 'y_min', 'y_max'))
    # The SDF places a 5 cm wall just outside each field boundary.
    origin = (xmin - 0.05, ymin - 0.05)
    width = math.ceil(round((xmax - xmin + 0.10) / resolution, 9))
    height = math.ceil(round((ymax - ymin + 0.10) / resolution, 9))
    xs = [origin[0] + (i + 0.5) * resolution for i in range(width)]
    ys = [origin[1] + (j + 0.5) * resolution for j in range(height)]
    half = resolution / 2
    cells = [0 if xmin + half - 1e-9 <= x <= xmax - half + 1e-9
             and ymin + half - 1e-9 <= y <= ymax - half + 1e-9 else 100
             for y in ys for x in xs]
    half_diagonal = math.sqrt(2.) * half

    def paint(x, y, radius, distance):
        radius += half_diagonal
        i_min = max(0, math.floor((x - radius - origin[0]) / resolution))
        i_max = min(width, math.ceil((x + radius - origin[0]) / resolution))
        j_min = max(0, math.floor((y - radius - origin[1]) / resolution))
        j_max = min(height, math.ceil((y + radius - origin[1]) / resolution))
        for j in range(j_min, j_max):
            for i in range(i_min, i_max):
                if distance(xs[i], ys[j]) <= half_diagonal:
                    cells[j * width + i] = 100

    for marker in scene['markers'].values():
        x, y, radius = (marker[k] for k in ('x', 'y', 'radius'))
        paint(x, y, radius, lambda px, py: math.hypot(px - x, py - y) - radius)
    for pose in (racks or {}).values():
        for wall in rack_walls(scene, pose):
            paint(*wall.pose[:2], wall.radius, wall.distance_to_point)
    return FieldMap(resolution, origin, width, height, cells)
