"""Forward constant-curvature segments in field coordinates."""

import math


def advance(p, distance, curvature):
    x, y, yaw = p
    delta = distance * curvature
    # Distance along the chord, including the straight-line limit.
    half = delta / 2
    chord = distance * math.sin(half) / half if half else distance
    return (x + chord * math.cos(yaw + half), y + chord * math.sin(yaw + half),
            math.remainder(yaw + delta, math.tau))


def samples(start, segments, linear_step=0.01, angular_step=0.04):
    """Yield arc length, pose and curvature, including all segment boundaries."""
    s = 0.
    yield s, start, segments[0][1]
    for length, curvature in segments:
        n = max(1, math.ceil(length / linear_step), math.ceil(length * abs(curvature) / angular_step))
        for i in range(1, n + 1):
            d = length * i / n
            yield s + d, advance(start, d, curvature), curvature
        start = advance(start, length, curvature)
        s += length


def tangent_paths(start, goal, radius):
    """Enumerate forward circle–tangent–circle connections of one radius."""
    for left in (-1, 1):
        for right in (-1, 1):
            x0 = start[0] - left * radius * math.sin(start[2])
            y0 = start[1] + left * radius * math.cos(start[2])
            x1 = goal[0] - right * radius * math.sin(goal[2])
            y1 = goal[1] + right * radius * math.cos(goal[2])
            d = math.hypot(x1 - x0, y1 - y0)
            lateral = (right - left) * radius
            if d < abs(lateral):
                continue
            heading = (math.atan2(y1 - y0, x1 - x0) - math.asin(lateral / d)
                       if d > 1e-9 else goal[2])
            entry = math.remainder(left * (heading - start[2]), math.tau)
            exit = math.remainder(right * (goal[2] - heading), math.tau)
            if min(entry, exit) < -1e-8:
                # Avoid looping around a turning circle in this compact field.
                continue
            lengths = (max(0., entry) * radius, math.sqrt(max(0., d*d - lateral*lateral)),
                       max(0., exit) * radius)
            segments = tuple((length, curvature) for length, curvature in zip(lengths, (left/radius, 0., right/radius))
                             if length > 1e-8)
            if segments:
                yield segments
