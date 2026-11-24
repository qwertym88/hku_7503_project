"""Feedback law shared by the mission and chassis comparison."""

from bisect import bisect_right
import math

from .curve import advance
from .route import angle


def motion_request(action, pose, control):
    """Return the requested velocity, goal status, error and any fault."""
    heading = angle(action.target[2] - pose[2])
    if action.kind == 'turn':
        error = abs(heading)
        done = error <= control['yaw_tolerance_rad']
        return (0., 0. if done else clamp(3. * heading, action.speed)), done, error, None

    dx, dy = action.target[0] - pose[0], action.target[1] - pose[1]
    along = dx * math.cos(action.target[2]) + dy * math.sin(action.target[2])
    cross = -dx * math.sin(action.target[2]) + dy * math.cos(action.target[2])
    error = math.hypot(dx, dy)
    done = error <= control['position_tolerance_m']
    if abs(cross) > control['max_cross_track_m'] or abs(heading) > control['max_heading_error_rad']:
        return (0., 0.), done, error, 'cross-track/heading limit'
    if along * action.value < -control['position_tolerance_m'] and not done:
        return (0., 0.), done, error, 'overshot target'
    if done:
        return (0., 0.), done, error, None
    v = math.copysign(min(action.speed, 2.5 * abs(along)), action.value)
    w = clamp(3. * heading + math.copysign(2., v) * cross, control['turn_rps'])
    return (v, w), done, error, None


def clamp(value, limit):
    return max(-limit, min(limit, value))


def curve_request(action, path, progress, pose, control):
    distances, poses, curvatures = path
    x, y, yaw = pose
    first = max(0, bisect_right(distances, progress - 0.02) - 1)
    last = min(len(distances) - 1, bisect_right(distances, progress + 0.12))
    best = math.inf
    for i in range(first, last):
        p, q = poses[i:i + 2]
        dx, dy = q[0] - p[0], q[1] - p[1]
        fraction = max(0., min(1., ((x - p[0]) * dx + (y - p[1]) * dy) / (dx*dx + dy*dy)))
        error = (x - p[0] - fraction * dx)**2 + (y - p[1] - fraction * dy)**2
        if error < best:
            best = error
            progress = distances[i] + fraction * (distances[i + 1] - distances[i])
    remaining = action.value - progress
    i = min(len(distances) - 2, bisect_right(distances, progress) - 1)
    reference = advance(poses[i], progress - distances[i], curvatures[i + 1])
    heading = angle(reference[2] - yaw)
    if best > control['max_cross_track_m']**2 or abs(heading) > control['max_heading_error_rad']:
        return (0., 0.), False, remaining, 'curve cross-track/heading limit'
    goal_error = math.dist(pose[:2], action.target[:2])
    if remaining <= control['position_tolerance_m'] and goal_error <= control['position_tolerance_m']:
        yaw_error = angle(action.target[2] - yaw)
        done = abs(yaw_error) <= control['yaw_tolerance_rad']
        return (0., 0. if done else clamp(3. * yaw_error, control['turn_rps'])), done, abs(yaw_error), None
    along = ((action.target[0] - x) * math.cos(action.target[2])
             + (action.target[1] - y) * math.sin(action.target[2]))
    if remaining < 0.012 and along <= -control['position_tolerance_m']:
        return (0., 0.), False, remaining, 'overshot curve target'
    s_next = bisect_right(distances, progress + 0.09)
    peak = max(abs(k) for k in curvatures[i + 1:s_next + 1])
    turn_feedforward_limit = min(0.90, 0.60 * control['turn_rps'])
    speed = min(action.speed, 0.34, turn_feedforward_limit / max(1., peak),
                2.5 * max(remaining, along) if remaining < 0.07 else 2.5 * remaining)
    i_next = min(len(curvatures) - 1, bisect_right(distances, progress + speed * 0.09))
    cross = -(reference[0] - x) * math.sin(reference[2]) + (reference[1] - y) * math.cos(reference[2])
    omega = speed * (curvatures[i_next] + 32. * cross) + 4.5 * heading
    return (speed, clamp(omega, control['turn_rps'])), False, remaining, None


def ramp_command(previous, requested, dt, control):
    return (
        previous[0] + clamp(requested[0] - previous[0], control['acceleration_mps2'] * dt),
        previous[1] + clamp(requested[1] - previous[1], control['angular_acceleration_rps2'] * dt),
    )
