"""Feedback law shared by the mission and chassis comparison."""

import math

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


def ramp_command(previous, requested, dt, control):
    return (
        previous[0] + clamp(requested[0] - previous[0], control['acceleration_mps2'] * dt),
        previous[1] + clamp(requested[1] - previous[1], control['angular_acceleration_rps2'] * dt),
    )
