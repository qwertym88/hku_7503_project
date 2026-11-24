#!/usr/bin/env python3
"""Offline constant-speed route estimate, including firmware command limits.

Excludes acceleration, settling, feedback corrections, photos and actuator time.
Does not connect to ROS, publish commands or claim a physical completion time.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/mission_control'))
from mission_control.route import compile_route, load_config


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def constant(path, name):
    match = re.search(r'^#define\s+' + name + r'\s+\(([0-9.]+)f?\)',
                      path.read_text(), re.M)
    if match is None:
        raise ValueError(f'cannot read numeric constant {name} from {path}')
    return positive(match[1])


def main():
    motion_header = ROOT / 'firmware/chassis/components/car_motion/car_motion.h'
    motor_header = ROOT / 'firmware/chassis/components/motor/motor.h'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'ros2_ws/src/mission_control/config/fixed_route_sim.yaml')
    parser.add_argument('--linear-limit', type=positive, default=constant(motion_header, 'CHASSIS_MAX_LINEAR_MPS'))
    parser.add_argument('--angular-limit', type=positive, default=constant(motion_header, 'CHASSIS_MAX_ANGULAR_RADPS'))
    parser.add_argument('--wheel-limit', type=positive, default=constant(motor_header, 'MOTOR_MAX_SPEED'))
    parser.add_argument('--budget-s', type=positive, default=90.)
    args = parser.parse_args()
    apb = (constant(motion_header, 'ROBOT_WIDTH') + constant(motion_header, 'ROBOT_LENGTH')) / 2
    actions = compile_route(load_config(args.config))
    totals = Counter()
    rows = []
    for action in actions:
        if action.kind == 'drive':
            speed = min(action.speed, args.linear_limit, args.wheel_limit)
            seconds = abs(action.value) / speed
            totals['distance_m'] += abs(action.value)
            totals['drive_s'] += seconds
        elif action.kind == 'turn':
            speed = min(action.speed, args.angular_limit, args.wheel_limit / apb)
            seconds = abs(action.value) / speed
            totals['absolute_rotation_rad'] += abs(action.value)
            totals['turn_s'] += seconds
        elif action.kind == 'wait':
            speed, seconds = None, action.value
            totals['configured_wait_s'] += seconds
        else:
            # Photo/lift/lower latencies are unknown and explicitly excluded.
            speed, seconds = None, None
        if action.kind in ('drive', 'turn'):
            rows.append(dict(id=action.id, kind=action.kind, speed=speed, seconds=seconds))
    estimate = totals['drive_s'] + totals['turn_s'] + totals['configured_wait_s']
    print(json.dumps(dict(
        config=str(args.config),
        limits=dict(linear_mps=args.linear_limit, angular_radps=args.angular_limit,
                    wheel_mps=args.wheel_limit),
        action_counts=dict(Counter(a.kind for a in actions)),
        totals=dict(totals), constant_speed_estimate_s=estimate,
        budget_s=args.budget_s, within_constant_speed_budget=estimate <= args.budget_s,
        excluded='acceleration, settling, corrections, photo and actuator latency',
        motion_actions=rows,
    ), indent=2))


if __name__ == '__main__':
    main()
