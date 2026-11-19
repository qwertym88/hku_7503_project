#!/usr/bin/env python3
"""Validate a deliberately stationary RAW probe capture and generate gyro bias.

No ROS, device access, motor commands, flashing or automatic header replacement.
Low constant angular motion cannot be distinguished from bias: the operator must
independently keep the chassis stationary throughout the accepted window.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def estimate(report):
    summary = report['summary']
    warmup = summary['warmup_s']
    rows = [r for r in report['imu_samples'] if r['t'] >= warmup]
    diagnostics = [r['data'] for r in report['diagnostics'] if r['t'] >= warmup]
    errors = []
    if report.get('motion_commands_sent') is not False:
        errors.append('capture must explicitly record no motor commands')
    if summary.get('cmd_vel_publishers') or summary.get('cmd_vel_publishers_seen'):
        errors.append('velocity publishers present')
    if len(rows) < 1000 or (rows and rows[-1]['t'] - rows[0]['t'] < 20):
        errors.append('need >=1000 samples spanning >=20s after warmup')
    if not diagnostics:
        errors.append('missing diagnostics during calibration window')
    for d in diagnostics:
        if d.get('enabled') != 0 or d.get('rx') != 0 or d.get('imu_state') != 1:
            errors.append('require disabled motors, rx=0 since reset, healthy IMU')
        if d.get('imu_bias_enabled', 0):
            errors.append('input already has bias compensation; require raw capture')
        if (d.get('imu_errors', 0) or not d.get('imu_valid') or
                d.get('imu_age_ms', 1000) < 0 or d.get('imu_age_ms', 1000) > 100):
            errors.append('unhealthy or stale IMU')
        if any(d.get('target', [])) or any(d.get('pid_output', [])):
            errors.append('nonzero motor targets/outputs')
    if diagnostics and any(d.get('counts') != diagnostics[0].get('counts') for d in diagnostics):
        errors.append('wheel counts changed')
    if any(r.get('frame') != 'imu_link' for r in rows):
        errors.append('unexpected IMU frame')
    if not rows:
        raise ValueError('; '.join(sorted(set(errors))))
    for r in rows:
        values = r['gyro'] + r['accel'] + [r['stamp'], r['t']]
        if len(r['gyro']) != 3 or len(r['accel']) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError('invalid/nonfinite six-axis data')
    for key in ['t', 'stamp']:
        gaps = [b[key] - a[key] for a, b in zip(rows, rows[1:])]
        if any(g <= 0 or g > 0.1 for g in gaps):
            errors.append('duplicate/reversed timestamps or >100ms data gap')
    bias = [statistics.mean(r['gyro'][i] for r in rows) for i in range(3)]
    std = [statistics.stdev(r['gyro'][i] for r in rows) if len(rows) > 1 else 0 for i in range(3)]
    accel_std = [statistics.stdev(r['accel'][i] for r in rows) if len(rows) > 1 else 0 for i in range(3)]
    norms = [math.sqrt(sum(v*v for v in r['accel'])) for r in rows]
    if any(abs(b) > 0.1 or s > 0.01 for b, s in zip(bias, std)):
        errors.append('gyro mean/noise exceeds stationary gate')
    if any(s > 0.15 for s in accel_std) or abs(statistics.mean(norms) - 9.80665) > 0.5:
        errors.append('acceleration inconsistent with a stationary chassis')
    if any(abs(v-bias[i]) > 0.05 for r in rows for i,v in enumerate(r['gyro'])):
        errors.append('gyro motion/transient detected')
    if len(rows) >= 2:
        half = len(rows)//2
        for i in range(3):
            if abs(statistics.mean(r['gyro'][i] for r in rows[:half]) -
                   statistics.mean(r['gyro'][i] for r in rows[half:])) > 0.003:
                errors.append('gyro bias not stable between halves')
    if errors:
        raise ValueError('; '.join(sorted(set(errors))))
    return {'gyro_bias_rad_s': bias, 'gyro_std_rad_s': std, 'samples': len(rows),
            'span_s': rows[-1]['t'] - rows[0]['t'],
            'limitation': 'one temperature/session; operator must rule out constant slow motion'}


def render_header(result, digest):
    values = result['gyro_bias_rad_s']
    constants = '\n'.join(f'#define CHASSIS_IMU_BIAS_{axis}_RADS ({v:.10f}f)'
                          for axis, v in zip('XYZ', values))
    return f'''#pragma once

// Stationary raw capture SHA256: {digest}
// One session only; verify residual bias after flashing, restart and warm-up.
#define CHASSIS_IMU_BIAS_ENABLED 1
{constants}

static inline float chassis_imu_correct_gyro(float raw, float bias)
{{
    return CHASSIS_IMU_BIAS_ENABLED ? raw - bias : raw;
}}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--header', type=Path, help='write candidate header to a NEW path')
    parser.add_argument('--confirm-stationary', action='store_true',
                        help='operator confirms chassis was stationary throughout the window')
    args = parser.parse_args()
    if not args.confirm_stationary:
        parser.error('independent stationary confirmation is required')
    raw = args.capture.read_bytes()
    try:
        result = estimate(json.loads(raw))
    except (ValueError, KeyError, TypeError) as error:
        parser.exit(2, f'Calibration rejected: {error}\n')
    digest = hashlib.sha256(raw).hexdigest()
    result['capture_sha256'] = digest
    if args.header:
        with args.header.open('x') as output:
            output.write(render_header(result, digest))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
