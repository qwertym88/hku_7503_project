"""Generate or replay a low-speed mission velocity profile and measure odometry."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from .motion import motion_request, ramp_command
from .route import Action, angle, compose, relative


PERIOD = 0.05
CASES = {'forward_1m': (1., 0.), 'diagonal_1m': (1., 1.)}
CONTROL = {
    'turn_rps': 0.20,
    'acceleration_mps2': 0.10,
    'angular_acceleration_rps2': 0.60,
    'position_tolerance_m': 0.02,
    'yaw_tolerance_rad': 0.02,
    'max_cross_track_m': 0.10,
    'max_heading_error_rad': 0.30,
}


def ideal_step(pose, command):
    v, w = command
    turn = w * PERIOD
    # Exact circular arc in the base frame, matching the simulated base.
    distance = v * PERIOD * (math.sin(turn / 2) / (turn / 2) if turn else 1.)
    return compose(pose, (distance * math.cos(turn / 2), distance * math.sin(turn / 2), turn))


def profile_for(case):
    x, y = CASES[case]
    heading = math.atan2(y, x)
    start, target = (0., 0., 0.), (x, y, heading)
    turned = (0., 0., heading)
    actions = []
    if heading:
        actions.append(Action('turn', '', '', 'turn', start, turned, heading, CONTROL['turn_rps'], 10.))
    actions.append(Action('drive', '', '', 'drive', turned, target, math.hypot(x, y), 0.05, 40.))

    current = start
    samples = []
    for action in actions:
        previous = (0., 0.)
        for _ in range(math.ceil(action.timeout / PERIOD)):
            request, done, _, fault = motion_request(action, current, CONTROL)
            if fault:
                raise RuntimeError(f'Ideal {action.kind}: {fault}')
            if done:
                break
            command = ramp_command(previous, request, PERIOD, CONTROL)
            samples.append({'stage': action.kind, 'cmd': command, 'ideal_start': current})
            current = ideal_step(current, command)
            previous = command
        else:
            raise RuntimeError(f'Ideal {action.kind} timed out')
        for _ in range(10):
            samples.append({'stage': 'stop', 'cmd': (0., 0.), 'ideal_start': current})
    return {'case': case, 'period_s': PERIOD, 'goal': target, 'ideal_end': current, 'samples': samples}


def digest(profile):
    # Round numeric integration noise so x86 and ARM give the same profile ID.
    commands = [[round(value, 9) for value in row['cmd']] for row in profile['samples']]
    payload = [profile['case'], profile['period_s'], commands]
    return hashlib.sha256(json.dumps(payload, separators=(',', ':')).encode()).hexdigest()


def error_from(actual, reference):
    dx, dy = actual[0] - reference[0], actual[1] - reference[1]
    return {'dx_m': dx, 'dy_m': dy, 'position_m': math.hypot(dx, dy),
            'yaw_deg': math.degrees(angle(actual[2] - reference[2]))}


def replay(profile, mode):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    expected_base = 'ideal_base' if mode == 'sim' else 'chassis'
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('chassis_compare')
    latest = None
    last_stamp = None
    origin = None
    start_time = None
    feedback_fault = None
    interrupted = False
    odometry = []
    sent = []

    def interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    def receive(msg):
        nonlocal latest, last_stamp, feedback_fault
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vx, wz = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if (msg.header.frame_id, msg.child_frame_id) != ('odom', 'base_footprint'):
            feedback_fault = 'Unexpected odometry frames'
            return
        if (not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w, vx, wz))
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > 0.05):
            feedback_fault = 'Invalid odometry value'
            return
        if stamp <= 0 or (last_stamp is not None and stamp < last_stamp):
            feedback_fault = 'Invalid or backwards odometry timestamp'
            return
        if stamp == last_stamp:
            return
        last_stamp = stamp
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        now = time.monotonic()
        latest = (now, (p.x, p.y, yaw), vx, wz)
        if origin is not None:
            odometry.append({'t_s': now - start_time, 'stamp_ns': stamp,
                             'pose': relative(origin, latest[1]), 'vx': vx, 'wz': wz})

    node.create_subscription(Odometry, '/odom', receive, qos_profile_sensor_data)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    old_signals = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}

    def publish(command, stage):
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        pub.publish(msg)
        sent.append({'t_s': time.monotonic() - start_time, 'stage': stage, 'cmd': command})

    def check_graph():
        return (
            [info.node_name for info in node.get_publishers_info_by_topic('/odom')] == [expected_base]
            and [info.node_name for info in node.get_subscriptions_info_by_topic('/cmd_vel')] == [expected_base]
            and [info.node_name for info in node.get_publishers_info_by_topic('/cmd_vel')] == [node.get_name()]
        )

    def check_feedback():
        if feedback_fault:
            raise RuntimeError(feedback_fault)
        if latest is None or time.monotonic() - latest[0] > 0.25:
            raise RuntimeError('Odometry missing or stale')

    def wait_until(deadline):
        while time.monotonic() < deadline:
            if interrupted:
                raise RuntimeError('Operator interrupted the test')
            rclpy.spin_once(node, timeout_sec=max(0., min(0.02, deadline - time.monotonic())))

    def stationary():
        return abs(latest[2]) <= 0.01 and abs(latest[3]) <= 0.02

    reason = None
    try:
        ready_deadline = time.monotonic() + 8.
        ready_since = None
        while time.monotonic() < ready_deadline:
            wait_until(time.monotonic() + 0.05)
            if feedback_fault:
                raise RuntimeError(feedback_fault)
            now = time.monotonic()
            if (latest is None or now - latest[0] > 0.25 or not stationary() or not check_graph()
                    or pub.get_subscription_count() != 1):
                ready_since = None
                continue
            if ready_since is None:
                ready_since = now
            if now - ready_since >= 0.4:
                break
        else:
            raise RuntimeError(f'Expected stationary {expected_base} with fresh /odom and exclusive /cmd_vel')

        origin = latest[1]
        start_time = time.monotonic()
        for i, sample in enumerate(profile['samples']):
            deadline = start_time + i * PERIOD
            wait_until(deadline)
            if interrupted:
                raise RuntimeError('Operator interrupted the test')
            if time.monotonic() - deadline > PERIOD:
                raise RuntimeError('Velocity replay missed its deadline')
            check_feedback()
            if not check_graph():
                raise RuntimeError('Odometry or velocity topic ownership changed')
            actual = relative(origin, latest[1])
            deviation = error_from(actual, sample['ideal_start'])
            if deviation['position_m'] > 0.25 or abs(deviation['yaw_deg']) > 25.:
                raise RuntimeError('Chassis deviated too far from the reference during replay')
            publish(sample['cmd'], sample['stage'])

        end = start_time + len(profile['samples']) * PERIOD
        wait_until(end)
        check_feedback()
        if not stationary():
            raise RuntimeError('Odometry did not report a stop at the end of the profile')
    except RuntimeError as exc:
        reason = str(exc)
    finally:
        if start_time is not None:
            # Keep publishing zero during teardown while the ROS session is alive.
            for _ in range(5):
                publish((0., 0.), 'final_stop')
                rclpy.spin_once(node, timeout_sec=0.)
                time.sleep(PERIOD)
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)

    final = relative(origin, latest[1]) if origin is not None and latest else None
    return {
        'status': 'stopped' if reason else 'replay_complete', 'reason': reason, 'mode': mode,
        'odom_end': final, 'odom_minus_goal': error_from(final, profile['goal']) if final else None,
        'odom_minus_ideal': error_from(final, profile['ideal_end']) if final else None,
        'commands': sent, 'odometry': odometry,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=CASES)
    parser.add_argument('--run', choices=('sim', 'hardware'), help='Publish the profile to the selected base')
    parser.add_argument('--output', type=Path, required=True, help='Path to a new JSON report')
    args = parser.parse_args()
    profile = profile_for(args.case)
    # Quantize commands themselves as well as their fingerprint across platforms.
    for row in profile['samples']:
        row['cmd'] = tuple(round(v, 9) for v in row['cmd'])
    with args.output.open('x', encoding='utf-8') as output:
        result = replay(profile, args.run) if args.run else {'status': 'reference_only'}
        report = {'profile_id': digest(profile), 'profile': profile,
                  'ideal_minus_goal': error_from(profile['ideal_end'], profile['goal']), **result}
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('profile', 'commands', 'odometry')},
                     indent=2))
    if result['status'] == 'stopped':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
