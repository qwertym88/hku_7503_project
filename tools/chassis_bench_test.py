#!/usr/bin/env python3
"""One bounded chassis bench pulse, with telemetry recording and final zero.

Needs ROS2 Jazzy and the normal chassis diagnostics firmware. Never runs an automatic sequence.
Hardware observation (wheel order/direction, actual stopping) remains necessary.
"""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['forward', 'reverse', 'left', 'right'])
    parser.add_argument('--duration', type=float, default=1.0, help='seconds, maximum 2')
    parser.add_argument('--output', type=Path, required=True, help='new JSON evidence path')
    parser.add_argument('--confirm-wheels-raised', action='store_true')
    args = parser.parse_args()
    if not args.confirm_wheels_raised:
        parser.error('requires all four wheels raised, wiring checked and operator ready to stop power')
    if not math.isfinite(args.duration) or not 0.1 <= args.duration <= 2:
        parser.error('duration must be between 0.1 and 2 seconds')
    # Reserve evidence before creating any ROS publisher or sending commands.
    output = args.output.open('x')
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    rclpy.init()
    node = Node('chassis_bench_test')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    started = time.monotonic()
    rows = []
    odom = []
    commands = []
    latest = None
    latest_at = None
    result = 'aborted_before_motion'
    reason = ''
    motion_started = False
    received_start = None
    zero_start = None

    def diagnostic(message):
        nonlocal latest, latest_at
        latest = json.loads(message.data)
        latest_at = time.monotonic()
        rows.append({'t': latest_at-started, 'data': latest})

    def on_odom(message):
        odom.append({'t': time.monotonic()-started,
                     'vx': message.twist.twist.linear.x,
                     'wz': message.twist.twist.angular.z})

    node.create_subscription(String, '/chassis/diagnostics', diagnostic,
        QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Odometry, '/odom', on_odom, 30)

    def validate(require_stopped=False):
        if latest is None or time.monotonic()-latest_at > 1:
            raise RuntimeError('missing or stale diagnostics (>1s)')
        if not all(k in latest for k in ['enabled','pid_output','rx']):
            raise RuntimeError('board lacks required normal chassis diagnostics')
        if latest.get('feedback_missing_mask', 0) or latest.get('feedback_reversed_mask', 0):
            raise RuntimeError('latched motor feedback fault; inspect wiring and reset board')
        # Existing firmware retains last target counts after an explicit stop.
        if require_stopped and (latest['enabled'] or any(latest['pid_output'])):
            raise RuntimeError('motor controller is not stopped before test')
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        if len(publishers) != 1 or publishers[0].node_name != node.get_name():
            raise RuntimeError('cmd_vel publisher ownership is not exclusive')
        if node.count_subscribers('/cmd_vel') != 1:
            raise RuntimeError('expected exactly one cmd_vel subscriber')

    def send(vx=0.0, wz=0.0):
        message = Twist()
        message.linear.x, message.angular.z = vx, wz
        pub.publish(message)
        commands.append({'t': time.monotonic()-started, 'vx': vx, 'wz': wz})

    try:
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if latest is not None and pub.get_subscription_count() > 0:
                validate(require_stopped=True)
                break
        else:
            raise RuntimeError('no chassis diagnostic/subscriber within 5s')
        received_start = latest['rx']
        vx, wz = {'forward': (.03, 0.), 'reverse': (-.03, 0.),
                  'left': (0., .15), 'right': (0., -.15)}[args.action]
        print(f'Start {args.action}: vx={vx}, wz={wz}, duration={args.duration}s', flush=True)
        deadline = time.monotonic()+args.duration
        next_send = time.monotonic()
        motion_started = True
        while time.monotonic() < deadline:
            validate()
            if time.monotonic() >= next_send:
                send(vx, wz)
                next_send = time.monotonic()+.05
            rclpy.spin_once(node, timeout_sec=.005)
        result = 'pulse_completed'
    except (RuntimeError, KeyboardInterrupt, ExternalShutdownException) as error:
        reason = str(error) or 'operator interrupted'
        print(f'Test stopped: {reason}', flush=True)
    finally:
        # Only send a stop if this tool had acquired ownership and sent motion.
        if motion_started:
            zero_start = time.monotonic()-started
            deadline = time.monotonic()+1.2
            next_zero = 0
            while rclpy.ok() and time.monotonic() < deadline:
                if time.monotonic() >= next_zero:
                    send()
                    next_zero = time.monotonic()+.05
                rclpy.spin_once(node, timeout_sec=.01)
        received_delta = latest['rx']-received_start if latest and received_start is not None else 0
        final_age = time.monotonic()-latest_at if latest_at is not None else None
        stopped = (latest is not None and not latest['enabled'] and
                   not any(latest['pid_output']) and final_age < 1)
        has_fault = latest and (latest.get('feedback_missing_mask', 0) or
                               latest.get('feedback_reversed_mask', 0))
        if result == 'pulse_completed' and (not stopped or has_fault or received_delta <= 0):
            result = 'failed_telemetry'
        report = {'action': args.action, 'duration_s': args.duration,
                  'software_result': result, 'reason': reason,
                  'last_diagnostic_age_s': final_age, 'rx_delta_including_stop': received_delta,
                  'physical_observation': 'pending user observation; software is not physical acceptance',
                  'zero_start_t': zero_start, 'commands': commands,
                  'diagnostics': rows, 'odom_samples': odom}
        json.dump(report, output, indent=2)
        output.write('\n')
        output.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(f'Recorded {args.output}; result={result}. Confirm actual wheel directions and stop.', flush=True)
    return 0 if result == 'pulse_completed' else 2


if __name__ == '__main__':
    raise SystemExit(main())
