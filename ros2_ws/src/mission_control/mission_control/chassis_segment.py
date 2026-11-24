"""Test one relative drive/turn with the simulation's live feedback controller."""

import argparse
import json
import math
from pathlib import Path
import signal
import time

from .motion import motion_request, ramp_command
from .route import Action, load_config, relative


PERIOD = 0.05


def make_action(kind, value, speed, control):
    limit = 0.45 if kind == 'drive' else 1.5
    if not math.isfinite(value) or value == 0 or (kind == 'turn' and abs(value) >= 180):
        raise ValueError('Use nonzero finite distance, or turn strictly between -180 and 180 degrees')
    if not math.isfinite(speed) or not 0 < speed <= limit:
        raise ValueError(f'Speed must be positive and <= {limit}')
    amount = value if kind == 'drive' else math.radians(value)
    target = (amount, 0., 0.) if kind == 'drive' else (0., 0., amount)
    tolerance = control['position_tolerance_m' if kind == 'drive' else 'yaw_tolerance_rad']
    if abs(amount) <= tolerance:
        raise ValueError('Target must exceed the configured tolerance')
    return Action('segment', '', 'TEST', kind, (0., 0., 0.), target,
                  amount, speed, control['segment_timeout_s'])


class Segment:
    """Same motion, ramp, progress and settling rules as Mission.tick."""

    def __init__(self, action, control):
        self.action, self.control = action, control
        self.command = (0., 0.)
        self.best_error = math.inf
        self.last_progress = self.last_tick = 0.
        self.settled_since = None
        self.complete = False

    def tick(self, elapsed, pose, velocity):
        c = self.control
        if elapsed >= self.action.timeout:
            raise RuntimeError('Segment timeout')
        request, done, error, fault = motion_request(self.action, pose, c)
        if fault:
            raise RuntimeError(fault)
        if not done:
            if error < self.best_error - 0.001:
                self.best_error, self.last_progress = error, elapsed
            elif elapsed - self.last_progress > c['no_progress_s']:
                raise RuntimeError('No progress')
        dt = min(0.1, max(0., elapsed - self.last_tick))
        self.last_tick = elapsed
        self.command = (0., 0.) if done else ramp_command(self.command, request, dt, c)
        stopped = (abs(velocity[0]) <= c['stopped_linear_mps']
                   and abs(velocity[1]) <= c['stopped_angular_rps'])
        if done and stopped:
            if self.settled_since is None:
                self.settled_since = elapsed
            self.complete = elapsed - self.settled_since >= c['settle_s']
        else:
            self.settled_since = None
        return self.command


def run(action, control, mode):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    expected = 'chassis' if mode == 'hardware' else 'ideal_base'
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('chassis_segment')
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    latest = origin = started = last_stamp = None
    fault = None
    interrupted = False
    commands, odometry = [], []
    segment = Segment(action, control)

    def interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    def receive(msg):
        nonlocal latest, last_stamp, fault
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vx, wz = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if (msg.header.frame_id, msg.child_frame_id) != ('odom', 'base_footprint'):
            fault = 'Unexpected odometry frames'
            return
        if (not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w, vx, wz))
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > 0.05):
            fault = 'Invalid odometry value'
            return
        if stamp <= 0 or (last_stamp is not None and stamp < last_stamp):
            fault = 'Invalid/backwards odometry timestamp'
            return
        if stamp == last_stamp:
            return
        last_stamp = stamp
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        now = time.monotonic()
        latest = (now, (p.x, p.y, yaw), (vx, wz))
        if origin is not None:
            odometry.append({'t_s': now-started, 'pose': relative(origin, latest[1]),
                             'vx': vx, 'wz': wz, 'stamp_ns': stamp})

    node.create_subscription(Odometry, '/odom', receive, qos_profile_sensor_data)
    old_signals = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}

    def graph_ok():
        return ([i.node_name for i in node.get_publishers_info_by_topic('/odom')] == [expected]
                and [i.node_name for i in node.get_subscriptions_info_by_topic('/cmd_vel')] == [expected]
                and [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel')] == [node.get_name()]
                and pub.get_subscription_count() == 1)

    def fresh():
        return latest is not None and time.monotonic()-latest[0] <= min(0.25, control['feedback_timeout_s'])

    def stopped():
        return (abs(latest[2][0]) <= control['stopped_linear_mps']
                and abs(latest[2][1]) <= control['stopped_angular_rps'])

    def wait_until(deadline):
        while time.monotonic() < deadline:
            if interrupted:
                raise RuntimeError('Operator interrupted')
            rclpy.spin_once(node, timeout_sec=min(0.01, max(0., deadline-time.monotonic())))

    def publish(command, phase):
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        pub.publish(msg)
        commands.append({'t_s': time.monotonic()-started, 'vx': command[0],
                         'wz': command[1], 'phase': phase})

    reason = None
    goal_reached_s = None
    try:
        ready_since = None
        ready_deadline = time.monotonic()+8.
        while time.monotonic() < ready_deadline:
            wait_until(time.monotonic()+PERIOD)
            if fault:
                raise RuntimeError(fault)
            if not (fresh() and stopped() and graph_ok()):
                ready_since = None
            elif ready_since is None:
                ready_since = time.monotonic()
            elif time.monotonic()-ready_since >= 0.4:
                break
        else:
            raise RuntimeError(f'Expected stationary {expected}, fresh /odom, exclusive /cmd_vel')
        origin, started = latest[1], time.monotonic()
        print(f'Start {action.kind}: target={action.target}, speed limit={action.speed}', flush=True)
        deadline = started
        while not segment.complete:
            wait_until(deadline)
            if interrupted:
                raise RuntimeError('Operator interrupted')
            if time.monotonic()-deadline > PERIOD:
                raise RuntimeError('Control missed a 50 ms deadline')
            if fault or not fresh():
                raise RuntimeError(fault or 'Odometry missing/stale')
            if not graph_ok():
                raise RuntimeError('Odometry/cmd_vel ownership changed')
            command = segment.tick(time.monotonic()-started, relative(origin, latest[1]), latest[2])
            publish(command, 'feedback')
            deadline += PERIOD
        goal_reached_s = time.monotonic()-started
    except RuntimeError as exc:
        reason = str(exc)
    finally:
        # Use the SAME publisher for braking; do not wait for a new CLI node.
        if started is not None:
            for _ in range(20):
                publish((0., 0.), 'final_stop')
                until = time.monotonic()+PERIOD
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=min(0.01, max(0., until-time.monotonic())))
            if not reason and (fault or not fresh() or not stopped() or not graph_ok()):
                reason = fault or 'Final stop not confirmed by fresh odometry'
        final = relative(origin, latest[1]) if origin is not None else None
        if final is not None and not reason:
            _, at_goal, _, final_fault = motion_request(action, final, control)
            if not at_goal or final_fault:
                reason = final_fault or 'Final settled pose outside tolerance'
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    return {'status': 'stopped' if reason else 'odom_goal_reached', 'reason': reason,
            'goal_reached_s': goal_reached_s,
            'elapsed_s': commands[-1]['t_s'] if commands else None,
            'odom_end': final,
            'odom_minus_goal': ([final[i]-action.target[i] for i in range(3)] if final else None),
            'physical_measurement': 'pending tape/independent yaw measurement',
            'commands': commands, 'odometry': odometry}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('drive', 'turn'))
    parser.add_argument('value', type=float, help='Signed meters (drive) or degrees (turn); +turn=left')
    parser.add_argument('--speed', type=float, required=True, help='Maximum m/s (drive) or rad/s (turn)')
    parser.add_argument('--config', type=Path, required=True, help='Simulation route YAML; reuse its control settings')
    parser.add_argument('--run', choices=('sim', 'hardware'), help='Omit to preview without ROS/motion')
    parser.add_argument('--output', type=Path, required=True, help='New JSON report; existing files are refused')
    args = parser.parse_args()
    cfg = load_config(args.config)
    control = cfg['control']
    try:
        action = make_action(args.kind, args.value, args.speed, control)
    except ValueError as exc:
        parser.error(str(exc))
    with args.output.open('x', encoding='utf-8') as output:
        result = run(action, control, args.run) if args.run else {'status': 'preview_only'}
        report = {'config_version': cfg['version'], 'mode': args.run,
                  'action': vars(action), 'control': control, **result}
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('commands', 'odometry')}, indent=2))
    if result['status'] == 'stopped':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
