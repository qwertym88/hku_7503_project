"""Segment migration checks: match mission behavior, stop faults and expose odom bias."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

from mission_control.chassis_compare import ideal_step
from mission_control.chassis_segment import Segment, make_action
from mission_control.core import Mission
from mission_control.route import load_config


CONFIG = Path(__file__).resolve().parents[1] / 'config/fixed_route_sim.yaml'


@pytest.mark.parametrize('kind,value,speed', [
    ('drive', 1., .2), ('drive', 1., .4), ('drive', -.35, .18),
    ('turn', 90., 1.5), ('turn', -45., .6),
])
def test_segment_matches_mission_through_settled_completion(kind, value, speed):
    cfg = load_config(CONFIG)
    action = make_action(kind, value, speed, cfg['control'])
    mission = Mission(cfg)
    # A single action on an empty field: isolate the existing motion semantics.
    mission.cfg['scene']['field'] = dict(x_min=-10., x_max=10., y_min=-10., y_max=10.)
    mission.cfg['scene']['markers'] = {}
    mission.racks = {}
    mission.actions = [action]
    mission.state = 'RUNNING'
    mission.started = mission.last_tick = 0.
    mission._enter(0.)
    segment = Segment(action, cfg['control'])
    pose, velocity = (0., 0., 0.), (0., 0.)
    commands = []
    for step in range(400):
        now = step * .05
        command = segment.tick(now, pose, velocity)
        assert command == pytest.approx(mission.tick(now, pose, velocity))
        assert segment.complete == (mission.state == 'DONE')
        commands.append(command)
        pose = ideal_step(pose, command)
        velocity = command
        if segment.complete:
            break
    assert segment.complete
    assert commands[-1] == (0., 0.)
    if kind == 'drive':
        assert abs(pose[0]-value) <= cfg['control']['position_tolerance_m']
        assert max(abs(v) for v, _ in commands) <= speed


def test_odom_goal_does_not_imply_ground_goal():
    control = load_config(CONFIG)['control']
    segment = Segment(make_action('drive', 1., .4, control), control)
    odom, velocity = (0., 0., 0.), (0., 0.)
    for step in range(400):
        velocity = segment.tick(step*.05, odom, velocity)
        odom = ideal_step(odom, velocity)
        if segment.complete:
            break
    assert segment.complete
    # A wheel-distance scale error is invisible to the controller.
    true_distance = .96*odom[0]
    assert 1.-true_distance > .04


@pytest.mark.parametrize('pose,match', [((1.02, 0., 0.), 'overshot'),
                                       ((.2, .05, 0.), 'cross-track'),
                                       ((.2, 0., .4), 'heading')])
def test_faults_abort_instead_of_claiming_completion(pose, match):
    control = load_config(CONFIG)['control']
    segment = Segment(make_action('drive', 1., .4, control), control)
    with pytest.raises(RuntimeError, match=match):
        segment.tick(.1, pose, (.4, 0.))


def test_arrival_requires_stopped_velocity_and_settling():
    control = load_config(CONFIG)['control']
    segment = Segment(make_action('drive', 1., .4, control), control)
    for t in (.1, .2, .3):
        assert segment.tick(t, (1., 0., 0.), (.4, 0.)) == (0., 0.)
        assert not segment.complete
    segment.tick(.4, (1., 0., 0.), (0., 0.))
    assert not segment.complete
    segment.tick(.6, (1., 0., 0.), (0., 0.))
    assert segment.complete


@pytest.mark.parametrize('kind,value,speed', [
    ('drive', math.nan, .2), ('drive', 0., .2), ('drive', 1., .46),
    ('drive', 1., math.inf), ('turn', 180., .6), ('turn', 45., 1.6),
])
def test_invalid_commands_rejected_before_ros(kind, value, speed):
    with pytest.raises(ValueError):
        make_action(kind, value, speed, load_config(CONFIG)['control'])


def test_stalled_feedback_and_timeout_fail():
    control = load_config(CONFIG)['control']
    segment = Segment(make_action('drive', 1., .4, control), control)
    segment.tick(0., (0., 0., 0.), (0., 0.))
    with pytest.raises(RuntimeError, match='No progress'):
        segment.tick(3.1, (0., 0., 0.), (0., 0.))
    with pytest.raises(RuntimeError, match='timeout'):
        segment.tick(20., (.5, 0., 0.), (.4, 0.))


@pytest.mark.parametrize('drop_feedback', [False, True])
def test_ros_loop_uses_live_feedback_and_same_publisher_to_stop(monkeypatch, drop_feedback):
    """Virtual ROS/clock: exercise acquisition, command scheduling and fault teardown."""
    from mission_control import chassis_segment as module

    clock = NS(now=100., next_sample=100., pose=(0., 0., 0.), command=(0., 0.),
               callback=None, dropped=False, publications=[], publishers=0)

    class Twist:
        def __init__(self):
            self.linear, self.angular = NS(x=0.), NS(z=0.)

    class Node:
        def __init__(self, name):
            self.name = name

        def get_name(self):
            return self.name

        def create_publisher(self, *_args):
            clock.publishers += 1
            return self

        def create_subscription(self, _type, _topic, callback, _qos):
            clock.callback = callback

        def get_subscription_count(self):
            return 1

        def get_publishers_info_by_topic(self, topic):
            return [NS(node_name='chassis' if topic == '/odom' else self.name)]

        def get_subscriptions_info_by_topic(self, _topic):
            return [NS(node_name='chassis')]

        def publish(self, msg):
            clock.command = (msg.linear.x, msg.angular.z)
            clock.publications.append((clock.now, clock.command))
            if drop_feedback and msg.linear.x:
                clock.dropped = True

        def destroy_node(self):
            pass

    def spin_once(_node, timeout_sec):
        dt = max(timeout_sec, 1e-6)
        v, w = clock.command
        x, y, yaw = clock.pose
        clock.pose = (x + v*math.cos(yaw+w*dt/2)*dt,
                      y + v*math.sin(yaw+w*dt/2)*dt, yaw+w*dt)
        clock.now += dt
        if clock.dropped or clock.now < clock.next_sample:
            return
        clock.next_sample = clock.now+.05
        x, y, yaw = clock.pose
        stamp = int(clock.now*1e9)
        clock.callback(NS(header=NS(frame_id='odom', stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)),
                          child_frame_id='base_footprint',
                          pose=NS(pose=NS(position=NS(x=x, y=y),
                                          orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))),
                          twist=NS(twist=NS(linear=NS(x=v), angular=NS(z=w)))))

    fake_modules = {
        'rclpy': NS(init=lambda **kw: None, shutdown=lambda: None, spin_once=spin_once),
        'rclpy.node': NS(Node=Node),
        'rclpy.qos': NS(qos_profile_sensor_data=None),
        'rclpy.signals': NS(SignalHandlerOptions=NS(NO=None)),
        'geometry_msgs': NS(), 'geometry_msgs.msg': NS(Twist=Twist),
        'nav_msgs': NS(), 'nav_msgs.msg': NS(Odometry=object),
    }
    for name, fake in fake_modules.items():
        monkeypatch.setitem(sys.modules, name, fake)
    monkeypatch.setattr(module, 'time', NS(monotonic=lambda: clock.now))
    control = load_config(CONFIG)['control']
    result = module.run(make_action('drive', 1., .4, control), control, 'hardware')
    assert clock.publishers == 1
    assert result['commands'][-1]['phase'] == 'final_stop'
    assert all((v, w) == (0., 0.) for _, (v, w) in clock.publications[-20:])
    motion_times = [t for t, (v, w) in clock.publications if v or w]
    first_stop = next(t for t, cmd in clock.publications if t > motion_times[-1] and cmd == (0., 0.))
    assert first_stop-motion_times[-1] <= .051
    if drop_feedback:
        assert result['status'] == 'stopped'
        assert 'stale' in result['reason']
        assert not result['goal_reached_s']
    else:
        assert result['status'] == 'odom_goal_reached', result['reason']
        assert abs(result['odom_end'][0]-1.) <= control['position_tolerance_m']
        assert result['goal_reached_s'] < result['elapsed_s']
