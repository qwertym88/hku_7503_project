#!/usr/bin/env python3
"""Exercise the actual bench CLI with a fake ROS graph, clock and diagnostics."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('bench_tool', ROOT/'tools/chassis_bench_test.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.)
        self.angular = SimpleNamespace(z=0.)


class FakeROS:
    def __init__(self, scenario):
        self.scenario = scenario
        self.now = 0.
        self.running = True
        self.callbacks = {}
        self.sent = []
        self.rx = 0
        self.fault = 0
        self.enabled = 0
        self.next_diag = 0
        self.first_motion = None

    def create_publisher(self, *args): return self
    def create_subscription(self, kind, topic, callback, qos): self.callbacks[topic] = callback
    def get_subscription_count(self): return 1
    def count_subscribers(self, topic): return 1
    def get_name(self): return 'chassis_bench_test'
    def get_publishers_info_by_topic(self, topic):
        return [SimpleNamespace(node_name=self.get_name())] * (2 if self.scenario == 'duplicate' else 1)
    def destroy_node(self): pass
    def publish(self, message):
        vx, wz = message.linear.x, message.angular.z
        self.sent.append((self.now, vx, wz))
        self.rx += 1
        self.enabled = int(bool(vx or wz) and not self.fault)
        if (vx or wz) and self.first_motion is None: self.first_motion = self.now

    def spin_once(self, node, timeout_sec):
        self.now += max(timeout_sec, .005)
        if self.scenario == 'fault' and self.first_motion is not None and self.now-self.first_motion >= .3:
            self.fault, self.enabled = 8, 0
        if self.now >= self.next_diag:
            if self.scenario != 'stale' or self.now < .2:
                diagnostic = {'rx':self.rx, 'enabled':self.enabled,
                    'target':[1]*4 if self.enabled or self.scenario=='retained_target' else [0]*4,
                    'pid_output':[20]*4 if self.enabled else [0]*4,
                    'feedback_missing_mask':self.fault, 'feedback_reversed_mask':0}
                self.callbacks['/chassis/diagnostics'](SimpleNamespace(data=json.dumps(diagnostic)))
            self.next_diag = self.now+.1

    def modules(self):
        ros = ModuleType('rclpy')
        ros.init = lambda: None
        ros.shutdown = lambda: setattr(self, 'running', False)
        ros.ok = lambda: self.running
        ros.spin_once = self.spin_once
        node = ModuleType('rclpy.node'); node.Node = lambda name: self
        executor = ModuleType('rclpy.executors'); executor.ExternalShutdownException = type('ExternalShutdownException',(Exception,),{})
        qos = ModuleType('rclpy.qos')
        qos.QoSProfile = lambda **kwargs: kwargs
        qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
        geometry = ModuleType('geometry_msgs.msg'); geometry.Twist = Twist
        nav = ModuleType('nav_msgs.msg'); nav.Odometry = object
        std = ModuleType('std_msgs.msg'); std.String = object
        return {'rclpy':ros, 'rclpy.node':node, 'rclpy.executors':executor,
                'rclpy.qos':qos, 'geometry_msgs':ModuleType('geometry_msgs'),
                'geometry_msgs.msg':geometry, 'nav_msgs':ModuleType('nav_msgs'),
                'nav_msgs.msg':nav, 'std_msgs':ModuleType('std_msgs'), 'std_msgs.msg':std}


class BenchToolTests(unittest.TestCase):
    def run_tool(self, scenario, action='forward', duration='1'):
        ros = FakeROS(scenario)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'record.json'
            argv = ['bench',action,'--duration',duration,'--output',str(path),'--confirm-wheels-raised']
            with patch.dict(sys.modules,ros.modules()), patch.object(sys,'argv',argv), \
                 patch.object(bench,'time',SimpleNamespace(monotonic=lambda:ros.now)), \
                 contextlib.redirect_stdout(io.StringIO()):
                code = bench.main()
            report = json.loads(path.read_text())
        return ros, code, report

    def test_bounded_forward_and_final_zero(self):
        ros, code, report = self.run_tool('normal')
        self.assertEqual(code,0)
        active = [s for s in ros.sent if s[1] or s[2]]
        self.assertTrue(active)
        self.assertTrue(all(s[1:] == (.03,0.) for s in active))
        self.assertLessEqual(active[-1][0]-active[0][0],1.01)
        self.assertEqual(ros.sent[-1][1:],(0.,0.))
        self.assertIn('pending',report['physical_observation'])

    def test_retained_target_on_disabled_controller(self):
        # A remembered setpoint is not an active output after Motor_Stop.
        _, code, _ = self.run_tool('retained_target')
        self.assertEqual(code, 0)

    def test_turn_sign(self):
        ros, code, _ = self.run_tool('normal','right')
        self.assertEqual(code,0)
        self.assertTrue(any(s[1:]==(0.,-.15) for s in ros.sent))

    def test_duplicate_publisher_never_sends(self):
        ros, code, _ = self.run_tool('duplicate')
        self.assertEqual(code,2)
        self.assertFalse(ros.sent)

    def test_feedback_fault_aborts_and_zeroes(self):
        ros, code, report = self.run_tool('fault')
        self.assertEqual(code,2)
        self.assertEqual(ros.sent[-1][1:],(0.,0.))
        self.assertIn('feedback fault',report['reason'])

    def test_stale_feedback_aborts(self):
        ros, code, report = self.run_tool('stale',duration='2')
        self.assertEqual(code,2)
        self.assertIn('stale',report['reason'])
        active = [s for s in ros.sent if s[1] or s[2]]
        self.assertLess(active[-1][0]-active[0][0],1.3)
        self.assertEqual(ros.sent[-1][1:],(0.,0.))


if __name__ == '__main__': unittest.main()
