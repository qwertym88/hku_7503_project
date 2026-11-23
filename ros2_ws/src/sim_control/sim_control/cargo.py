"""Acknowledged kinematic cargo attachment for Gazebo; no contact/lifter physics."""

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mission_control.route import angle, cargo_offset, compose, load_config, pickup_pose, pose


class CargoSimulator(Node):
    def __init__(self):
        super().__init__('cargo_simulator')
        self.declare_parameter('config', '')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('Cargo simulator requires use_sim_time=true')
        self.cfg = load_config(self.get_parameter('config').value)
        self.io_group = ReentrantCallbackGroup()
        self.client = self.create_client(SetEntityPose, '/world/dase7503_field/set_pose', callback_group=self.io_group)
        self.pub = self.create_publisher(String, '/sim/cargo/status', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/sim/base/status', self.on_base, 10)
        self.create_service(Trigger, '/sim/cargo/reset', self.reset_scene, callback_group=self.io_group)
        for key in self.cfg['order']:
            for kind in ('lift','lower'):
                async def callback(request, response, key=key, kind=kind):
                    return await self.handle_cargo(key, kind, response)
                self.create_service(Trigger, f'/sim/cargo/{key}/{kind}', callback, callback_group=self.io_group)
        self.p = None
        self.velocity = (0.,0.)
        self.odom_received = 0.
        self.base_received = 0.
        self.base_ready = False
        self.carrying = None
        self.placed = []
        self.busy = False
        self.error = ''
        self.pending = None
        self.pending_started = 0.
        self.initialized = set()
        self.last_follow_success = 0.
        self.positions = {k: pose(r['pose']) for k, r in self.cfg['racks'].items()}
        self.create_timer(1/30, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.p = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                  math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.odom_received = time.monotonic()

    def on_base(self, msg):
        self.base_ready, self.base_received = msg.data == 'ready', time.monotonic()

    def request(self, key, p, height=0.):
        req = SetEntityPose.Request()
        req.entity.name = self.cfg['racks'][key]['model']
        req.pose.position.x, req.pose.position.y = p[:2]
        req.pose.position.z = height
        req.pose.orientation.z, req.pose.orientation.w = math.sin(p[2]/2), math.cos(p[2]/2)
        return req

    async def move(self, key, p, height=0.):
        if not self.client.service_is_ready():
            raise RuntimeError('Gazebo set_pose unavailable')
        future = self.client.call_async(self.request(key,p,height))
        started = time.monotonic()
        def deadline():
            if not future.done() and time.monotonic()-started > 1.5:
                future.cancel()
        timer = self.create_timer(0.05, deadline, clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            result = await future
            if result is None or not result.success:
                raise RuntimeError(f'Gazebo rejected/timed out moving {key}')
            self.positions[key] = p
        finally:
            self.destroy_timer(timer)

    def healthy(self):
        now = time.monotonic()
        return (not self.error and len(self.initialized) == 4 and self.client.service_is_ready()
                and self.base_ready and now-self.base_received < 0.5
                and self.p is not None and now-self.odom_received < 0.5
                and (not self.carrying or self.busy or now-self.last_follow_success < 0.75))

    async def handle_cargo(self, key, kind, response):
        if self.busy or not self.healthy():
            response.success, response.message = False, 'Cargo adapter busy/not healthy'
            return response
        if abs(self.velocity[0]) > 0.01 or abs(self.velocity[1]) > 0.02:
            response.success, response.message = False, 'Stop the base before lift/lower'
            return response
        self.busy = True
        try:
            if self.pending is not None:
                result = await self.pending
                self.pending = None
                if result is None or not result.success:
                    raise RuntimeError('Pending cargo pose failed')
            r = self.cfg['racks'][key]
            expected = pickup_pose(r) if kind == 'lift' else pose(r['drop_pose'])
            sim = self.cfg['simulation']
            if math.hypot(self.p[0]-expected[0],self.p[1]-expected[1]) > sim['cargo_alignment_m'] or abs(angle(self.p[2]-expected[2])) > sim['cargo_alignment_rad']:
                raise RuntimeError(f'{key}: base is not aligned at {kind} pose')
            if kind == 'lift':
                if self.carrying or key in self.placed:
                    raise RuntimeError('Cargo already attached / rack already placed')
                target = compose(self.p, cargo_offset(r))
                await self.move(key,target,sim['lift_height_m'])
                self.carrying = key
                self.last_follow_success = time.monotonic()
            else:
                if self.carrying != key:
                    raise RuntimeError(f'{key}: not the attached rack')
                target = compose(self.p, cargo_offset(r))
                await self.move(key,target,0.)
                self.carrying = None
                self.placed.append(key)
            response.success, response.message = True, f'{key} {kind} confirmed by Gazebo'
        except Exception as exc:
            self.error = str(exc)
            response.success, response.message = False, self.error
        finally:
            self.busy = False
        return response

    async def reset_scene(self, request, response):
        if self.busy:
            response.success, response.message = False, 'Cargo operation still in progress'
            return response
        self.busy = True
        try:
            if self.pending is not None:
                await self.pending
                self.pending = None
            for key, r in self.cfg['racks'].items():
                await self.move(key,pose(r['pose']))
            self.carrying = None
            self.placed = []
            self.error = ''
            self.initialized = set(self.cfg['order'])
            response.success, response.message = True, 'All racks restored'
        except Exception as exc:
            self.error = str(exc)
            response.success, response.message = False, self.error
        finally:
            self.busy = False
        return response

    def tick(self):
        now = time.monotonic()
        if not self.busy and not self.error:
            if self.pending is not None:
                if self.pending.done():
                    try:
                        result = self.pending.result()
                        if result is None or not result.success:
                            raise RuntimeError('Gazebo cargo pose rejected')
                        self.positions[self.pending_key] = self.pending_pose
                        self.initialized.add(self.pending_key)
                        self.last_follow_success = now
                    except Exception as exc:
                        self.error = str(exc)
                    self.pending = None
                elif now-self.pending_started > 1.5:
                    self.error = 'Gazebo cargo pose timeout'
                    self.pending.cancel()
                    self.pending = None
            if self.pending is None and not self.error and self.client.service_is_ready():
                key = self.carrying
                if key and self.p is not None:
                    target = compose(self.p,cargo_offset(self.cfg['racks'][key]))
                    height = self.cfg['simulation']['lift_height_m']
                elif len(self.initialized) < 4:
                    key = next(k for k in self.cfg['order'] if k not in self.initialized)
                    target, height = pose(self.cfg['racks'][key]['pose']), 0.
                else:
                    key = None
                if key:
                    self.pending_key, self.pending_pose = key, target
                    self.pending = self.client.call_async(self.request(key,target,height))
                    self.pending_started = now
        self.pub.publish(String(data=json.dumps(dict(healthy=self.healthy(), carrying=self.carrying,
            placed=self.placed, error=self.error, positions=self.positions, simulation_only=True))))


def main(args=None):
    rclpy.init(args=args)
    node = CargoSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
