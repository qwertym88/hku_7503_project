#!/usr/bin/env python3
"""Local joystick and lifter panel for the differential-drive chassis."""

import argparse
import json
import math
from pathlib import Path
import re
import signal
import subprocess
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8


JOY_TOPIC = '/dase_teleop/joy'
INPUT_TIMEOUT = 0.25
# SDL GameController indices, in the ROS joy package's standardized mapping.
AXIS_TURN, AXIS_DRIVE = 0, 1
BUTTON_DOWN, BUTTON_UP, BUTTON_ENABLE = 0, 3, 9


def firmware_limits():
    header = Path(__file__).resolve().parents[1] / 'firmware/chassis/components/car_motion/car_motion.h'
    limits = dict(re.findall(r'^#define\s+CHASSIS_MAX_(\w+)\s+\(([0-9.]+)f?\)', header.read_text(), re.M))
    return float(limits['LINEAR_MPS']), float(limits['ANGULAR_RADPS']) / 2


class RobotTeleop(Node):
    def __init__(self, gamepad, max_linear, max_angular):
        super().__init__('dase_teleop')
        self.gamepad = gamepad
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.status = '等待底盘连接'
        self.lift_status = ''
        self.odom_at = self.diagnostics_at = self.joy_at = -math.inf
        self.last_motion_at = -math.inf
        self.diagnostics = {}
        self.axes = (0., 0.)
        self.held = False
        self.armed = self.released = False
        self.previous_buttons = None
        self.lift_request = self.lift_pending = None
        self.cmd_pub = self.lift_pub = None
        self.moving = False
        feedback = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self.on_odom, feedback)
        self.create_subscription(String, '/chassis/diagnostics', self.on_diagnostics, feedback)
        if gamepad:
            self.create_subscription(Joy, JOY_TOPIC, self.on_joy, feedback)
        self.create_timer(0.05, self.tick)

    def on_odom(self, _message):
        self.odom_at = time.monotonic()

    def on_diagnostics(self, message):
        try:
            status = json.loads(message.data)
            wheels = status['speed']
            if (len(wheels) != 4 or not all(math.isfinite(v) for v in wheels)
                    or type(status['lifter_rx']) is not int
                    or not isinstance(status['lifter_state'], str)
                    or status['enabled'] not in (0, 1) or status['lift_inhibited'] not in (0, 1)):
                raise ValueError('invalid diagnostic fields')
        except (ValueError, TypeError, KeyError):
            self.diagnostics_at = -math.inf
            return
        self.diagnostics, self.diagnostics_at = status, time.monotonic()

    def set_stick(self, forward, left, held):
        if held and not self.held:
            # Require a press from center after every release or blocked input.
            self.armed = self.released and (forward, left) == (0., 0.)
            self.released = False
        elif not held:
            self.armed = False
        self.axes = (forward, left)
        self.held = held

    def on_joy(self, message):
        now = time.monotonic()
        if (len(message.axes) <= AXIS_DRIVE or len(message.buttons) <= BUTTON_ENABLE
                or not all(math.isfinite(axis) and abs(axis) <= 1.0
                           for axis in (message.axes[AXIS_DRIVE], message.axes[AXIS_TURN]))):
            self.joy_at = -math.inf
            self.previous_buttons = None
            return
        held, up, down = (bool(message.buttons[i]) for i in (BUTTON_ENABLE, BUTTON_UP, BUTTON_DOWN))
        if now - self.joy_at > INPUT_TIMEOUT:
            self.armed = self.released = False
            self.previous_buttons = None
        if self.previous_buttons == (False, False) and up != down and not held:
            self.request_lift(int(up))
        self.previous_buttons = up, down
        self.joy_at = now
        self.set_stick(message.axes[AXIS_DRIVE], message.axes[AXIS_TURN], held)

    def request_lift(self, target):
        self.lift_request = target

    def command_endpoints(self):
        """Wait for the chassis and take ownership only when both topics are free."""
        topics = ('/cmd_vel', '/lifter')
        for topic in topics:
            subscribers = self.get_subscriptions_info_by_topic(topic)
            if sum((info.node_name, info.node_namespace) == ('chassis', '/') for info in subscribers) != 1:
                return False, '等待唯一底盘连接，请检查树莓派 Agent 和 ROS 域 42'

        publishers = [self.get_publishers_info_by_topic(topic) for topic in topics]
        if self.cmd_pub is None and not any(publishers):
            self.cmd_pub = self.create_publisher(Twist, topics[0], QoSProfile(
                depth=1, lifespan=Duration(seconds=0.2)))
            self.lift_pub = self.create_publisher(UInt8, topics[1], QoSProfile(
                depth=1, lifespan=Duration(seconds=0.3)))
            return False, '正在连接底盘控制接口'
        owner = self.get_name(), self.get_namespace()
        exclusive = self.cmd_pub is not None and all(
            [(info.node_name, info.node_namespace) for info in entries] == [owner]
            for entries in publishers)
        if not exclusive:
            return False, '控制接口被占用，请退出固定路线或其他遥控程序'
        return True, ''

    def stop(self, exclusive):
        if self.moving and exclusive:
            self.cmd_pub.publish(Twist())
        self.moving = False

    def tick(self):
        now = time.monotonic()
        request, self.lift_request = self.lift_request, None
        exclusive, reason = self.command_endpoints()
        status = self.diagnostics
        if exclusive:
            if now - self.odom_at > 0.30 or now - self.diagnostics_at > 1.4:
                reason = '等待底盘里程计和诊断'
            elif status.get('command_locked'):
                reason = '底盘已锁定，请按固件恢复流程解锁'
            elif self.gamepad and now - self.joy_at > INPUT_TIMEOUT:
                reason = '等待手柄连接'

        if self.lift_pending is not None:
            baseline, deadline = self.lift_pending
            if status.get('lifter_rx', baseline) > baseline:
                self.lift_pending = None
                result = status.get('lifter_last_result', '')
                self.lift_status = '升降指令已接收' if result in ('accepted', 'duplicate') else f'升降回执（{result}）'
            elif now >= deadline:
                self.lift_pending = None
                self.lift_status = '未收到升降回执，请确认机构状态'

        if request is not None:
            if reason or self.held or self.moving or now - self.last_motion_at < 0.25:
                self.lift_status = '请松开摇杆并等小车停稳，再按升降按钮'
            elif (self.lift_pending is not None or status['lift_inhibited']
                  or status['lifter_state'] != 'idle' or status['enabled']
                  or any(abs(v) > 0.005 for v in status['speed'])):
                self.lift_status = '底盘或升降机构尚未就绪'
            else:
                self.lift_pub.publish(UInt8(data=request))
                self.lift_pending = status['lifter_rx'], now + 1.5
                self.lift_status = '等待升降回执'

        if not reason and (self.lift_pending is not None or status['lift_inhibited']):
            reason = '升降机构占用底盘'
        if reason:
            self.armed = self.released = False
            self.stop(exclusive and not status.get('command_locked'))
            self.status = reason
            return

        if not self.held:
            self.released = True
        if self.armed and self.axes != (0., 0.):
            velocity = Twist()
            velocity.linear.x = self.axes[0] * self.max_linear
            velocity.angular.z = self.axes[1] * self.max_angular
            self.cmd_pub.publish(velocity)
            self.moving, self.last_motion_at = True, now
            self.status = f'行驶中  {velocity.linear.x:+.2f} m/s  {velocity.angular.z:+.2f} rad/s'
        else:
            self.stop(exclusive)
            self.status = '已连接  ·  ' + ('按住 LB 并推动左摇杆' if self.gamepad else '按住圆心并拖动摇杆')

    def close(self):
        if self.moving:
            exclusive, _ = self.command_endpoints()
            self.stop(exclusive and not self.diagnostics.get('command_locked'))
            if exclusive:
                self.cmd_pub.wait_for_all_acked(Duration(seconds=0.15))


class Panel:
    CENTER = 110
    TRAVEL = 68

    def __init__(self, root, robot):
        self.root, self.robot = root, robot
        root.title('小车遥控')
        root.resizable(False, False)
        frame = ttk.Frame(root, padding=16)
        frame.grid()
        self.status = tk.StringVar()
        self.lift_status = tk.StringVar()
        ttk.Label(frame, textvariable=self.status, width=42, anchor='center').pack(pady=(0, 8))

        self.active = False
        if robot.gamepad:
            ttk.Label(frame, text='左摇杆前进 / 后退 / 左右转向\nY / △ 抬升，A / × 放下',
                      justify='center').pack(padx=12, pady=20)
        else:
            self.canvas = tk.Canvas(frame, width=220, height=220, highlightthickness=0, background='#f5f7fa')
            self.canvas.pack(pady=6)
            self.canvas.create_oval(42, 42, 178, 178, outline='#78869a', width=2)
            for x, y, label in ((110, 24, '前进'), (110, 196, '后退'), (22, 110, '左转'), (198, 110, '右转')):
                self.canvas.create_text(x, y, text=label, fill='#23364d')
            self.knob = self.canvas.create_oval(94, 94, 126, 126, fill='#176ba5', outline='')
            self.canvas.bind('<Button-1>', self.press)
            self.canvas.bind('<B1-Motion>', self.drag)
            self.canvas.bind('<ButtonRelease-1>', self.release)
            root.bind('<FocusOut>', self.release)
            root.bind('<Escape>', self.release)

        buttons = ttk.Frame(frame)
        buttons.pack(pady=(10, 4))
        for label, target in (('抬升', 1), ('放下', 0)):
            ttk.Button(buttons, text=label, width=10,
                       command=lambda value=target: robot.request_lift(value)).pack(side='left', padx=6)
        ttk.Label(frame, textvariable=self.lift_status, width=42, anchor='center').pack()
        ttk.Label(frame, text=f'满杆  {robot.max_linear:g} m/s  ·  {robot.max_angular:g} rad/s').pack(pady=(8, 0))
        root.protocol('WM_DELETE_WINDOW', root.quit)
        self.pump()

    def press(self, event):
        if math.hypot(event.x - self.CENTER, event.y - self.CENTER) <= 16:
            self.active = True
            self.robot.set_stick(0., 0., True)

    def drag(self, event):
        if self.active:
            x = event.x - self.CENTER
            y = event.y - self.CENTER
            scale = max(self.TRAVEL, math.hypot(x, y))
            self.draw_knob(x * self.TRAVEL / scale, y * self.TRAVEL / scale)
            if math.hypot(x, y) <= 6:
                self.robot.set_stick(0., 0., True)
            else:
                self.robot.set_stick(-y / scale, -x / scale, True)

    def release(self, _event=None):
        if self.active:
            self.active = False
            self.draw_knob(0., 0.)
            self.robot.set_stick(0., 0., False)

    def draw_knob(self, x, y):
        x += self.CENTER
        y += self.CENTER
        self.canvas.coords(self.knob, x - 16, y - 16, x + 16, y + 16)

    def pump(self):
        for _ in range(8):
            rclpy.spin_once(self.robot, timeout_sec=0.)
        self.status.set(self.robot.status)
        self.lift_status.set(self.robot.lift_status)
        self.root.after(20, self.pump)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gamepad', type=int, nargs='?', const=0, metavar='DEVICE_ID')
    args = parser.parse_args()
    controller = None
    if args.gamepad is not None:
        if args.gamepad < 0:
            parser.error('手柄设备编号必须大于或等于 0')
        try:
            controller = Path(get_package_prefix('joy')) / 'lib/joy/game_controller_node'
        except PackageNotFoundError:
            parser.error('手柄模式需要安装 ros-jazzy-joy')

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        parser.exit(1, f'无法打开遥控窗口，请在本机桌面运行（{exc}）\n')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    robot = device = None
    try:
        robot = RobotTeleop(controller is not None, *firmware_limits())
        if controller is not None:
            device = subprocess.Popen([
                str(controller), '--ros-args', '-r', f'joy:={JOY_TOPIC}', '-p', f'device_id:={args.gamepad}',
                '-p', 'deadzone:=0.12', '-p', 'autorepeat_rate:=20.0', '-p', 'sticky_buttons:=false',
            ])
        Panel(root, robot)
        signal.signal(signal.SIGINT, lambda *_: root.quit())
        signal.signal(signal.SIGTERM, lambda *_: root.quit())
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if robot is not None:
            robot.close()
            robot.destroy_node()
        rclpy.try_shutdown()
        if device is not None and device.poll() is None:
            device.terminate()
            try:
                device.wait(timeout=2)
            except subprocess.TimeoutExpired:
                device.kill()
                device.wait()
        root.destroy()


if __name__ == '__main__':
    main()
