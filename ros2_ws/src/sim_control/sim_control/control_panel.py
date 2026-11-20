#!/usr/bin/env python3
"""Small hold-to-move Tk panel that publishes geometry_msgs/Twist on /cmd_vel."""

import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ControlPanel(Node):
    def __init__(self):
        super().__init__('sim_control_panel')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command = (0.0, 0.0)
        self.root = tk.Tk()
        self.root.title('DASE7503 仿真底盘控制')
        self.linear_speed = tk.DoubleVar(master=self.root, value=0.15)
        self.angular_speed = tk.DoubleVar(master=self.root, value=0.60)
        self.root.resizable(False, False)

        tk.Label(self.root, text='按住方向键控制，松开即停车').grid(
            row=0, column=0, columnspan=3, padx=12, pady=(10, 6))
        tk.Label(self.root, text='线速度 (m/s)').grid(row=1, column=0, sticky='w', padx=8)
        tk.Scale(self.root, variable=self.linear_speed, from_=0.03, to=0.40,
                 resolution=0.01, orient=tk.HORIZONTAL, length=190).grid(
                     row=1, column=1, columnspan=2, padx=8)
        tk.Label(self.root, text='角速度 (rad/s)').grid(row=2, column=0, sticky='w', padx=8)
        tk.Scale(self.root, variable=self.angular_speed, from_=0.10, to=1.20,
                 resolution=0.05, orient=tk.HORIZONTAL, length=190).grid(
                     row=2, column=1, columnspan=2, padx=8)

        self._button('↑ 前进', 3, 1, self.linear_speed, 0.0)
        self._button('← 左转', 4, 0, 0.0, self.angular_speed)
        self._button('■ 停止', 4, 1, 0.0, 0.0, stop=True)
        self._button('右转 →', 4, 2, 0.0, self.angular_speed, sign=-1.0)
        self._button('↓ 后退', 5, 1, self.linear_speed, 0.0, sign=-1.0)
        tk.Label(self.root, text='/cmd_vel 仅使用 linear.x 与 angular.z').grid(
            row=6, column=0, columnspan=3, padx=8, pady=(4, 10))

        self.root.bind_all('<KeyPress>', self.on_key_down)
        self.root.bind_all('<KeyRelease>', self.on_key_up)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(20, self.pump_ros)
        self.root.after(50, self.publish_command)

    def _button(self, label, row, column, linear, angular, sign=1.0, stop=False):
        button = tk.Button(self.root, text=label, width=11)
        button.grid(row=row, column=column, padx=4, pady=4)
        if stop:
            button.bind('<ButtonPress-1>', lambda _event: self.set_command(0.0, 0.0))
        else:
            button.bind('<ButtonPress-1>', lambda _event: self.set_command(
                sign * (linear.get() if isinstance(linear, tk.Variable) else linear),
                sign * (angular.get() if isinstance(angular, tk.Variable) else angular)))
        button.bind('<ButtonRelease-1>', lambda _event: self.set_command(0.0, 0.0))

    def set_command(self, linear, angular):
        self.command = (float(linear), float(angular))

    def on_key_down(self, event):
        key = event.keysym.lower()
        if key in ('w', 'up'):
            self.set_command(self.linear_speed.get(), 0.0)
        elif key in ('s', 'down'):
            self.set_command(-self.linear_speed.get(), 0.0)
        elif key in ('a', 'left'):
            self.set_command(0.0, self.angular_speed.get())
        elif key in ('d', 'right'):
            self.set_command(0.0, -self.angular_speed.get())
        elif key == 'space':
            self.set_command(0.0, 0.0)

    def on_key_up(self, event):
        if event.keysym.lower() in ('w', 'a', 's', 'd', 'up', 'down', 'left', 'right'):
            self.set_command(0.0, 0.0)

    def publish_command(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.command
        self.publisher.publish(msg)
        self.root.after(50, self.publish_command)

    def pump_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(10, self.pump_ros)

    def close(self):
        self.set_command(0.0, 0.0)
        for _ in range(3):
            self.publish_command_once()
        self.root.destroy()

    def publish_command_once(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanel()
    try:
        node.root.mainloop()
    finally:
        node.set_command(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
