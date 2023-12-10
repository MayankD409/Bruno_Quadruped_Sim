#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from math import fabs
from numpy import array_equal

from time import time
from time import monotonic

class PS4Controller:
    def __init__(self, rate):
        rclpy.init()
        self.node = rclpy.create_node("joypad_ramped")
        qos_profile = QoSProfile(depth=10)

        self.publisher = self.node.create_publisher(Joy, "bruno_joy/joy_ramped", qos_profile)
        self.subscription = self.node.create_subscription(Joy, "joy", self.callback, qos_profile)
        self.rate = rate

        # target
        self.target_joy = Joy()
        self.target_joy.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
        self.target_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # last
        self.last_joy = Joy()
        self.last_joy.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
        self.last_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.last_send_time = self.node.get_clock().now()

        self.use_button = True

        self.speed_index = 2
        self.available_speeds = [0.5, 1.0, 3.0, 4.0]

    def run(self):
        while rclpy.ok():
            self.publish_joy()
            rclpy.spin_once(self.node, timeout_sec=1.0 / self.rate)

    def callback(self, msg):
        if self.use_button:
            if msg.buttons[4]:
                self.speed_index -= 1
                if self.speed_index < 0:
                    self.speed_index = len(self.available_speeds) - 1
                self.node.get_logger().info(f"Joystick speed: {self.available_speeds[self.speed_index]}")
                self.use_button = False
            elif msg.buttons[5]:
                self.speed_index += 1
                if self.speed_index >= len(self.available_speeds):
                    self.speed_index = 0
                self.node.get_logger().info(f"Joystick speed: {self.available_speeds[self.speed_index]}")
                self.use_button = False

        if not self.use_button:
            if not (msg.buttons[4] or msg.buttons[5]):
                self.use_button = True

        self.target_joy.axes = msg.axes
        self.target_joy.buttons = msg.buttons

    def ramped_vel(self, v_prev, v_target, t_prev, t_now):
        step = (t_now - t_prev).to_msg().sec
        sign = self.available_speeds[self.speed_index] if (v_target > v_prev) else -self.available_speeds[
            self.speed_index]
        error = fabs(v_target - v_prev)

        # if we can get there within this timestep -> we're done.
        if error < self.available_speeds[self.speed_index] * step:
            return v_target
        else:
            return v_prev + sign * step  # take a step toward the target

    def publish_joy(self):
        t_now = self.node.get_clock().now()

        # determine changes in state
        buttons_change = array_equal(self.last_joy.buttons, self.target_joy.buttons)
        axes_change = array_equal(self.last_joy.axes, self.target_joy.axes)

        # if the desired value is the same as the last value, there's no
        # need to publish the same message again
        if not (buttons_change and axes_change):
            # new message
            joy = Joy()
            if not axes_change:
                # do ramped_vel for every single axis
                for i in range(len(self.target_joy.axes)):
                    if self.target_joy.axes[i] == self.last_joy.axes[i]:
                        joy.axes.append(self.last_joy.axes[i])
                    else:
                        joy.axes.append(self.ramped_vel(self.last_joy.axes[i],
                                                        self.target_joy.axes[i], self.last_send_time, t_now))
            else:
                joy.axes = self.last_joy.axes

            joy.buttons = self.target_joy.buttons
            self.last_joy = joy
            self.publisher.publish(self.last_joy)

        self.last_send_time = t_now

if __name__ == "__main__":
    joystick = PS4Controller(rate=30)
    joystick.run()
