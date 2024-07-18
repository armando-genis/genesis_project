#!/usr/bin/env python3

import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.wheel_base = 0.156

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

class JoySubscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global commander

        twist = Twist()

        # RT trigger axis (axes[5]) for linear velocity (range: 0 to 5.0)
        linear_velocity = max(0.0, (1.0 - data.axes[5]) / 2.0 * 5.0)  # Map 1.0 to 0 and -1.0 to 5.0
        twist.linear.x = linear_velocity

        # Left joystick horizontal axis (axes[0]) for angular velocity (range: -0.48 to 0.48)
        angular_velocity = data.axes[0] * 0.48
        twist.angular.z = angular_velocity

        commander.pub_cmd_vel.publish(twist)

if __name__ == '__main__':
    rclpy.init(args=None)

    commander = Commander()
    joy_subscriber = JoySubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
