#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02  # seconds
        self.wheel_base = 0.156
        self.steering_track = 0.122  # Approximate track width

        self.steering_positions = np.array([0.0, 0.0], float)  # Left and right steering positions
        self.wheel_velocities = np.array([0.0, 0.0], float)  # Left and right wheel velocities

        self.pub_steering = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_velocity = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        steering_array = Float64MultiArray(data=self.steering_positions)
        velocity_array = Float64MultiArray(data=self.wheel_velocities)
        self.pub_steering.publish(steering_array)
        self.pub_velocity.publish(velocity_array)

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

        # RT trigger axis (axes[5]) for linear velocity (range: 0 to 5.0)
        linear_velocity = max(0.0, (1.0 - data.axes[5]) / 2.0 * 5.0)  # Map 1.0 to 0 and -1.0 to 5.0

        # Left joystick horizontal axis (axes[0]) for angular velocity (range: -0.48 to 0.48)
        angular_velocity = data.axes[0] * 0.48

        # Set wheel velocities
        commander.wheel_velocities[0] = linear_velocity - angular_velocity * commander.wheel_base / 2
        commander.wheel_velocities[1] = linear_velocity + angular_velocity * commander.wheel_base / 2

        # Set steering positions
        commander.steering_positions[0] = angular_velocity
        commander.steering_positions[1] = angular_velocity

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
