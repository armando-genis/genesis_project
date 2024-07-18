#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformStamped
from math import sin, cos, tan

class SteeringWheelVelocityLogger(Node):
    def __init__(self):
        super().__init__('steering_wheel_velocity_logger')
        
        self.declare_parameter('wheel_base', 2.0)
        self.declare_parameter('wheel_radius', 0.3)
        self.declare_parameter('track_width', 1.2)

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.fl_steering_position = None
        self.fr_steering_position = None
        self.rr_wheel_velocity = None
        self.rl_wheel_velocity = None

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
    
        self.timer = self.create_timer(0.1, self.compute_odometry)

    def joint_state_callback(self, msg):
        if len(msg.velocity) < 4:
            return

        try:
            fr_steering_position_raw = msg.position[msg.name.index('fr_steering_joint')]
            fl_steering_position_raw = msg.position[msg.name.index('fl_steering_joint')]
            rr_wheel_velocity_raw = msg.velocity[msg.name.index('rr_wheel_joint')]
            rl_wheel_velocity_raw = msg.velocity[msg.name.index('rl_wheel_joint')]
            self.fl_steering_position = self.filter_value(fl_steering_position_raw)
            self.fr_steering_position = self.filter_value(fr_steering_position_raw)
            self.rr_wheel_velocity = self.filter_value(rr_wheel_velocity_raw)
            self.rl_wheel_velocity = self.filter_value(rl_wheel_velocity_raw)
            
        except ValueError as e:
            self.get_logger().error(f"Error extracting joint velocities: {e}")
            return

    def filter_value(self, value):
        if abs(value) < 1e-5:
            return 0.0
        return value

    def compute_odometry(self):
        if self.fl_steering_position is None or self.fr_steering_position is None or self.rr_wheel_velocity is None or self.rl_wheel_velocity is None:
            return
        
        self.get_logger().info("Computing odometry")
        current_time = self.get_clock().now()

        # Calculate the average wheel velocity in m/s
        average_wheel_velocity = (self.rr_wheel_velocity + self.rl_wheel_velocity) / 2.0 * self.wheel_radius

        # Compute the average steering angle
        average_steering_angle = (self.fl_steering_position + self.fr_steering_position) / 2.0

        # Bicycle model calculations
        if abs(average_steering_angle) > 1e-5:  # Avoid division by zero
            turning_radius = self.wheel_base / tan(average_steering_angle)
            angular_velocity = average_wheel_velocity / turning_radius
        else:
            angular_velocity = 0.0

        # Time step
        dt = 0.1  # 100 ms or 0.1 s

        # Update pose
        delta_x = average_wheel_velocity * cos(self.th) * dt
        delta_y = average_wheel_velocity * sin(self.th) * dt
        delta_th = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = sin(self.th / 2.0)
        odom.pose.pose.orientation.w = cos(self.th / 2.0)
        odom.twist.twist.linear.x = average_wheel_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_publisher.publish(odom)

        # Publish the transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = sin(self.th / 2.0)
        transform.transform.rotation.w = cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringWheelVelocityLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
