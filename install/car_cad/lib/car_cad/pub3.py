#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

from sensor_msgs.msg import Imu
import math
import tf2_py

class RobotController(Node):

    def __init__(self):
        super().__init__('proportional_controller')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Define a QoS profile with RELIABLE reliability
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.subscription = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, qos_profile)

        # Initialize variables for position and orientation estimation
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0  # Initial orientation in radians
        self.last_timestamp = None

    def robot_vel_publisher(self, x_target, y_target):
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_velocity = 0.0
        steer_angle = 0.0

        while True:
            K_linear = 0.5
            distance = math.sqrt((x_target - self.x) ** 2 + (y_target - self.y) ** 2)
            linear_velocity = distance * K_linear

            K_angular = 0.4
            desired_angle_goal = math.atan2(y_target - self.y, x_target - self.x)
            steer_angle = (self.orientation / K_angular)

            if steer_angle > desired_angle_goal:
                steer_angle = 0.0
            if steer_angle < -desired_angle_goal:
                steer_angle = 0.0

            wheel_velocities.data = [-linear_velocity, linear_velocity, -linear_velocity, linear_velocity]
            joint_positions.data = [steer_angle, steer_angle]

            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)

            if distance < 0.1:
                steer_angle = 0.0
                linear_velocity = 0.0

            self.get_logger().info(f"Steer_angle = {steer_angle}, yaw = {self.x}")

    def imu_callback(self, imu_msg):
        if self.last_timestamp is not None:
            # Calculate the time elapsed since the last IMU message
            current_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1e9
            dt = current_timestamp - self.last_timestamp

            # Extract linear acceleration and angular velocity from the IMU message
            linear_acceleration = imu_msg.linear_acceleration
            self.angular_velocity = imu_msg.angular_velocity.z

            # Estimate position in x and y based on linear acceleration
            delta_x = linear_acceleration.x * math.cos(self.yaw) * self.dt
            delta_y = linear_acceleration.y * math.sin(self.yaw) * self.dt
            self.x += delta_x
            self.y += delta_y

            # Extract the orientation quaternion from the IMU message
            orientation = imu_msg.orientation
            orientation_quat = [orientation.x, orientation.y, orientation.z, orientation.w]

            # Convert the orientation quaternion to Euler angles (roll, pitch, yaw)
            (roll, pitch, self.orientation) = tf2_py.transformations.euler_from_quaternion(orientation_quat)

            # Log the estimated position and orientation
            self.get_logger().info(f"Estimated Position: x={self.x}, y={self.y}, Yaw={self.orientation}")

        # Update the last timestamp for the next calculation
        self.last_timestamp = current_timestamp

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    controller.robot_vel_publisher(10.0, 10.0)  # Pass the linear velocity as an argument
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
