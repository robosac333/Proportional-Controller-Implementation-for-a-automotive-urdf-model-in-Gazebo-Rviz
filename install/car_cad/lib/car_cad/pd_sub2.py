#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from tf_transformations import euler_from_quaternion
import numpy as np

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.get_logger().info("IMU subscriber node is running")


        self.joint_position_pub =  self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub =  self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub =  self.create_publisher(JointState, '/joint_states', 10)

        # Define a QoS profilewith RELIABLE reliability
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
        self.subscription =  self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)

        self.last_timestamp = None
        self.dx = 0.0000
        self.dy = 0.0000
        self.yaw = 0.0000
        self.x_target = 10.0
        self.y_target = 10.0
        self.desired_angle_goal = 0.9265
        self.wheel_radius = 4.0
        self.steer_angle = 0.0
        self.angular_velocity = 0.0

    def imu_callback(self, imu_msg):

        if self.last_timestamp is not None:
            current_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1e9
            dt = current_timestamp - self.last_timestamp

            orientation = imu_msg.orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]


            roll, pitch, yaw = euler_from_quaternion(q)
            self.yaw = yaw

            # Extract linear acceleration and angular velocity from the IMU message
            # linear_acceleration = imu_msg.linear_acceleration
            self.angular_velocity = imu_msg.angular_velocity.z

            # Estimate position in x and y based on linear acceleration
            self.dx = self.wheel_radius * self.angular_velocity * np.cos(self.steer_angle) * dt
            self.dy = self.wheel_radius * self.angular_velocity * np.sin(self.steer_angle) * dt


            self.get_logger().info(f"dx = {self.dx}, radius = {self.wheel_radius}, omega={self.angular_velocity}, yaw={self.yaw}, dt={dt}"  )

        self.last_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1e9

    # def robot_vel_publisher(self):
    #     # # Define a QoS profilewith RELIABLE reliability
    #     # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
    #     # self.subscription =  self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)


    #     joint_positions = Float64MultiArray()
    #     wheel_velocities = Float64MultiArray()
    #     linear_velocity = 0.0
    #     self.steer_angle = 0.0
    #     self.get_logger().info(f"entered publisher")
    #     while rclpy.ok():
    #     # Define a QoS profilewith RELIABLE reliability
    #         qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
    #         self.subscription =  self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)
    
    #         K_linear = 0.5
    #         distance = abs(math.sqrt(((self.x_target - self.dx)**2) + ((self.y_target - self.dy)**2)))
    #         linear_velocity = distance * K_linear

    #         # self.steer_angle = 1.0
    #         K_angular = 0.4
    #         self.steer_angle = (self.angular_velocity/ K_angular) + self.yaw
            
    #         if (self.yaw > self.desired_angle_goal):
    #             self.steer_angle = 0.0
    #         # if (self.yaw < -self.desired_angle_goal):
    #         #      self.steer_angle = 0.0

    #         wheel_velocities.data = [-linear_velocity, linear_velocity, -linear_velocity, linear_velocity]
    #         joint_positions.data = [ self.steer_angle,  self.steer_angle]

    #         self.joint_position_pub.publish(joint_positions)
    #         self.wheel_velocities_pub.publish(wheel_velocities)

    #         if (distance < 0.1):
    #             self.steer_angle = 0.0
    #             linear_velocity = 0.0

    #         self.get_logger().info(f"vel = {self.dx}, yaw = {self.yaw}, da = {self.desired_angle_goal}")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('imu_subscriber_node')
    imu_subscriber = ImuSubscriber()
    
    # imu_subscriber.robot_vel_publisher()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
