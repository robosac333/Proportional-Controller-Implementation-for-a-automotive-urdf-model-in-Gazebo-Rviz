#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy

# from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# from geometry_msgs.msg import Quaternion
import tf2_ros
from tf2_ros.buffer import Buffer 
from tf2_ros.transform_listener import TransformListener

class ImuSubscriber(Node):
    def __init__(self, node):
        super().__init__('imu_subscriber')
        self.get_logger().info("IMU subscriber node is running")
        self.node = node

        self.joint_position_pub = self.node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.node.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.node.create_publisher(JointState, '/joint_states', 10)

        # Define a QoS profile with RELIABLE reliability
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
        self.subscription = self.node.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)

        self.last_timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.x_target = 10.0
        self.y_target = 10.0
        self.desired_angle_goal = 0.9265

        # Create a tf2 buffer and listener with the node argument
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def imu_callback(self, imu_msg):

        if self.last_timestamp is not None:
            current_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1e9
            dt = current_timestamp - self.last_timestamp

            
            # transform = self.tf_buffer.lookup_transform('world', 'Chassis', rclpy.time.Time().to_msg())
            orientation = imu_msg.orientation
            # q = [orientation.x, orientation.y, orientation.z, orientation.w]
            transform = tf2_ros.TransformStamped()
            transform.header.stamp = imu_msg.header.stamp
            transform.header.frame_id = imu_msg.header.frame_id
            transform.child_frame_id = 'Chassis'
            transform.transform.rotation = orientation
            # Estimate yaw angle based on orientation
            # Convert the orientation to Euler angles

            roll, pitch, self.yaw = self.t

            # Extract linear acceleration and angular velocity from the IMU message
            linear_acceleration = imu_msg.linear_acceleration
            angular_velocity = imu_msg.angular_velocity.z

            # Estimate position in x and y based on linear acceleration
            delta_x = linear_acceleration.x 
            delta_y = linear_acceleration.y 
            # delta_x = linear_acceleration.x * math.cos(self.yaw) * dt
            # delta_y = linear_acceleration.y * math.sin(self.yaw) * dt
            self.x += delta_x
            self.y += delta_y

            self.get_logger().info(f"Estimated Position: x={self.x}, y={self.y}, Yaw={self.yaw}")

        self.last_timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1e9

    # def robot_vel_publisher(self):
    #     joint_positions = Float64MultiArray()
    #     wheel_velocities = Float64MultiArray()
    #     linear_velocity = 0.0
    #     steer_angle = 0.0

    #     # while rclpy.ok():

    #     K_linear = 0.5
    #     distance = abs(math.sqrt(((self.x_target - self.x)**2) + ((self.y_target - self.y)**2)))
    #     linear_velocity = distance * K_linear

    #     steer_angle = 1.0
        
    #     # steer_angle = (self.angular_velocity/ K_angular) + self.yaw
        
    #     if (self.yaw > self.desired_angle_goal):
    #         steer_angle = 0.0
    #     # if (self.yaw < -self.desired_angle_goal):
    #     #     steer_angle = 0.0

    #     wheel_velocities.data = [-linear_velocity, linear_velocity, -linear_velocity, linear_velocity]
    #     joint_positions.data = [steer_angle, steer_angle]

    #     self.joint_position_pub.publish(joint_positions)
    #     self.wheel_velocities_pub.publish(wheel_velocities)

    #     if (distance < 0.1):
    #         steer_angle = 0.0
    #         linear_velocity = 0.0

    #     self.get_logger().info(f"vel = {self.x}, yaw = {self.yaw}, da = {self.desired_angle_goal}")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('imu_subscriber_node')
    imu_subscriber = ImuSubscriber(node)
    imu_subscriber.robot_vel_publisher()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
