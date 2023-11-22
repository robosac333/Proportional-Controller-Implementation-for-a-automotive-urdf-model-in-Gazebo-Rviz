#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion
from std_msgs.msg import  MultiArrayLayout, MultiArrayDimension, Float64MultiArray
from sensor_msgs.msg import JointState

from tf_transformations import euler_from_quaternion
import numpy as np

# global dx, dy, yaw, 
global wheel_radius, steer_angle
global x_target, y_target

def imu_callback( msg):
    global wheel_radius, steer_angle
    global dx_, dy_, yaw
    steer_angle = 0.0
    wheel_radius = 4.0
    last_timestamp = None
    last_timestamp =  msg.header.stamp.sec +  msg.header.stamp.nanosec / 1e9
    if  last_timestamp is not None:
        current_timestamp =  msg.header.stamp.sec +  msg.header.stamp.nanosec / 1e9
        dt = current_timestamp -  last_timestamp

        orientation =  msg.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]

        roll, pitch, yaw = euler_from_quaternion(q)

        # Assigning initial_agular Velocity to neglect the errie imu_angular velocity message
        angular_velocity_ = 25.0
        # Estimate position in x and y based on linear acceleration
        dx_ =  wheel_radius *  angular_velocity_ * dt
        dy_ =  wheel_radius *  angular_velocity_  * dt
        dx_ += dx_
        dy_ += dy_
        a = []
        a.append(dx_)
        print(f"dx = { dx_}, radius = {wheel_radius}, omega={angular_velocity_}, yaw={ yaw}, dt={dt}"  )

    last_timestamp =  msg.header.stamp.sec +  msg.header.stamp.nanosec / 1e9


def main(args=None):
        rclpy.init(args=args)

        global dx_, dy_, yaw

        node = rclpy.create_node('vel_pos_publisher_node')
        joint_position_pub =   node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        wheel_velocities_pub = node.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        joint_state_pub =   node.create_publisher(JointState, '/joint_states', 10)
           
        node1 = rclpy.create_node('imu_data_subscriber_node')
        dx_ = 10.0
        dy_ = 10.0
        yaw = 0.0
        ang_velocity = 0.0
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
        subscription =   node1.create_subscription(Imu, 'imu_plugin/out',  lambda msg: imu_callback(msg), qos_profile)

        K_linear = 1
        x_target = 10.0
        y_target = 10.0
        distance_to_travel = abs(math.sqrt(( x_target**2) + ( y_target**2)))
  
        initial_velocity = 1.0
        steer_angle = 1.0
        distance = abs(math.sqrt((( x_target - dx_)**2) + (( y_target - dy_)**2)))
        print(distance)
        print(distance_to_travel)
        while distance < distance_to_travel and rclpy.ok(): 
            rclpy.spin_once(node1)
            linear_velocity = (distance_to_travel - distance) * K_linear
            
            print(yaw)
            desired_angle = 2.3562 
            if yaw > desired_angle:
                steer_angle = 0.0
            else:
                steer_angle = 1.0

            wheel_velocities = Float64MultiArray(layout=MultiArrayLayout(
                dim=[MultiArrayDimension(label="wheel_velocities", size=4, stride=1)],
                data_offset=0
            ))
            wheel_velocities.data = [-linear_velocity, linear_velocity, -linear_velocity, linear_velocity]

            joint_positions = Float64MultiArray(layout=MultiArrayLayout(
                dim=[MultiArrayDimension(label="joint_positions", size=2, stride=1)],
                data_offset=0
            ))        
            joint_positions.data = [steer_angle, steer_angle]   
            joint_position_pub.publish(joint_positions)
            wheel_velocities_pub.publish(wheel_velocities)       



        node1.get_logger().info('Stopping Car position tracking')
        steer_angle = 0.0
        linear_velocity = 0.0
        wheel_velocities = Float64MultiArray(layout=MultiArrayLayout(
            dim=[MultiArrayDimension(label="wheel_velocities", size=4, stride=1)],
            data_offset=0
        ))
        wheel_velocities.data = [-linear_velocity, linear_velocity, -linear_velocity, linear_velocity]

        joint_positions = Float64MultiArray(layout=MultiArrayLayout(
            dim=[MultiArrayDimension(label="joint_positions", size=2, stride=1)],
            data_offset=0
        ))          
        joint_positions.data = [steer_angle, steer_angle]         
        joint_position_pub.publish(joint_positions)
        wheel_velocities_pub.publish(wheel_velocities)
        node.get_logger().info('Stopping the Car')
        
        rclpy.spin_once(node)
        node1.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

            