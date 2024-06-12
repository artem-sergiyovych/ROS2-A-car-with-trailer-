#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class CarTrailerKinematics(Node):
    def __init__(self):
        super().__init__('car_trailer_kinematics')
        
        # Subscriber to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Initial states
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_theta = 0.0
        self.trailer_x = 0.0
        self.trailer_y = 0.0
        self.trailer_theta = 0.0

        # Car and trailer dimensions
        self.L_car = 2  # Length of the car (approximated)
        self.L_trailer = 2.4  # Length of the trailer (approximated)
        
        self.joint_state = JointState()
        
    def cmd_vel_callback(self, msg):
        # Extract velocity commands
        v = msg.linear.x
        w = msg.angular.z
        
        # Time step for simulation
        dt = 0.1
        
        # Calculate the car's new position
        self.car_x += v * np.cos(self.car_theta) * dt
        self.car_y += v * np.sin(self.car_theta) * dt
        self.car_theta += w * dt
        
        # Calculate the trailer's new position based on the car's movement
        beta = np.arctan2(self.trailer_y - self.car_y, self.trailer_x - self.car_x) - self.car_theta
        beta_dot = (v * np.sin(beta)) / self.L_trailer
        
        self.trailer_x += v * np.cos(self.trailer_theta) * dt
        self.trailer_y += v * np.sin(self.trailer_theta) * dt
        self.trailer_theta += beta_dot * dt
        
        # Update joint states
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.name = [
            'car_trailer_connector_joint', 
            'car_trailer_joint',
            'car_wheel_fl_joint', 'car_wheel_fr_joint', 
            'car_wheel_rl_joint', 'car_wheel_rr_joint', 
            'trailer_wheel_fl_joint', 'trailer_wheel_fr_joint', 
            'trailer_wheel_rl_joint', 'trailer_wheel_rr_joint']
        
        self.joint_state.position = [
            self.car_theta, 
            self.trailer_theta,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish joint state
        self.joint_state_publisher.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = CarTrailerKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

