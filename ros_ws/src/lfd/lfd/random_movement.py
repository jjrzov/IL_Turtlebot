#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import select

import tty
import termios

''' A node that sets the robot to move randomly following either Gaussian or Exponential
distribution.
'''

class RandomMovement(Node):
    def __init__(self):
        super().__init__('random_movement')
        self.publisher = self.create_publisher(Twist, 'diff_drive/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.distribution = 0

        self.get_logger().info('Description of running node')
        self.get_logger().info('Randomly moves robot around')
        self.get_logger().info('Linear and Angular velocities are between 0 and 0.4')
        self.get_logger().info('Press Key:')
        self.get_logger().info('g for Gaussian Distribution')
        self.get_logger().info('e for Exponential Distribution')
        self.get_logger().info('u for Uniform Distribution')
        self.get_logger().info('q to stop node')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            input = select.select([sys.stdin], [], [], 0.1)[0]
            if input:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None

    def timer_callback(self):
        key = self.get_key()
        if key == 'q':
            self.twist.linear.x = 0.0   # NOTE: Doesnt matter as node will terminate
            self.twist.angular.z = 0.0
            self.destroy_node() # Stop node
            return
        
        elif key == 'g':
            self.distribution = 0
            self.get_logger().info('Switching to Gaussian Distribution')

        elif key == 'e':
            self.distribution = 1
            self.get_logger().info('Switching to Exponential Distribution')

        elif key == 'u':
            self.distribution = 2
            self.get_logger().info('Switching to Uniform Distribution')

        
        # Draw samples from distribution
        linear_bound = [0.0, 0.4]
        angular_bound = [-0.4, 0.4]

        if self.distribution == 0:
            # Gaussian Distribution
            linear_mu, linear_sigma = 0.2, 0.1
            angular_mu, angular_sigma = 0.0, 0.2

            linear_sample = np.random.normal(linear_mu, linear_sigma)
            angular_sample = np.random.normal(angular_mu, angular_sigma)

        elif self.distribution == 1:
            # Exponential Distribution
            scale = 0.2

            linear_sample = np.random.default_rng().exponential(scale)
            angular_sample = np.random.default_rng().exponential(scale)

            # Choose between straight, left, or right
            angular_sample *= np.random.choice([-1, 0, 1], p=[1/3, 1/3, 1/3])

        else:
            # Uniform Distribution
            linear_sample = np.random.uniform(linear_bound[0], linear_bound[1])
            angular_sample = np.random.uniform(angular_bound[0], angular_bound[1])

        # Bound sampled velocities to [0.0, 0.4]
        linear_sample = np.clip(linear_sample, linear_bound[0], linear_bound[1])
        angular_sample = np.clip(angular_sample, angular_bound[0], angular_bound[1])

        self.twist.linear.x = linear_sample
        self.twist.angular.z = angular_sample

        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    
    node = RandomMovement()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()