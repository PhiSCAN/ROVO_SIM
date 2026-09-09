#!/usr/bin/env python3
"""
ArUco Bot Movement Controller

This script publishes cmd_vel messages to make an ArUco bot move in various patterns.
It's designed to be launched alongside the spawn_aruco.launch.py file.

Available movement patterns:
- circle: Moves in a circular path
- square: Moves in a square path
- figure8: Moves in a figure-8 pattern
- backforth: Moves back and forth in a straight line
- stop: Stays stationary

Parameters:
- cmd_vel_topic: The topic to publish cmd_vel messages to (default: /aruco_bot/cmd_vel)
- movement_pattern: The pattern to follow (default: circle)
- radius: Radius for circular movements (default: 2.0)
- speed: Movement speed in m/s (default: 0.5)
- use_sim_time: Whether to use simulation time (default: True)
"""

import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ArucoBotMovement(Node):
    def __init__(self):
        super().__init__('aruco_bot_movement')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/aruco_bot/cmd_vel')
        self.declare_parameter('movement_pattern', 'circle')
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('use_sim_time', True)
        
        # Get parameter values
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.movement_pattern = self.get_parameter('movement_pattern').get_parameter_value().string_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        # Publish cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Timer for publishing movement commands
        self.update_rate = 20.0  # Hz
        self.dt = 1.0 / self.update_rate
        self.timer = self.create_timer(self.dt, self.update_movement)
        
        # Movement state
        self.start_time = time.time()
        self.last_time = self.start_time
        
        # Setup info
        self.get_logger().info(f"ArUco Bot Movement Controller started")
        self.get_logger().info(f"Publishing to: {cmd_vel_topic}")
        self.get_logger().info(f"Movement pattern: {self.movement_pattern}")
        self.get_logger().info(f"Speed: {self.speed} m/s, Radius: {self.radius} m")
    
    def update_movement(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        self.last_time = current_time
        
        cmd_vel = Twist()
        
        if self.movement_pattern == "circle":
            # Circular motion: angular velocity = linear speed / radius
            cmd_vel.linear.x = self.speed
            cmd_vel.angular.z = self.speed / self.radius
            
        elif self.movement_pattern == "square":
            # Square motion: move in straight lines with 90 degree turns
            side_time = self.radius / self.speed  # Time to traverse one side
            cycle_time = 4 * side_time
            phase = (elapsed % cycle_time) / cycle_time
            
            if phase < 0.25:  # First side
                cmd_vel.linear.x = self.speed
                cmd_vel.angular.z = 0.0
            elif phase < 0.5:  # Second side
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = math.pi / 2 / (0.25 * cycle_time)  # 90 degree turn
            elif phase < 0.75:  # Third side
                cmd_vel.linear.x = self.speed
                cmd_vel.angular.z = 0.0
            else:  # Fourth side
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = math.pi / 2 / (0.25 * cycle_time)  # 90 degree turn
                
        elif self.movement_pattern == "figure8":
            # Figure-8 motion: alternating circular motion
            half_cycle = (math.pi * self.radius) / self.speed
            cycle_time = 2 * half_cycle
            phase = (elapsed % cycle_time) / cycle_time
            
            if phase < 0.5:
                # First half - clockwise circle
                cmd_vel.linear.x = self.speed
                cmd_vel.angular.z = -self.speed / self.radius
            else:
                # Second half - counter-clockwise circle
                cmd_vel.linear.x = self.speed
                cmd_vel.angular.z = self.speed / self.radius
                
        elif self.movement_pattern == "backforth":
            # Back and forth motion
            distance = self.radius
            cycle_time = (2 * distance) / self.speed
            phase = (elapsed % cycle_time) / cycle_time
            
            if phase < 0.5:
                # Move forward
                cmd_vel.linear.x = self.speed
                cmd_vel.angular.z = 0.0
            else:
                # Move backward
                cmd_vel.linear.x = -self.speed
                cmd_vel.angular.z = 0.0
                
        elif self.movement_pattern == "stop":
            # Stay stationary
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            # Default to circular motion
            cmd_vel.linear.x = self.speed
            cmd_vel.angular.z = self.speed / self.radius
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def destroy_node(self):
        # Stop the robot when shutting down
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoBotMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()