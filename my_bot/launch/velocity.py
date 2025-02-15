#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('velocity_publisher')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

twist = Twist()
twist.linear.x = 0.2  # Set linear velocity in the x-direction
twist.angular.z = 0.1  # Set angular velocity about the z-axis

pub.publish(twist)