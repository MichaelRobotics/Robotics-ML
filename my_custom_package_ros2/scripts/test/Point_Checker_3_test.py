#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from math import cos, sin

class LaserRot(object):
    def __init__(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.laser = LaserScan()

        self.node = rclpy.create_node("Point_node")

        self.laserS = self.node.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.closestP = self.node.create_publisher(PointStamped, "/closest_point", 1)
        self.node.get_logger().info('Publishing: "%s"' % msg.data)

    def laser_callback(self, msg):
        self.laser = msg

    def transform_scan(self):

        laser = self.laser.ranges

        shortest_laser = 10000
        point = PointStamped()
        for i in range(len(laser)):
            if laser[i] < shortest_laser:
                angle = self.laser.angle_min + i * self.laser.angle_increment
                x = laser[i] * cos(angle)
                shortest_laser = laser[i]
                point.point.x = x
                point.point.y = shortest_laser * sin(angle)

        point.header.frame_id = 'laser_frame'  # Set the frame ID

        self.closestP.publish(point)


def main(args=None):
    rclpy.init(args=args)
    lr = LaserRot()
    rate = lr.node.create_rate(10)

    while rclpy.ok():
        lr.transform_scan()
        rclpy.spin_once(lr.node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
