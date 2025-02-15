#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from tf2_ros import Buffer
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from math import cos, sin

class LaserRot(object):
    def __init__(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.laser = LaserScan()

        self.node = rclpy.create_node("compute_closest_obstcl")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        iteration_count = 0

        while iteration_count < 4 and rclpy.ok():
            try:
                all_frames = self.tf_buffer.all_frames_as_yaml()
                print("All TF Frames:")
                print(all_frames)
            except Exception as e:
                print("Exception: {}".format(e))
            rclpy.spin_once(self.node)
            iteration_count += 1
        self.transform = None
        self.get_transform()
        self.laserS = self.node.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.closestP = self.node.create_publisher(PointStamped, "/closest_point", 1)
        self.node.get_logger().info('Publishing: "%s"' % msg.data)

        

    def get_transform(self):
        try:

            iteration_count = 0
            while iteration_count < 4 and rclpy.ok():
                try:
                    all_frames = self.tf_buffer.all_frames_as_yaml()
                    print("All TF Frames:")
                    print(all_frames)
                except Exception as e:
                    print("Exception: {}".format(e))
                rclpy.spin_once(self.node)
                iteration_count += 1

        except tf2_ros.LookupException:
            self.node.get_logger().error("LookupException: Error getting transform")
            print("LookupException: ERROR")

    def laser_callback(self, msg):
        self.laser = msg

    def publish_closest_obstacle(self):

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
        lr.publish_closest_obstacle()
        rclpy.spin_once(lr.node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
