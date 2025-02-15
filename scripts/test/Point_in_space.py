import rclpy
from geometry_msgs.msg import PointStamped

def publish_point_in_space():
    rclpy.init()
    node = rclpy.create_node('point_publisher')

    publisher = node.create_publisher(PointStamped, 'point_topic', 10)

    point_msg = PointStamped()
    point_msg.header.frame_id = 'odom'  # Set the frame ID
    point_msg.point.x = 2.0  # Set X coordinate
    point_msg.point.y = 2.0  # Set Y coordinate
    point_msg.point.z = 0.0  # Set Z coordinate

    while rclpy.ok():
        publisher.publish(point_msg)
        node.get_logger().info('Publishing point in space')
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publish_point_in_space()
