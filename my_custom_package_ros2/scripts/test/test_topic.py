import rclpy
from std_msgs.msg import String
import time

def publish_topic():
    rclpy.init()
    node = rclpy.create_node('topic_publisher')

    # Create a publisher
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()
    msg.data = 'Hello, ROS 2!'

    try:
        while True:
            node.get_logger().info('Publishing: "%s"' % msg.data)
            publisher.publish(msg)
            time.sleep(1)  # Publish a message every 1 second

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publish_topic()
