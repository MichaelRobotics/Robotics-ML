import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Initialize subscribers
        self.closest_point_subscription = self.create_subscription(
            PointStamped,
            '/closest_point',
            self.closest_point_callback,
            10
        )
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Topic for goal poses published by Rviz2
            self.goal_pose_callback,
            10
        )

        # Initialize variables
        self.saved_goal_pose = None
        self.x = 0
        self.y = 0
        self.z = 0
        self.threshold_distance = 1.0

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal pose: {msg}")
        self.saved_goal_pose = msg

    def closest_point_callback(self, msg: PointStamped):
        if msg is not None:
            self.x = msg.point.x
            self.y = msg.point.y
            self.z = msg.point.z

    def safety_control(self):
        distance_to_robot = (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5

        if self.saved_goal_pose is not None and distance_to_robot < self.threshold_distance:
            action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            if action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("NavigateToPose action server exists. Sending a goal...")
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = self.saved_goal_pose.pose

                send_goal_future = action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, send_goal_future)

                if send_goal_future.result() is not None:
                    self.get_logger().info('Goal sent successfully!')
                else:
                    self.get_logger().info('Failed to send the goal.')

            else:
                self.get_logger().info("NavigateToPose action server does not exist or is not available.")

            while True:  # Loop for safety check
                if distance_to_robot >= self.threshold_distance:
                    break  # Break out of safety condition

                if action_client.wait_for_server(timeout_sec=5.0):
                    future = action_client._cancel_all_goals_async()
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None:
                        self.get_logger().info('Cancel all goals request sent successfully')
                    else:
                        self.get_logger().info('Failed to send cancel all goals request')
                else:
                    self.get_logger().info('Action server not available')


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()

    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass

    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
