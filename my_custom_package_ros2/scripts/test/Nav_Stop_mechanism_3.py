import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class SafetyNode(object):

    def __init__(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.node = rclpy.create_node("compute_closest_obstcl")
        # closest point parameters
        self.x = 0
        self.y = 0

        # Create a subscriber to the "/closest_point" topic with a callback function
        self.closest_point_subscription = self.node.create_subscription(
            PointStamped,
            '/closest_point',
            self.closest_point_callback,
            10
        )

        self.goal_pose_subscription= self.node.create_subscription(
            PoseStamped,
            '/goal_pose',  # Topic for goal poses published by Rviz2
            self.goal_pose_callback,
            10  # QoS profile history depth
        )
        self.goal_pose_subscription

        # Threshold distances
        self.threshold_distance = 0.5

        # Saved point for navigation
        self.saved_point = None

        # Was safety used already?
        self.safety_run = False

        # is running?
        self.isrunning = False

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal pose: {msg}")

        # Save the goal pose clicked by the user
        self.saved_goal_pose = msg

    def closest_point_callback(self, msg: PointStamped):
        # Extract coordinates from the received message
        self.x = msg.point.x
        self.y = msg.point.y
        print(f'msg callback x{msg.point.x}')
        print(f'msg callback y{msg.point.y}')
        print(f'msg  x{self.x}')
        print(f'msg  y{self.y}')

    def safety_controll(self):
        print(f'msg  x safe{self.x}')
        print(f'msg  y{self.y}')
        distance_to_robot = (self.x ** 2 + self.y ** 2) ** 0.5

        action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        if self.safety_run == True and distance_to_robot > self.threshold_distance and isrunning == False:
            
            server_exists = action_client.wait_for_server(timeout_sec=1.0)

            if server_exists:
                print("NavigateToPose action server exists. Sending a goal...")

                goal_msg = NavigateToPose.Goal()

                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = self.saved_goal_pose.pose.position.x
                goal_msg.pose.pose.position.y = self.saved_goal_pose.pose.position.y
                goal_msg.pose.pose.position.z = self.saved_goal_pose.pose.position.z

                send_goal_future = action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, send_goal_future)

                if send_goal_future.result() is not None:
                    print('Goal sent successfully!')
                    isrunning = True
                else:
                    print('Failed to send the goal.')
                    isrunning = False
                print("Start test done!")

            else:
                print("NavigateToPose action server does not exist or is not available(start nav).")

        
        if distance_to_robot <= self.threshold_distance:
            time.sleep(1)
            print(self.x)
            print(self.y)
            print(distance_to_robot)
            server_exists = action_client.wait_for_server(timeout_sec=1.0)
            if server_exists:
                print("NavigateToPose action server exists. Stopping a goal...")

                self.safety_run = True
                if action_client.wait_for_server(timeout_sec=5.0):
                    if self.saved_goal_pose != None
                        future = action_client._cancel_goal_async(saved_goal_pose)
                    elif goal_msg != None
                        future = action_client._cancel_goal_async(goal_msg)
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None:
                        print('Cancel all goals request sent successfully')
                        isrunning = False
                    else:
                        print('Failed to send cancel all goals request')
                else:
                    print('Action server not available')
                print("Stop test done!")
            else:
                print("NavigateToPose action server does not exist or is not available (stop nav).")


def main(args=None):
    rclpy.init(args=args)
    SF = SafetyNode()
    rate = SF.node.create_rate(100)

    while rclpy.ok():
        SF.safety_controll()
        rclpy.spin_once(SF)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

