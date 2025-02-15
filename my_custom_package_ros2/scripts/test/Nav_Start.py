import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def send_goal_if_server_exists():
    rclpy.init()
    node = rclpy.create_node('send_goal_to_action_server')
    action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    server_exists = action_client.wait_for_server(timeout_sec=1.0)

    if server_exists:
        print("NavigateToPose action server exists. Sending a goal...")

        goal_msg = NavigateToPose.Goal()
        # Set your goal parameters here
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 1.5
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future)

        if send_goal_future.result() is not None:
            print('Goal sent successfully!')
        else:
            print('Failed to send the goal.')

    else:
        print("NavigateToPose action server does not exist or is not available.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    send_goal_if_server_exists()
