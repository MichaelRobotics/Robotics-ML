import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('cancel_navto_pose')
    cancel_navto_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    if cancel_navto_client.wait_for_server(timeout_sec=5.0):
        future = cancel_navto_client._cancel_goal_async()
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print('Cancel all goals request sent successfully')
        else:
            print('Failed to send cancel all goals request')
    else:
        print('Action server not available')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
