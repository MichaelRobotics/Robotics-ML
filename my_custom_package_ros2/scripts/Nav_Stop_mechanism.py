import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time
from nav2_msgs.action import NavigateToPose

class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        # closest point parameters
        self.x = 0
        self.y = 0
        self.callback_active = False
        self.callback_iter = 0

        self.goal_pose_subscription= self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Topic for goal poses published by Rviz2
            self.goal_pose_callback,
            10  # QoS profile history depth
        )
        self.goal_pose_subscription

        # Create a subscriber to the "/closest_point" topic with a callback function
        self.closest_point_subscription = self.create_subscription(
            PointStamped,
            '/closest_point',
            self.closest_point_callback,
            10
        )

        self.closest_point_subscription
        iteration_count = 0
        while iteration_count < 4 and rclpy.ok():
            try:
                print("All TF Frames:")
            except Exception as e:
                print("Exception: {}".format(e))
            rclpy.spin_once(self)
            iteration_count += 1

        # Threshold distances
        self.threshold_distance = 0.5

        self.saved_goal_pose = None

        self.arg_start = False

        self.goal_msg = None

        self.safety = False

        self.i = 0

        self.goal_handle = None

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.first_goal_msg = None

        self.publisher_bool = self.create_publisher(String, '/bool_topic', 10)
        
        self.timer_period = 0.05  # seconds

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        if self.safety == False:
            msg.data = 'False'
            print(msg)
            self.publisher_bool.publish(msg)
        elif self.safety == True:
            msg.data = 'True'
            self.publisher_bool.publish(msg)
        print(msg)



    def closest_point_callback(self, msg: PointStamped):
        # Extract coordinates from the received message
        self.x = msg.point.x
        self.y = msg.point.y
        self.callback_iter = self.callback_iter + 1
        if self.callback_iter == 300:
            self.callback_active = True
#        print(f'msg callback x{msg.point.x}')
#        print(f'msg callback y{msg.point.y}')


    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal pose: {msg}")

        # Save the goal pose clicked by the user
        self.saved_goal_pose = msg
##        print("XD")

    def send_goal(self):

        self.action_client.wait_for_server()

        self.first_goal_msg = NavigateToPose.Goal()  # Replace with your actual goal message
        self.first_goal_msg.pose.header.frame_id = 'map'  # Assuming the goal is given in the map frame
        self.first_goal_msg.pose.pose.position.x = 5.0  # Set the x coordinate
        self.first_goal_msg.pose.pose.position.y = 2.0  # Set the y coordinate


        send_goal_future = self.action_client.send_goal_async(self.first_goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        if send_goal_future.result() is not None:
            print('Goal sent successfully!')
        else:
            print('Failed to send the goal.')

    def safety_controll(self):

        server_exists = self.action_client.wait_for_server(timeout_sec=0.5)

        stop_war = None

        distance_to_robot = (self.x ** 2 + self.y ** 2) ** 0.5

        if server_exists:
#            print("NavigateToPose action server exists. Safety can operate")

            if distance_to_robot <= self.threshold_distance:
                stop_war = True
            elif distance_to_robot > self.threshold_distance:
                stop_war = False
            else:
                raise(f"Error, stop_war is {stop_war}")
            print(stop_war)

            if self.safety == True and stop_war == True:
                pass

            elif self.safety == False and stop_war == True:
                print("STOPED_GOAL")
                self.goal_msg = NavigateToPose.Goal()
                self.goal_msg.pose.header.frame_id = 'map'
                self.goal_msg.pose.pose.position.x = 5.0
                self.goal_msg.pose.pose.position.y = 2.0    

#                future = action_client._cancel_goal_async()
                future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    print('Cancel all goals request sent successfully')
                else:
                    print('Failed to send cancel all goals request')


                self.safety = True


            elif self.safety == False and stop_war == False:
                pass


            elif self.safety == True and stop_war == False:
                print("SENT_GOAL")
                time.sleep(3)
                if self.goal_msg == None:
                    raise("Error when start, goal_msg is None, when it should be set! ")
                send_goal_future = self.action_client.send_goal_async(self.goal_msg)
                rclpy.spin_until_future_complete(self, send_goal_future)
                self.goal_handle = send_goal_future.result()
                if send_goal_future.result() is not None:
                    print('Goal sent successfully!')
                else:
                    print('Failed to send the goal.')

                self.safety = False
            else:
                print("Upredictable things happened!!!!!!!! when working with safety")
        else:
            print("NavigateToPose action server does not exist or is not available, safety cannot operate.")



#        if self.safety_run == True and distance_to_robot > self.threshold_distance and isrunning == False:
#            
#            server_exists = action_client.wait_for_server(timeout_sec=1.0)
#
#            if server_exists:
#                print("NavigateToPose action server exists. Sending a goal...")
#
#                self.goal_msg = NavigateToPose.Goal()
#
#                self.goal_msg.pose.header.frame_id = 'map'
#                self.goal_msg.pose.pose.position.x = self.saved_goal_pose.pose.position.x
#                self.goal_msg.pose.pose.position.y = self.saved_goal_pose.pose.position.y
#                self.goal_msg.pose.pose.position.z = self.saved_goal_pose.pose.poSF.saved_goal_pose.pose.position.xsition.z
#
#                send_goal_future = action_client.send_goal_async(self.goal_msg)
#                rclpy.spin_until_future_complete(self, send_goal_future)
#
#                if send_goal_future.result() is not None:
#                    print('Goal sent successfully!')
#                    self.isrunning = True
#                else:
#                    print('Failed to send the goal.')
#                    self.isrunning = False
#                print("Start test done!")
#
#            else:
#                print("NavigateToPose action server does not exist or is not available(start nav).")
#
#        print(self.isrunning)
#        print(f"threshold {self.threshold_distance}")
#        print(f"distance {distance_to_robot}")
#
#        if distance_to_robot <= self.threshold_distance and self.x != 0 and self.isrunning == True:
#            time.sleep(1)
##            print(self.x)
##            print(self.y)
#            print(distance_to_robot)
#            server_exists = action_client.wait_for_server(timeout_sec=1.0)
#            if server_exists:
#                print("NavigateToPose action server exists. Stopping a goal...")
#
#                self.safety_run = True
#                if action_client.wait_for_server(timeout_sec=5.0):
#                    if self.saved_goal_pose != None:
#                        future = action_client._cancel_goal_async(self.saved_goal_pose)
#                        rclpy.spin_until_future_complete(self, future)
#                    elif self.goal_msg != None:
#                        future = action_client._cancel_goal_async(self.goal_msg)
#                        rclpy.spin_until_future_complete(self, future)
#                    else:
#                        pass
##                    if future.result() is not None:
##                        print('Cancel all goals request sent successfully')
##                        self.isrunning = False
##                    else:
##                        print('Failed to send cancel all goals request')
#                else:
#                    print('Action server not available')
#                print("Stop test done!")
#            else:
#                print("NavigateToPose action server does not exist or is not available (stop nav).")


def main(args=None):
    rclpy.init(args=args)
    SF = SafetyNode()
    rate = SF.create_rate(10)
    run_once = False
    while rclpy.ok():
        if SF.callback_active == True:
            if run_once == False:
                SF.send_goal()
                run_once = True 
            SF.arg_start = True
        else:
            pass

        if SF.arg_start == True:
 #           timer = SF.create_timer(SF.timer_period, SF.timer_callback)
            SF.safety_controll()
            print("safety_work")
        else:
            pass
        rclpy.spin_once(SF)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

