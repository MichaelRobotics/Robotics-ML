import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time
from nav2_msgs.action import NavigateToPose

NAVIGATION_X_POSITION = 3.0
NAVIGATION_Y_POSITION = 4.0
TRESHOLD_DISTANCE = 1.0


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        # Parametry /closest_point parameters

        self.x = 0
        self.y = 0
        self.callback_active = False
        self.callback_iter = 0
        

        self.closest_point_subscription = self.create_subscription(
            PointStamped,
            '/closest_point',
            self.closest_point_callback,
            10
        )


        # Atrybuty obiektu 

        self.threshold_distance = TRESHOLD_DISTANCE

        self.arg_start = False

        self.goal_msg = None

        self.safety = False

        self.goal_handle = None

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.first_goal_msg = None



    def closest_point_callback(self, msg: PointStamped):

        # Pobranie informacji o polozeniu punktu publikowanego na /closest_point
        self.x = msg.point.x
        self.y = msg.point.y

        # Wykonanie w petli dodawania do 300, opoznienie rozruchu funckji safety, az atrybutry funkcji callback beda dostepne dla funkcji safety
        self.callback_iter = self.callback_iter + 1
        if self.callback_iter == 300:
            self.callback_active = True

    def send_goal(self):
        """
        Funkcja odpowiedzialna za przeslanie pierwszego zlecenia jazdy do celu.
        Konieczne do prawidlowego funkcjonowania systemu safety
        """

        self.action_client.wait_for_server()
        self.first_goal_msg = NavigateToPose.Goal()  
        self.first_goal_msg.pose.header.frame_id = 'map'  
        self.first_goal_msg.pose.pose.position.x = NAVIGATION_X_POSITION  
        self.first_goal_msg.pose.pose.position.y = NAVIGATION_Y_POSITION  

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

        if server_exists:

            distance_to_robot = (self.x ** 2 + self.y ** 2) ** 0.5

            if distance_to_robot <= self.threshold_distance:
                stop_war = True
            elif distance_to_robot > self.threshold_distance:
                stop_war = False
            else:
                raise(f"Error, stop_war is {stop_war}")
            print(stop_war)

            if self.safety == True and stop_war == True:
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

            elif self.safety == False and stop_war == True:
                print("STOPPED_GOAL")
                self.goal_msg = NavigateToPose.Goal()
                self.goal_msg.pose.header.frame_id = 'map'
                self.goal_msg.pose.pose.position.x = NAVIGATION_X_POSITION
                self.goal_msg.pose.pose.position.y = NAVIGATION_Y_POSITION

                future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    print('Cancel all goals request sent successfully')
                else:
                    print('Failed to send cancel all goals request')


                self.safety = True


            elif self.safety == False and stop_war == False:
                pass

                self.safety = False
            else:
                print("Upredictable things happened!!!!!!!! when working with safety")
        else:
            print("NavigateToPose action server does not exist or is not available, safety cannot operate.")




def main(args=None):
    rclpy.init(args=args)
    SF = SafetyNode()
    rate = SF.create_rate(10)
    run_once = False
    run_first_goal = False
    while rclpy.ok():
        if SF.callback_active == True and run_first_goal == False:
            if run_once == False:
                SF.send_goal()
                run_once = True 
            SF.arg_start = True
            run_first_goal = True
        else:
            pass

        if SF.arg_start == True:
            SF.safety_controll()
            print("safety works!")
        else:
            pass
        rclpy.spin_once(SF)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

