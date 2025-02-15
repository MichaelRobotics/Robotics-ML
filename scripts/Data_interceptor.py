import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
import pandas as pd
import re

# Set time limit for test
TIME_LIMIT = 50

class CallVelOrPos:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

class DataSubscriber(Node):

    def __init__(self):
        super().__init__('Data_subscriber')


        # SUBSCRIBERS
            # SUB 1 - /bool_topic
        self.subscription_one = self.create_subscription(
            String,
            '/bool_topic',
            self.callback_one,
            10)
            # SUB 2 - /cmd_vel
        self.subscription_two = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_two,
            10)
            # SUB 3 - /cmd_vel_nav
        self.subscription_three = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.callback_three,
            10)
        self.subscription_four = self.create_subscription(
            PointStamped,
            '/closest_point',
            self.callback_four,
            10
        )
        
        self.cmd_message = None

        #DATAFRAME DEFINITION

        self.time = 0

        self.df_cmd_vel = pd.DataFrame(columns=['value', 'time'])
        self.df_cmd_vel_nav = pd.DataFrame(columns=['value', 'time'])
        self.df_closest_point = pd.DataFrame(columns=['value', 'time'])
        self.df_bool_topic = pd.DataFrame(columns=['value', 'time'])

        # TIMERS
            # TIM 1
        self.timer_one = self.create_timer(1.0, self.timer_one_callback)
            # TIM 2
        self.timer_two = self.create_timer(1.0, self.timer_two_callback)
            # TIM 3
        self.timer_three = self.create_timer(1.0, self.timer_three_callback)
            # TIM 4
        self.timer_four = self.create_timer(1.0, self.timer_four_callback)


        # CALLBACK ATRIBUTE DEFINITIONS
            #CALL_ATR 1
        self.callback_one_msg = None
            #CALL_ATR 2
        self.callback_two_msg = CallVelOrPos()
            #CALL_ATR 3
        self.callback_three_msg = CallVelOrPos()
            #CALL_ATR 4
        self.callback_four_msg = CallVelOrPos()


        # CALLBACK FUNTIONS
            #CALL_FUN 1
    def callback_one(self, msg: String):
        self.callback_one_msg = msg
            #CALL_FUN 2
    def callback_two(self, msg: Twist):
        self.callback_two_msg.x = msg.linear.x
        self.callback_two_msg.y = msg.angular.z
            #CALL_FUN 3
    def callback_three(self, msg: Twist):
        self.callback_three_msg.x = msg.linear.x
        self.callback_three_msg.y = msg.angular.z 
            #CALL_FUN 4
    def callback_four(self, msg: PointStamped):
        self.callback_four_msg.x = msg.point.x
        self.callback_four_msg.y = msg.point.y


        # CALLBACK INTERCEPTORS WITH TIMERS
            #CALL_ITER 1
    def timer_one_callback(self):
        self.time += 1
        message = str(self.callback_one_msg)
        match = re.search(r"'(.*?)'", message)
        if match:
            result = match.group(1)
        else:
            result = None
        self.callback_one_msg_timer = result
        print(result)
        print(type(result))

        if result == "False" or result == None:
            message = "0"
            print(message)
        else:
            message = "1"
            print(message)
        self.df_bool_topic = self.dataframe_merge(message, self.df_bool_topic)
        self.df_bool_topic.to_csv('SafetyInfo.csv', index=False)
        print(self.callback_one_msg_timer)
            #CALL_ITER 2
    def timer_two_callback(self):
        if ((self.callback_two_msg.x != 0 or self.callback_two_msg.y != 0) and self.cmd_message == "0"):
            message = "1"
            print(message)
        else:
            message = "0"
            print(message)
        self.df_cmd_vel = self.dataframe_merge(message, self.df_cmd_vel)
        self.df_cmd_vel.to_csv('CmdVel.csv', index=False)
            #CALL_ITER 3
    def timer_three_callback(self):
        if self.callback_three_msg.x != 0 or self.callback_three_msg.y != 0:
            self.cmd_message = "1"
            print(self.cmd_message)
        else:
            self.cmd_message = "0"
            print(self.cmd_message)
        self.df_cmd_vel_nav = self.dataframe_merge(self.cmd_message, self.df_cmd_vel_nav)
        self.df_cmd_vel_nav.to_csv('NavVel.csv', index=False)
            #CALL_ITER 4
    def timer_four_callback(self):
        distance_to_robot = round((self.callback_four_msg.x ** 2 + self.callback_four_msg.y ** 2) ** 0.5,4)
        self.df_closest_point = self.dataframe_merge(distance_to_robot, self.df_closest_point)
        self.df_closest_point.to_csv('Distance.csv', index=False)
        print(distance_to_robot)
        print(self.time)
        print(self.df_closest_point)

    def dataframe_merge(self, value, dataframe):
            data = {'value': [value],
                    'time': [self.time]}
            buff_dataframe = pd.DataFrame(data)
            concatenated_df = pd.concat([dataframe, buff_dataframe], ignore_index=True)
            return concatenated_df

def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    while rclpy.ok():
        rclpy.spin_once(data_subscriber)
        if data_subscriber.time == TIME_LIMIT + 1:
            break
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    