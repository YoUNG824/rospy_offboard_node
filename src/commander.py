#!/usr/bin/env python

#environment: ubuntu18.04, ros melodic python3.6.6 gazebo9
# author: xueyang wang 28.10.2019

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
import time
import math

class Commander:
    def __init__(self):
        self.local_pos = None
        self.current_state = None
        # self.current_heading = None
        self.tookoff_check_flag = False

        self.offboard_state = False
        self.arming_state = False

        '''
        ros subscribers
        '''
        self.state_sub = rospy.Subscriber("mavros/state", State, self.current_state_cb)  # 第一个参数是订阅的话题名，
                                                                                         # 第二个参数是初始化订阅者以订阅State类型的消息，
                                                                                         # 第三个参数是回调函数，用于处理接收到的消息
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pos_cb)

        '''
        ros publishers
        '''
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_raw/local",  # 创建话题
                                             PositionTarget,  # 第一个参数是话题名
                                             queue_size=10)   # 第二个参数是消息类型
                                                              # 第三个参数是保存收到的消息的队列的大小
        '''
        ros services
        '''
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)  # 启动电机用的服务客户端的初始化，第二个参数为消息类型
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)  # 更改模式用的服务客户端的初始化，第二个参数为消息类型

        print("Commander Initialized!")

    def move(self, x, y, z):
        self.position_target_pub.publish(self.set_pose(x, y, z))

    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

    # land at current position
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))

    # hover at current position
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))

    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height))

    def set_pose(self, x=0, y=0, z=2):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose


if __name__ == "__main__":
    con = Commander()
    time.sleep(2)
    con.move(1, 0, 0)
    time.sleep(2)
    con.turn(90)
    time.sleep(2)
    con.land()
