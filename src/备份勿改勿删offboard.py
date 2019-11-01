#!/usr/bin/env python

#Switch to offboard in gazebo and take off to a certain height
#environment: ubuntu18.04, ros melodic python3.6.6 gazebo9
# author: xueyang wang 28.10.2019

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget  # State类型消息储存飞机的连接状态和电机的运行状态
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

current_state = State()
current_pose = PoseStamped()


class PX4:

    def __init__(self):
        self.local_pos = None
        self.current_state = None
        # self.current_heading = None
        self.tookoff_check_flag = False

        self.takeoff_x = -1
        self.takeoff_y = -2
        self.takeoff_z = 3

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

        print("Controller Initialized!")

    def current_state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, msg):
        self.local_pos = msg

    def set_target(self, x, y, z):
        # 事先设定移动的目标位置坐标，若不提前发布目标点坐标，则拒绝切换至“OFFBOARD”模式
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        return target_raw_pose

    def arming_state_check(self):
        if self.arming_client(True):
            print("Arming Succeed!")
            return True
        else:
            print("Arming Failed!")
            return False

    def offboard_state_check(self):
        if self.set_mode_client(custom_mode='OFFBOARD'):
            print("Offboard Succeed!")
            return True
        else:
            print("Offboard Failed")
            return False

    def tookoff_check(self):
        if self.local_pos.pose.position.z > 0.3 and self.offboard_state and self.arming_state:
            return True
        else:
            print("Offboard failed")
            return False

    def disarming(self):
        if self.arming_client(False):
            return True
        else:
            print("DisArming Failed!")
            return False

    def start(self):
        rospy.init_node("offb_node")  # 初始化ROS节点名为“offb_node”
        r = rospy.Rate(20)  # “OFFBOARD” 命令的超时时间为500毫秒，因此必须以2Hz或者更高的频率发送命令，此处以20Hz的频率发送

        # 连接到FCU之前等待，连接到FCU时current_state.connected变成true
        while not rospy.is_shutdown() and not self.current_state.connected:
            r.sleep()  # 通过睡眠度过循环中剩下的时间来达到设定的20Hz的频率
            # 如果能够达到设定的频率则返回True，否则返回False
        print("FCU Connected!")

        for i in range(20):
            self.local_pos_pub.publish(self.set_target(self.takeoff_x, self.takeoff_y, self.takeoff_z))
            r.sleep()

            if rospy.is_shutdown():  # 用于检测程序是否退出，是否按Ctrl-C或其他
                break

        last_request = rospy.Time.now()

        while not rospy.is_shutdown() and (not self.arming_state or not self.offboard_state):
            self.arming_state = self.arming_state_check()
            self.offboard_state = self.offboard_state_check()

        while not rospy.is_shutdown() and self.arming_state and self.offboard_state:
            self.local_pos_pub.publish(self.set_target(self.takeoff_x, self.takeoff_y, self.takeoff_z))

            if not self.tookoff_check_flag:
                if self.tookoff_check():
                    print("Vehicle Took Off!")
                    self.tookoff_check_flag = True
                else:
                    print("Vehicle Took Off Failed!")

            if rospy.Time.now() - last_request > rospy.Duration(6):
                print(self.local_pos.pose.position)
                print("********************")

                last_request = rospy.Time.now()

            if (self.current_state is "LAND") and (self.local_pos.pose.position.z < 0.3):
                if self.disarming():
                    self.current_state = "DISARMED"

            r.sleep()


if __name__ == "__main__":
    try:
        offb = PX4()
        offb.start()
    except rospy.ROSInterruptException:  # 操作中断的错误异常，经常在rospy.sleep()and rospy.Rate 中用到
        pass
