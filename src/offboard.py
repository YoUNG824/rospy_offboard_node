#!/usr/bin/env python

#Switch to offboard in gazebo and take off to a certain height
#environment: ubuntu18.04, ros melodic python3.6.6 gazebo9
# author: xueyang wang 28.10.2019

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget  # State类型消息储存飞机的连接状态和电机的运行状态
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

class PX4:

    def __init__(self):
        self.local_pos = None
        self.current_state = None
        self.velocity = None
        self.attitude_setpoint = None
        # self.current_heading = None
        self.tookoff_check_flag = False

        self.takeoff_pos_x = 1
        self.takeoff_pos_y = 2
        self.takeoff_pos_z = 3

        self.takeoff_vel_x = 1
        self.takeoff_vel_y = 2
        self.takeoff_vel_z = 3

        # self.body_rate_x = 1
        # self.body_rate_y = 2
        # self.body_rate_z = 4
        self.takeoff_thrust = 0.6

        self.offboard_state = False
        self.arming_state = False

        '''
        ros subscribers
        '''
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.current_state_cb)  # 第一个参数是订阅的话题名，
                                                                                         # 第二个参数是初始化订阅者以订阅State类型的消息，
                                                                                         # 第三个参数是回调函数，用于处理接收到的消息
        self.local_velocity_sub = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.local_velocity_cb)
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pos_cb)

        self.attitude_sub = rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self.local_attitude_setpoint_cb)


        '''
        ros publishers
        '''
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_raw/local",  # 创建话题
                                             PositionTarget,  # 第一个参数是话题名
                                             queue_size=10)   # 第二个参数是消息类型
                                                              # 第三个参数是保存收到的消息的队列的大小
        self.local_attitude_pub = rospy.Publisher("mavros/setpoint_raw/attitude",
                                                  AttitudeTarget,
                                                  queue_size=10)
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
        
    def local_attitude_setpoint_cb(self, msg):
        self.attitude_setpoint = msg

    def local_velocity_cb(self, msg):
        self.velocity = msg
    
    def set_target_pos(self, x, y, z):
        # 事先设定移动的目标位置坐标，若不提前发布目标点坐标，则拒绝切换至“OFFBOARD”模式
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        return target_raw_pose

    def set_target_vel(self, x, y, z):
        # 事先设定移动的目标位置坐标，若不提前发布目标点坐标，则拒绝切换至“OFFBOARD”模式
        target_raw_vel = PositionTarget()
        target_raw_vel.header.stamp = rospy.Time.now()

        target_raw_vel.velocity.x = x
        target_raw_vel.velocity.y = y
        target_raw_vel.velocity.z = z

        return target_raw_vel

    # def set_target_att(self, body_rate_x, body_rate_y, body_rate_z, thrust):
    #     # 事先设定移动的目标位置坐标，若不提前发布目标点坐标，则拒绝切换至“OFFBOARD”模式
    #     target_raw_att = AttitudeTarget()
    #     target_raw_att.header.stamp = rospy.Time.now()
    #
    #     target_raw_att.thrust = thrust
    #     target_raw_att.body_rate.x = body_rate_x
    #     target_raw_att.body_rate.y = body_rate_y
    #     target_raw_att.body_rate.z = body_rate_z
    #
    #     return target_raw_att

    def set_target_att(self, thrust):
        # 事先设定移动的目标位置坐标，若不提前发布目标点坐标，则拒绝切换至“OFFBOARD”模式
        target_raw_att = AttitudeTarget()
        target_raw_att.header.stamp = rospy.Time.now()

        target_raw_att.thrust = thrust

        return target_raw_att

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
            #self.local_pos_pub.publish(self.set_target_pos(self.takeoff_pos_x, self.takeoff_pos_y, self.takeoff_pos_z))
            #self.local_attitude_pub.publish(self.set_target_att(self.body_rate_x, self.body_rate_y, self.body_rate_z, self.takeoff_thrust))
            self.local_attitude_pub.publish(self.set_target_att(self.takeoff_thrust))
            #self.local_pos_pub.publish(self.set_target_vel(self.takeoff_vel_x, self.takeoff_vel_y, self.takeoff_vel_z))
            r.sleep()

            if rospy.is_shutdown():  # 用于检测程序是否退出，是否按Ctrl-C或其他
                break

        last_request1 = rospy.Time.now()
        last_request2 = rospy.Time.now()

        while (not rospy.is_shutdown()) and ((not self.arming_state) or (not self.offboard_state)):
            self.arming_state = self.arming_state_check()
            self.offboard_state = self.offboard_state_check()

        while not rospy.is_shutdown() and self.arming_state and self.offboard_state:
            #self.local_pos_pub.publish(self.set_target_pos(self.takeoff_pos_x, self.takeoff_pos_y, self.takeoff_pos_z))
            #self.local_attitude_pub.publish(self.set_target_att(self.body_rate_x, self.body_rate_y, self.body_rate_z, self.takeoff_thrust))
            self.local_attitude_pub.publish(self.set_target_att(self.takeoff_thrust))
            #self.local_pos_pub.publish(self.set_target_vel(self.takeoff_vel_x, self.takeoff_vel_y, self.takeoff_vel_z))

            if not self.tookoff_check_flag:
                if self.tookoff_check():
                    print("Vehicle Took Off!")
                    self.tookoff_check_flag = True
                else:
                    print("Vehicle Took Off Failed!")

            if rospy.Time.now() - last_request1 > rospy.Duration(3):
                print(self.local_pos.pose.position)
                print(self.attitude_setpoint.thrust)
                #print(self.velocity)
                print("********************")
                last_request1 = rospy.Time.now()


            if rospy.Time.now() -last_request2 > rospy.Duration(0.05):
                if self.local_pos.pose.position.z >= 30 and self.takeoff_thrust>=0.005:
                    self.takeoff_thrust -= 0.005

                if self.local_pos.pose.position.z <= 10 and self.takeoff_thrust<=0.605:
                    self.takeoff_thrust +=0.005
            # if (self.state is "LAND") and (self.local_pos.pose.position.z < 0.15):
            #     if self.disarming():
            #         self.state = "DISARMED"
                last_request2 = rospy.Time.now()

            r.sleep()


if __name__ == "__main__":
    try:
        offb = PX4()
        offb.start()
    except rospy.ROSInterruptException:  # 操作中断的错误异常，经常在rospy.sleep()and rospy.Rate 中用到
        pass
