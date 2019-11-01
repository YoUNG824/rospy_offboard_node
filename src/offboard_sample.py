#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State  # State类型消息储存飞机的连接状态和电机的运行状态
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


def offboard_node():
    rospy.init_node("offb_node")  # 初始化ROS节点名为“offb_node”
    r = rospy.Rate(20)  # “OFFBOARD” 命令的超时时间为500毫秒，因此必须以2Hz或者更高的频率发送命令，此处以20Hz的频率发送

    rospy.Subscriber("mavros/state", State, state_cb)  # 第一个参数是订阅的话题名，
                                                       # 第二个参数是初始化订阅者以订阅State类型的消息，
                                                       # 第三个参数是回调函数，用于处理接收到的消息
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",  # 创建话题
                                    PoseStamped,                       # 第一个参数是话题名
                                    queue_size=10)                     # 第二个参数是消息类型
                                                                       # 第三个参数是保存收到的消息的队列的大小

    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)  # 启动电机用的服务客户端的初始化，第二个参数为消息类型
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)    # 更改模式用的服务客户端的初始化，第二个参数为消息类型

    # 连接到FCU之前等待，连接到FCU时current_state.connected变成true
    while not rospy.is_shutdown() and not current_state.connected:
        r.sleep()  # 通过睡眠度过循环中剩下的时间来达到设定的20Hz的频率
                   # 如果能够达到设定的频率则返回True，否则返回False

    # 事先设定移动的目标位置坐标，若不提前发布目标点坐标，则拒绝切换至“OFFBOARD”模式
    pose = PoseStamped()
    pose.pose.position.x = 6
    pose.pose.position.y = 7
    pose.pose.position.z = 8

    for i in range(100):
        local_pos_pub.publish(pose)
        r.sleep()

        if rospy.is_shutdown():  # 用于检测程序是否退出，是否按Ctrl-C或其他
            break

    # 将SetMode消息的custom_mode字段设置为OFFBOARD
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = "OFFBOARD"

    # 生成给/mavros/cmd/arming服务发送的信息
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        # 判断现在的模式是否为OFFBOARD已经请求切换模式过去是否5秒以上
        if current_state.mode != "OFFBOARD" \
                and (rospy.Time.now() - last_request > rospy.Duration(5)):

            #如果服务调用失败，则会引发一场，因此有必要使用try-except捕获异常
            try:
                # 在set_mode_client（offb_set_mode）中，使用offb_set_mode变量的消息来调用mavros/set_mode服务
                # 该服务具有请求字段（用于在进行服务调用时提供必要的信息）和响应字段（用于存储处理结果等），如果服务调用成功，则将返回响应
                offb_set_mode_resp = set_mode_client(offb_set_mode)
                # 通过检查以下来判断切换模式的命令有没有被送达以及被识别
                if offb_set_mode_resp.mode_sent:
                    rospy.loginfo("Offboard enabled")
            except rospy.ServiceException as e:
                rospy.logwarn(e)

            last_request = rospy.Time.now()

        else:
            if not current_state.armed \
                    and (rospy.Time.now() - last_request > rospy.Duration(5)):
                try:
                    arm_cmd_resp = arming_client(arm_cmd)
                    if arm_cmd_resp.success:
                        rospy.loginfo("Vehicle armed")
                except rospy.ServiceException as e:
                    rospy.logwarn(e)

                last_request = rospy.Time.now()

        local_pos_pub.publish(pose)
        r.sleep()


if __name__ == "__main__":
    try:
        offboard_node()
    except rospy.ROSInterruptException:   # 操作中断的错误异常，经常在rospy.sleep()and rospy.Rate 中用到
        pass
