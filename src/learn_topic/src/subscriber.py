#! /usr/bin/env python
import rospy
from std_msgs.msg import String

def get_test_msg(data):
    rospy.loginfo(f'Listener get msg{data.data}')

def listener():
    rospy.init_node('listener')

    #创建topic订阅者
    rospy.Subscriber('test_msg',String,get_test_msg)

    # 防止程序提前退出
    rospy.spin()

if __name__ == '__main__':
    listener()