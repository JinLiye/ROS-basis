#! /usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker')

    # 创建topic
    pub = rospy.Publisher('test_msg',String,queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = f'Hellp,From talker {rospy.get_time}'
        rospy.loginfo(msg)
        # 发布topic
        pub.publish(msg)

        rate.sleep()

    rospy.loginfo('Talker exist.')

if __name__ == '__main__':
    talker()