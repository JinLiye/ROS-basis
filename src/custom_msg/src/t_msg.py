#! /usr/bin/env python
import rospy
from custom_msg.msg import robot_state

def pub_state():
    rospy.init_node('robot_state_publisher')
    pub = rospy.Publisher('robot_state',robot_state,queue_size=10)
    rate = rospy.Rate(1)

    pos=[0,0]
    speed = 0.5

    while not rospy.is_shutdown():
        msg = robot_state()
        msg.id = 0
        msg.name = 'robot'
        msg.position = pos
        msg.speed = speed
        pub.publish(msg)

        pos[0] += 0.5
        rate.sleep()

if __name__ == '__main__':
    pub_state()
