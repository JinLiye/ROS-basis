#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

def getgps(data):
    rospy.loginfo(f'gps:{data}')

def main():
    rospy.init_node('gps')
    sub = rospy.Subscriber("position",Float64,getgps)
    rospy.spin()

if __name__ == '__main__':
    main()