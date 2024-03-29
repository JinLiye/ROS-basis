#! /usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('distance')

    pub = rospy.Publisher('distance_temp',Float64,queue_size=10);
    rate = rospy.Rate(10)

    rospy.sleep(2)

    while not rospy.is_shutdown():
        dis = 3.8

        pub.publish(dis)

        rate.sleep()

if __name__=='__main__':
    main()