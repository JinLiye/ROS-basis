#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

def getgps(data,params):
    # rospy.loginfo(f'gps:{data}')
    dev = params[0]
    measure = data.data + np.random.normal(dev)

    pub= params[1]
    pub.publish(measure)

def main():
    rospy.init_node('gps')
    # 获取gps标准差
    std_dev = rospy.get_param('/gps_std_dev')

    pub = rospy.Publisher("p_gps",Float64,queue_size=10)

    rospy.Subscriber("position",Float64,getgps,(std_dev,pub))
    rospy.spin()

if __name__ == '__main__':
    main()