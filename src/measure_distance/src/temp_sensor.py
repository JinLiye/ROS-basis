#! /usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float64

def measure_temp(data,params):
    std_dev = params[0]
    pub:rospy.Publisher = params[1]

    # 添加均值为0，标准差为std_dev的高斯噪声
    measure = data.data + np.random.normal(0,std_dev)
    pub.publish(measure)

def main():
    rospy.init_node('temp_sensor')
    # 获取传感器标准差
    std_dev = rospy.get_param('/std_dev')

    pub = rospy.Publisher('temp_sensor',Float64,queue_size=10)

    # 订阅
    rospy.Subscriber('distance_temp',Float64,measure_temp,(std_dev,pub))
    rospy.spin()

if __name__ == '__main__':
    main()