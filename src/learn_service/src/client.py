#! /usr/bin/env python
import rospy
from learn_service.srv import *

def add_two_num_client(a,b):
    rospy.loginfo('waitting')
    rospy.wait_for_service('add_two_num')

    try:
        service = rospy.ServiceProxy('add_two_num',AddTwoNum)
        res:AddTwoNumResponse = service(a,b)
        return res.Sum
    except rospy.exceptions as e:
        rospy.logerr(f'{e}')

if __name__ == '__main__':
    rospy.init_node('add_two_num_client')
    a = 2
    b = 3
    res = add_two_num_client(a,b)
    rospy.loginfo(f'get response{res}')