#! /usr/bin/env python
import rospy
from learn_service.srv import AddTwoNum,AddTwoNumRequest,AddTwoNumResponse

def handle_add_two_num(req:AddTwoNumRequest):
    sum = req.A + req.B
    rospy.loginfo(f'{req.A}+{req.B}={sum}')
    return AddTwoNumResponse(sum)

if __name__ == '__main__':
    rospy.init_node('add_two_num_server')

    rospy.Service('add_two_num',AddTwoNum,handler=handle_add_two_num)
    rospy.loginfo('service ready')
    rospy.spin()