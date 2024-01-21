#! /usr/bin/env python
import rospy
import tf
from turtlesim.msg import Pose

def handle_turtle_pose(msg:Pose,turtlename):
    # 定义一个广播器
    br = tf.TransformBroadcaster()

    # 发送一个tf变换
    br.sendTransform((msg.x,msg.y,0),
                     tf.transformations.quaternion_from_euler(0,0,msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")
    
if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    # 从参数中获取turtle名字 
    turtlename = rospy.get_param('~turtle')
    # 订阅topic
    rospy.Subscriber(f'/{turtlename}/pose',Pose,handle_turtle_pose,turtlename)

    rospy.spin()

