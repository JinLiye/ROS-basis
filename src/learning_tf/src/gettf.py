#! /usr/bin/env python
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import random

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    # 定义一个tf监听器
    listener = tf.TransformListener()

    # 定义Service造一个海龟
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn',turtlesim.srv.Spawn)
    name = rospy.get_param("~turtle")
    spawner(4,2,0,name)

    # 定义发布海龟的topic
    # turtle_vel = rospy.Publisher('/turtle2/cmd_vel',geometry_msgs.msg.Twist,queue_size=1)
    turtle_vel = rospy.Publisher(f'/{name}/cmd_vel',geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            listener.waitForTransform(f'/{name}','/turtle1',rospy.Time(0),rospy.Duration(1))

            # 查询tf树中/turtle1到/turtle2的变换（turtle1在turtle2下的位姿）
            (trans,rot) = listener.lookupTransform(f'/{name}','/turtle1',rospy.Time(0))
            
        except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException) as e:
            rospy.logwarn(f'error:{e}')
            continue

        # 用相对位置计算角速度，线速度
        roll,pitch,yaw = tf.transformations.euler_from_quaternion(rot)
        desire_dis = rospy.get_param('~desire_dis')
        temp = trans
        temp[0] = trans[0]-desire_dis*math.cos(yaw)
        temp[1] = trans[1]-desire_dis*math.sin(yaw)
        angular = .7*math.atan2(temp[1],temp[0])

        dis = math.sqrt(temp[0] ** 2 + temp[1] ** 2)
        if dis > .05:
            liner = .5*dis
        else:
            liner = 0
            angular = .5*yaw

        # 发布速度控制指令 Topic
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = liner
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()