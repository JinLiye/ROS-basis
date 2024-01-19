#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import random
from turtlesim.srv import Spawn,SpawnRequest,SpawnResponse
import string

class TurtleGenerator:

    tw = Twist()
    name = "newTs"

    def __init__(self):
        rospy.init_node('turtle_generator', anonymous=True)
        self.name = self.generate_random_string(10)
        self.tw.linear.x = random.uniform(0.1,1.5)
        self.tw.angular.z = random.uniform(-2,2)

        self.turtle_cmd_pub = rospy.Publisher('/' + self.name+ '/cmd_vel', Twist, queue_size=10)

    def generate_random_string(self,length):
        letters = string.ascii_letters  # 包含大小写字母的字符串
        return ''.join(random.choice(letters) for _ in range(length))

    def generate_turtle(self):
        rate = rospy.Rate(1) 

        # 生成出生点随机数
        x = random.randint(1,10)
        y = random.randint(1,10)
        
        rospy.wait_for_service('/spawn')
        try:
            spawn_turtle_proxy = rospy.ServiceProxy('/spawn', Spawn)

            spawn_turtle_proxy(x, y, 0, self.name)

        except rospy.ServiceException as e:
            print("Service call failed:%s" % e)

        while not rospy.is_shutdown():
            self.turtle_cmd_pub.publish(self.tw)
            rate.sleep()

if __name__ == '__main__':
    try:
        turtle_gen = TurtleGenerator()
        turtle_gen.generate_turtle()
    except rospy.ROSInterruptException:
        pass
