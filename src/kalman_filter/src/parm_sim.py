#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

# 发布汽车的速度，位置，加速度
class Car:
    def __init__(self) -> None:
        self.p = 0
        self.v = 0

    def step(self,a,dt,p_std_dev,v_std_dev):
        self.p += self.v*dt + 0.5*a*dt**2
        self.v += a*dt

        self.p += np.random.normal(0,p_std_dev)
        self.v += np.random.normal(0,v_std_dev)

def main():
    rospy.init_node("parm_sim")

    pub1 = rospy.Publisher("position",Float64,queue_size=10)
    pub2 = rospy.Publisher("velocity",Float64,queue_size=10)
    pub3 = rospy.Publisher("acceleration",Float64,queue_size=10)

    rate = rospy.Rate(10)
    # filepath = f'../data/accelerations.npy'
    
    filepath = f'/home/qrobo/learn_ws/src/kalman_filter/data/accelerations.npy'
    load_data = np.load(filepath)
    i = 0
    dt = 0.1
    p_std_dev = 0.2
    v_std_dev = 0.3
    c = Car()

    while (not rospy.is_shutdown) and i < 200 :
        acc = load_data[i]
        i += 1
        c.step(acc,dt,p_std_dev,v_std_dev)
        pub1.publish(c.p)
        pub2.publish(c.v)
        pub3.publish(acc)
        rospy.loginfo(f'position:{c.p} a:{acc}')
        rate.sleep()

if __name__ == '__main__':
    main()


        


