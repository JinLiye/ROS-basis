#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

# 同步信号
gps_value = None
odom_value = None
a_value = None

class kalmanFilter():
    # x_dim 表示状态的维度
    # u_dim 表示控制的维度
    # z_dim 表示测量的维度
    # A为状态转移矩阵
    # B为控制矩阵
    # H为观测矩阵
    # Q为过程噪声矩阵
    # R为测量误差矩阵
    # x0为状态初始值
    # p0为初始协方差
    def __init__(self,x_dim,u_dim,z_dim,A,B,H,Q,R,x0,P0) -> None:
        # 数据检验
        assert A.shape[0] == x_dim and A.shape[1] == x_dim,"A.shape must be x_dim*x_dim"
        assert B.shape[0] == x_dim and B.shape[1] == u_dim, "B.shape must be x_dim * u_dim!"
        assert H.shape[0] == z_dim and H.shape[1] == x_dim, "H.shape must be z_dim * x_dim!"
        assert Q.shape[0] == x_dim and Q.shape[1] == x_dim, "Q.shape must be x_dim * x_dim!"
        assert R.shape[0] == z_dim and R.shape[1] == z_dim, "R.shape must be z_dim * z_dim!"
        assert x0.shape[0] == x_dim, "x0.shape[0] must be x_dim!"
        assert P0.shape[0] == x_dim and P0.shape[1] == x_dim, "P0.shape must be x_dim * x_dim!"

        # 赋值
        self.x_dim = x_dim
        self.u_dim = u_dim
        self.z_dim = z_dim
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R

        # 状态的后验估计
        self.x = x0

        # 状态的先验估计
        self.x_ = None

        # 后验估计的协方差
        self.P = P0

        # 先验估计的协方差
        self.P_ = None

        # 卡尔曼增益
        self.K = None

    # 传入控制量u进行预测
    def predect(self,u=None):
        # 计算先验估计
        self.x_ = self.A @ self.x

        if u is not None:
            assert u.shape[0] == self.u_dim, "u.shape[0] must equals to u_dim"
            self.x_ += self.B @ u
        
        # 计算先验估计协方差
        self.P_ = self.A @ self.P @ self.A.T + self.Q

    # 更新过程，传入测量值z
    def update(self,z):
        assert z.shape[0] == self.z_dim, "z.shape[0] must equals to z_dim"
        # 计算卡尔曼增益
        self.K = (self.P_ @ self.H.T) @ np.linalg.inv(self.H @ self.P_ @ self.H.T + self.R)

        # 更新测量值
        self.x = self.x_ + self.K @ (z - self.H @ self.x_)

        # 更新协方差
        # self.P = (np.identity(self.x_dim) - self.K @ self.H) @ self.P_ @ np.transpose(np.identity(self.x_dim) - self.K @ self.H) + self.K @ self.R @ self.K.T
        self.P = (np.identity(self.x_dim) - self.K @ self.H) @ self.P_
        # 返回滤波器的输出
        return self.x
    
class KalmanFilter2():
    def __init__(self, x_dim, u_dim, z_dim, A, B, H, Q, R, x0, P0) -> None:
        # 数据检验
        assert A.shape[0] == x_dim and A.shape[1] == x_dim, "A.shape must be x_dim * x_dim!"
        assert B.shape[0] == x_dim and B.shape[1] == u_dim, "B.shape must be x_dim * u_dim!"
        assert H.shape[0] == z_dim and H.shape[1] == x_dim, "H.shape must be z_dim * x_dim!"
        assert Q.shape[0] == x_dim and Q.shape[1] == x_dim, "Q.shape must be x_dim * x_dim!"
        assert R.shape[0] == z_dim and R.shape[1] == z_dim, "R.shape must be z_dim * z_dim!"
        assert x0.shape[0] == x_dim, "x0.shape[0] must be x_dim!"
        assert P0.shape[0] == x_dim and P0.shape[1] == x_dim, "P0.shape must be x_dim * x_dim!"

        # 赋值
        self.x_dim = x_dim
        self.u_dim = u_dim
        self.z_dim = z_dim
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R

        # 状态的后验估计
        self.x = x0

        # 状态的先验估计
        self.x_ = None

        # 后验估计的协方差
        self.P = P0

        # 先验估计的协方差
        self.P_ = None

        # 卡尔曼增益
        self.K = None
    
    def predict(self, u=None):
        # 预测，传入控制量 u_k

        # 先验估计
        self.x_ = self.A @ self.x
        if u is not None:
            assert u.shape[0] == self.u_dim, "u.shape[0] must be u_dim!"
            self.x_ += self.B @ u
        
        # 先验估计的协方差
        self.P_ = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        #更新，传入测量值 z_k

        assert z.shape[0] == self.z_dim, "z.shape[0] must be z_dim!"

        # 计算卡尔曼增益
        self.K = (self.P_ @ self.H.T) @ np.linalg.inv(self.H @ self.P_ @ self.H.T + self.R)

        # 后验估计
        self.x = self.x_ + self.K @ (z - self.H @ self.x_)

        # 后验估计的协方差
        self.P = (np.identity(self.x_dim) - self.K @ self.H) @ self.P_
        
        # 返回后验估计，滤波器的输出
        return self.x

    
def get_a(data):
    global a_value
    a_value = data.data

def get_odom(data):
    global odom_value
    odom_value = data.data

def get_gps(data):
    global gps_value
    gps_value = data.data

def main():
    rospy.init_node('filter_node')
    # 定义参数
    dt = rospy.get_param('dt')
    x_dim = 2
    u_dim = 1
    z_dim = 2

    A = np.array([[1, dt], 
                  [0, 1]])
    B = np.array([[0.5 * dt * dt],[dt]])
    H = np.array([[1,0],[0,1]])
    gps_std_dev = rospy.get_param('/gps_std_dev')
    odom_std_dev = rospy.get_param('/odom_std_dev')
    Q = np.array([[gps_std_dev**2,0],[0,odom_std_dev**2]])
    measure_gps_std_dev = rospy.get_param('/measure_gps_std_dev')
    measure_odom_std_dev = rospy.get_param('/measure_odom_std_dev')
    R = np.array([[measure_gps_std_dev**2,0],[0,measure_odom_std_dev**2]])

    x0 = np.array([[0],[0]])
    P0 = np.array([[1,0],[0,1]])
    # 订阅参数
    rospy.Subscriber('p_gps',Float64,get_gps)
    rospy.Subscriber('v_odom',Float64,get_odom)
    rospy.Subscriber('acceleration',Float64,get_a)


    # 初始化滤波类
    kf = kalmanFilter(x_dim,u_dim,z_dim,A,B,H,Q,R,x0,P0)

    # 定义发布者
    pub1 = rospy.Publisher('filter_p',Float64,queue_size=10)
    pub2 = rospy.Publisher('filter_v',Float64,queue_size=10)

    global gps_value, odom_value, a_value

    # 轮询频率大一点，保证同步不丢失
    rate = rospy.Rate(5 / dt)
    i = 0t
    while not rospy.is_shutdown():
        if odom_value is not None and gps_value is not None and a_value is not None:
            # 预测
            # kf.predect(np.array([[a_value]]))
            kf.predect(np.array([[a_value]]))
            # 更新
            x = kf.update(np.array([[gps_value],[odom_value]]))
            i += 1
            # 打印
            rospy.loginfo(f'{i}: status:{kf.x}')
            # 发布topic
            pub1.publish(x[0,0])
            pub2.publish(x[1,0])
            # 等待下一次轮询
            odom_value = None
            gps_value = None
            a_value = None
        rate.sleep()

if __name__ == "__main__":
    main()