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
        pass
