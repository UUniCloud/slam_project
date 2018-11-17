#!usr/bin/env python
# coding:UTF-8
#agv-motion-model

import math
import rospy

'''
theta:agv转向角(定义为逆时针为正）
l:agv轴距
r:瞬态回转半径
d:两从动轮轮距的1/2
v:AGV行驶的瞬态线速度
w:AGV行驶的瞬态角速度
点P：前后轮轴线的交点，AGV的瞬态回转中心
Vagv：AGV中心轴线所在方向的线速度。
该模型以agv的后两轮为车体位置的参考点，计算Odometry里程计消息时会用到。
l=0
d=0
theta=0

V_agv=v * cos(theta)
if theta>0:
    
    r = l / tan(theta)
else:
    r = - l/ tan(theta)

w=V_agv /r
其中v 和 theta 需要计算求得
v是通过驱动编码器返回的值计算出的AGV的瞬态线速度(即舵轮的线速度)。
theta是通过转向编码器返回的值计算出的AGV转向角度。
delta_t是车载系统主控制器的一个极小的程序运行周期。
即只要知道AGV启动时的位置和姿态坐标点的初值，通过迭代法，便可以求出任意时刻AGV所在位置的坐标。

#需要的已知量是v返回的线速度，alpha初始方位角，和delta_t。
'''
x_=0 #初始位置
y_=0
alpha_=0 #初始方位角

l=102 #主舵轮中心到两从动轮连线中心的距离cm
d=7.5 #两从动轮连线距离的一半cm
v_agv =v * cos(alpha_) #agv中轴线的方向的线速度
if alpha_ > 0:
    
    r = l / tan(alpha_)
else:
    r = - l/ tan(alpha_)

w_ = v_agv / r
curr_time = rospy.time.now()
delta_t = (curr_time - last_time).to_sec()

x = x_ - r * sin(alpha_) + r * sin(alpha_ + w_ * delta_t)
y = y_ + r * cos(alpha_) - r * cos(alpha_ + w_ * delta_t)
alpha = alpha_ + w_ * delta_t
last_time = curr_time
#从而计算的agv的里程计数据为(v_agv,w_,alpha)
