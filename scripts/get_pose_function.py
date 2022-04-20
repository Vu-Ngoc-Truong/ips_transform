#!/usr/bin/env python
# -*- coding: utf-8 -*-

from distutils.errors import DistutilsPlatformError
from turtle import position
import numpy as np
from math import *
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from marvelmind_nav.msg import hedge_pos
import yaml
def yaml_load(filepath):
    with open(filepath, "r") as read_file:
        data = yaml.load(read_file, Loader=yaml.FullLoader)
    return data


def getRTfromQuat(translation, quat):

	# pose_map = np.array([[-0.22571, 0.063645, 0.20089]])
	# orient_map = np.array([ -0.0027935, 0.70555, -0.0043037, 0.70864])
	# print quat
	[roll1,pitch1,yaw1] = euler_from_quaternion(np.asarray(quat))
	# print roll1, pitch1, yaw1
	Rp1= np.matrix([[cos((pitch1)), 0, sin((pitch1))],\
                    [0            , 1,             0],\
                    [-sin(pitch1) , 0, cos((pitch1))]])


	Ry1= np.matrix([[cos((yaw1)) ,-sin((yaw1)) ,0],\
                    [sin((yaw1)), cos((yaw1)), 0],\
                    [0, 0, 1]])

	Rr1= np.matrix([[1 ,                0,         0  ],\
                    [0 ,    cos((roll1)), -sin((roll1))],\
                    [0 ,    sin((roll1)) ,cos((roll1))]])

	R1=Ry1*Rp1*Rr1

	R = np.asarray(R1)
	R = np.vstack((R, np.array([0,0,0])))
	R = np.hstack((R, np.array([[translation.item(0)],[translation.item(1)],[translation.item(2)],[1]])))
	# print(R)
	return R
def getRTfromEuler(translation ,euler):
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	Rp1=np.matrix([[cos((pitch)), 0, sin((pitch))],\
	     		   [0            , 1,             0],\
	     		   [-sin(pitch) , 0, cos((pitch))]])


	Ry1=np.matrix([[cos((yaw)) ,-sin((yaw)) ,0],\
	     [sin((yaw)), cos((yaw)), 0],\
	     [0, 0, 1]])

	Rr1=np.matrix([[1 ,0 ,0],\
	    [0 ,cos((roll)), -sin((roll))],\
	    [0 ,sin((roll)) ,cos((roll))]])

	R1=Ry1*Rp1*Rr1

	R = np.asarray(R1)
	R = np.vstack((R, np.array([0,0,0])))
	R = np.hstack((R, np.array([[translation.item(0)],[translation.item(1)],[translation.item(2)],[1]])))
	# print(R)
	return R
def getQuatfromRT(RT):
	qw = sqrt(1+RT[0,0] + RT[1,1]+ RT[2,2])/2
	qx = (RT[2,1] - RT[1,2])/(4*qw)
	qy = (RT[0,2] - RT[2,0])/(4*qw)
	qz = (RT[1,0] - RT[0,1])/(4*qw)
	return np.array([qx, qy, qz, qw])

# def getPOSE(poseODOM_ar3, orientODOM_ar3):

def get_pose(posIps, oriIps, file_path):
    data = yaml_load(file_path)
    # print(data)
    pos_trans = np.array([[data["position_x"], data["position_y"], 0.0]])
    ori_trans = np.array([data["roll"], data["pitch"], data["yaw"]])

    T_1 = np.asmatrix(getRTfromEuler(pos_trans, ori_trans))
    T_2 = np.asmatrix(getRTfromEuler(posIps, oriIps))
    T = T_1*T_2
    position = T[0:3, -1]  # supress the z coordinate
    # print(position)
    orientation = getQuatfromRT(T)
    # print ("position")
    # print (position)
    # print ("orientation")
    # print (orientation)
    return position

# find center of circle from 3 point:
def angle_from_coordinates(start_point,end_point):
    a = start_point
    b = end_point
    angle = atan2(b[1]-a[1],b[0]-a[0])
    # print("angle rad: {}".format(angle))
    # print("angle deg: {}".format(degrees(angle)))
    return angle

def distance_from_coordinates(start_point,end_point):
    a = start_point
    b = end_point
    distance = sqrt((b[1]-a[1])**2 + (b[0]-a[0])**2)
    # print("angle rad: {}".format(angle))
    # print("angle deg: {}".format(degrees(angle)))
    return distance

def hs_mt1(A):
    return [2*A[0], 2*A[1], -1]

def hs_mt_c(A):
    return  A[0]**2 + A[1]**2

def center_of_three_point(A,B,C):
    """
    arg : start_point, mid_point , end_point
    return: Center of circle, Radius, Yaw angle from Center to end_point with coordinate axis.

    """
    M_hs = np.matrix([hs_mt1(A),hs_mt1(B),hs_mt1(C)])
    # print(M_hs)
    M_c = np.matrix([[hs_mt_c(A)],[hs_mt_c(B)],[hs_mt_c(C)]])
    # print(M_c)
    M_kq = np.linalg.inv(M_hs) * M_c
    R_c = sqrt(M_kq[0,0]**2 + M_kq[1,0]**2 - M_kq[2,0])
    O_c = np.array([M_kq[0,0],M_kq[1,0]])
    yaw = angle_from_coordinates(O_c,C)
    print(M_kq)
    print("Center of Circle: x= {} \t y= {}".format(M_kq[0,0],M_kq[1,0] ))
    print("Radius: ",R_c)
    print("Yaw : ",)

    return O_c, R_c, yaw
