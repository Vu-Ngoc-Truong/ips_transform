#!/usr/bin/env python

import numpy as np
from math import cos, sin, pi, sqrt
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
	Rp1=np.matrix([[cos((pitch1)), 0, sin((pitch1))],\
	     		   [0            , 1,             0],\
	     		   [-sin(pitch1) , 0, cos((pitch1))]])


	Ry1=np.matrix([[cos((yaw1)) ,-sin((yaw1)) ,0],\
	     [sin((yaw1)), cos((yaw1)), 0],\
	     [0, 0, 1]])

	Rr1=np.matrix([[1 ,0 ,0],\
	    [0 ,cos((roll1)), -sin((roll1))],\
	    [0 ,sin((roll1)) ,cos((roll1))]])

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

"""
	Ground truth about ar tag pose_map
"""
# posemap_AR = np.array([[-0.22571, 0.063645, 0.20089]])
# orientmap_AR = np.array([ -0.0027935, 0.70555, -0.0043037, 0.70864])


#
# posemap_AR = np.array([[3.3404, 0.516725, 0.344577]])
# orientmap_AR = np.array([-0.0118515, -0.72394, 0.0075599, 0.68972])

poseMap_sendGoal = np.array([[-6.654101848602295, 2.6939444541931152,10]])
orientMap_sendGoal = np.array([0, 0, -0.016154152818269988, 0.9998695131599543])
poseSendGoal_AR = np.array([[0, 0, 0]])
orientSendGoal_AR = np.array([1.57, 0, 1.57])

"""
	Robot pose wrt to tag in disoriented map
"""


# poseODOM_ar3 = np.array([0.21105, 0.13398, 1.736]);
# orientODOM_ar3 = np.array([ -0.0312144, -0.704887, 0.0224505, 0.708277])

# poseODOM_ar3 = np.array([[-0.32219, 0.18232, 1.1379]])
# orientODOM_ar3 = np.array([0.052972, 0.71198, -0.0866, 0.696344])
def get_pose(msg):
	data = yaml_load("config.yaml")
	# print(data)
	pos = np.array([[data["position_x"], data["position_y"], 0.0]])
	ori = np.array([data["roll"], data["pitch"], data["yaw"]])

	posIps = np.array([msg.x_m, msg.y_m, 0.0])
	oriIps = [0,0,0]
	T_1 = np.asmatrix(getRTfromEuler(pos, ori))
	T_2 = np.asmatrix(getRTfromEuler(posIps, oriIps))
	T = T_1*T_2
	Rr = T[0:3, 0:3]
	position = T[0:3, -1]  #supress the z coordinate
	print(position)
	orientation = getQuatfromRT(T)
	# print ("position")
	# print (position)
	# print ("orientation")
	# #print (orientation)


def main():
	rospy.init_node("pose_calc_from_ips")
	rospy.loginfo('Init node ' + rospy.get_name())
	# while True:
	# 	a = get_odom()
	# ips_pose_pub = rospy.Publisher("/pose_calc_from_ips",Pose,queue_size=10)
	ips_pose_sub = rospy.Subscriber("/hedge_pos", hedge_pos,get_pose)
	rospy.spin()


if __name__ == "__main__":

    main()