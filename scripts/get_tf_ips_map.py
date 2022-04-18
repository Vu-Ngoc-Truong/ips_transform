#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from math import cos, sin, pi, sqrt
import rospy
import yaml
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from marvelmind_nav.msg import hedge_pos
import tf
check = False
def yaml_dump(filepath,value):
        with open(filepath,'w') as read_file:
            data = yaml.dump(value,read_file,sort_keys=True,indent=4 )

def get_odom():
	try:

		tf_listener = tf.TransformListener()
		rospy.loginfo("Checking for transform between ar-tag and map frames")
		tf_listener.waitForTransform("/map","/ips_link",  rospy.Time(), rospy.Duration(4.0))
		# print(rospy.get_time())
		rospy.loginfo("transform found :)")
		pos, ori = tf_listener.lookupTransform('/ips_link','/map', rospy.Time(0))
		pose = (pos,ori)
		############ self.rot is in quaternion ############

		rospy.loginfo("transformed successfully")
		# print(pos)

		return pose
	except(tf.Exception, tf.ConnectivityException, tf.LookupException):
		rospy.loginfo("TF exception")
		return False

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
	print(R)
	return R
def getRTfromEuler(translation,euler):
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
	print(R)
	return R
def getQuatfromRT(RT):
	qw = sqrt(1+RT[0,0] + RT[1,1]+ RT[2,2])/2
	qx = (RT[2,1] - RT[1,2])/(4*qw)
	qy = (RT[0,2] - RT[2,0])/(4*qw)
	qz = (RT[1,0] - RT[0,1])/(4*qw)
	return np.array([qx, qy, qz, qw])



# poseMap_sendGoal = np.array([[-6.654101848602295, 2.6939444541931152, 0]])
# orientMap_sendGoal = np.array([0, 0, -0.016154152818269988, 0.9998695131599543])
# poseSendGoal_AR = np.array([[0, 0, 0]])
# orientSendGoal_AR = np.array([1.57, 0, 1.57])


def getPoseIps(data):
	# get coordinate from map
	pose = get_odom()
	print(pose)
	if pose:
		print("get odom")
		# Get coordinate from ip
		position= np.array([data.x_m, data.y_m, 0])
		orientation = np.array([0, 0, 0.4712])
		# print("ips coordinate: {}".format(ips_coord))

		# tranfer to array
		# position = np.array([pose[0][0],pose[0][1]] )
		# print("odom coordinate: {}".format(position))

		T_Map_BaseLink = np.asmatrix(getRTfromQuat(np.asarray(pose[0]), np.asarray(pose[1])))
		T_IpsOrigin_BaseLink = np.asmatrix(getRTfromEuler(position, orientation))
		T_Map_IpsOrigin = T_Map_BaseLink*np.linalg.inv(T_IpsOrigin_BaseLink)
		position = T_Map_IpsOrigin[0:3, -1]  # supress the z coordinate
		orientation = getQuatfromRT(T_Map_IpsOrigin)
		# print ("position")
		# print (position)
		# print ("orientation")
		# print (orientation)
		value= {'position_x': float(position[0]),
				'position_y': float(position[1]),
				'position_z': 0
				}
		print("value",value)
		# yaml_dump("config.yaml",value)
		print("succsess dump to yaml file")

		# return position, orientation

def main():
	rospy.init_node("save_tf_map_IpsOrigin")
	rospy.loginfo('Init node ' + rospy.get_name())
	# while True:
	# 	a = get_odom()
	ips_sub = rospy.Subscriber("/hedge_pos", hedge_pos,getPoseIps)
	rospy.spin()




if __name__ == "__main__":

    main()

		# T_SA = getPoseIps(poseMap_sendGoal,orientMap_sendGoal)