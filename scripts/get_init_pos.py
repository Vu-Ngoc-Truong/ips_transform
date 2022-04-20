#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import os
import sys
import rospy
import rospkg
import copy
import json
import yaml
from std_stamped_msgs.msg import StringStamped, StringAction, StringFeedback, StringResult, StringGoal, EmptyStamped
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import  Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from marvelmind_nav.msg import hedge_pos
import numpy as np
import tf
from math import *
from get_pose_function import get_pose, distance_from_coordinates, center_of_three_point

common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

from common_function import (
    MIN_FLOAT,
    EnumString,
    lockup_pose,
    offset_pose_x,
    offset_pose_yaw,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    offset_pose_xy_theta,
    angle_two_pose,
    pose_dict_template
)

class MainState(EnumString):
    NONE                = -1
    STATE_1             = 0
    STATE_2             = 1

class PoseEstimate():
	def __init__(self, name, *args, **kwargs):
		self.init_variable(*args, **kwargs)
		print("Name: ",name)
		# Action

		# Publisher
		self.pub_init_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
		self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		# Subscriber
		rospy.Subscriber("/hedge_pos",hedge_pos,self.get_pose_ips_cb)
		rospy.Subscriber("/final_cmd_vel_mux/output",Twist,self.final_cmd_vel_cb)
		rospy.Subscriber("/odom",Odometry,self.get_odom_cb)
        # Service

		# self.loop()

        # self.sub_ori_vslam = rospy.Subscriber("/oriVslam",,self.get_ori_vslam)

	def init_variable(self, *args, **kwargs):
		# Read YAML file
		self.config_file = kwargs["config_file"]
		rospy.loginfo("config_file: %s" % self.config_file)
		self.cfg_file_path =os.path.join(rospkg.RosPack().get_path('ips_transform'), 'cfg', 'ips_config.yaml')

		self.init_pose = PoseWithCovarianceStamped()
		self.init_pose.pose.pose.position.x = 0
		self.init_pose.pose.pose.position.y = 0
		self.init_pose.pose.pose.position.z = 0
		self.init_pose.pose.pose.orientation.x = 0
		self.init_pose.pose.pose.orientation.z = 0
		self.init_pose.pose.pose.orientation.y = 0
		self.init_pose.pose.pose.orientation.w = 1
		self.init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
										0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
										0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
										0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
										0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
										0.0, 0.0, 0.0, 0.0, 0.0, 0.068]

		# Get point when robot rotary self
		self.state = "get_first_point"
		self.position_rotary = list()
		self.robot_angle_init = 0
		self.robot_angle_tolerance = 0.2 # radian
		# self.position_second =None
		self.min_distance = 0.2 # met
		self.rotation_vel = 0.5 # rad/s

		# Get robot speed :
		self.robot_cmd_vel_linear = 0
		self.robot_cmd_vel_rotary = 0

		# Set robot speed:
		self.robot_cmd_vel = Twist()
		self.robot_cmd_vel.linear.x  = 0
		self.robot_cmd_vel.angular.z = 0

		# Get robot odom:
		self.robot_odom_angle_yaw = 0


	def shutdown(self):
		rospy.loginfo("Shuting down")



	def yaml_load(self,filepath):
		with open(filepath, "r") as read_file:
			data = yaml.load(read_file, Loader=yaml.FullLoader)
		return data

	"""
	 ######     ###    ##       ##       ########     ###     ######  ##    ##
	##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
	##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
	##       ##     ## ##       ##       ########  ##     ## ##       #####
	##       ######### ##       ##       ##     ## ######### ##       ##  ##
	##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
	 ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
	"""

	def get_pose_ips_cb(self,msg):
		pose_ips = np.array([msg.x_m, msg.y_m, 0])
		orien_ips = np.array([0,0,0])
		TTT = get_pose(pose_ips, orien_ips, self.cfg_file_path)
		# print(TTT)
		pose_map_xy = np.array([TTT[0,0],TTT[1,0]])
		# print("position x: {}	y : {}".format(format(pose_map_xy[0],'0.4f'),format(pose_map_xy[1],'0.4f')))

		if self.state == "get_first_point":
			rospy.loginfo("state: {}".format(self.state))
			self.position_rotary.append(pose_map_xy)
			self.state = "get_2nd_point"
			# Get current angle of robot:
			self.robot_angle_init = self.robot_odom_angle_yaw
			# Rotary robot
			self.robot_cmd_vel.linear.x  = 0
			self.robot_cmd_vel.angular.z = self.rotation_vel
			self.pub_cmd_vel.publish(self.robot_cmd_vel)


		elif self.state == "get_2nd_point":
			rospy.loginfo("state: {}".format(self.state))
			distance = distance_from_coordinates(self.position_rotary[0],pose_map_xy)
			print("distance : {}".format(distance))
			# Rotary robot
			self.robot_cmd_vel.linear.x  = 0
			self.robot_cmd_vel.angular.z = self.rotation_vel
			self.pub_cmd_vel.publish(self.robot_cmd_vel)
			# print(abs(distance))
			if  distance > self.min_distance  :
				self.position_rotary.append(pose_map_xy)
				rospy.loginfo("Point 2 OK")
				self.state = "get_3rd_point"
			elif distance <= self.min_distance :
				rospy.logwarn("distance travel not enough!")

		elif self.state == "get_3rd_point":
			rospy.loginfo("state: {}".format(self.state))
			distance = distance_from_coordinates(self.position_rotary[1],pose_map_xy)
			print("distance : {}".format(distance))
			# print(abs(distance))
			if  distance > self.min_distance  :
				rospy.loginfo("Point 3 OK")
				self.position_rotary.append(pose_map_xy)
				self.state = "get_3_point_complete"
			elif distance <= self.min_distance :
				rospy.logwarn("distance travel not enough!")
							# Rotary robot
				self.robot_cmd_vel.linear.x  = 0
				self.robot_cmd_vel.angular.z = self.rotation_vel
				self.pub_cmd_vel.publish(self.robot_cmd_vel)
		elif self.state == "get_3_point_complete":
			rospy.loginfo("state: {}".format(self.state))
			center, radius, yaw = center_of_three_point(self.position_rotary[0],self.position_rotary[1],self.position_rotary[2])
			self.init_pose.pose.pose.position.x = center[0]
			self.init_pose.pose.pose.position.y = center[1]
			yaw = yaw - 0.7854	# offset with tf ips_base link
			self.init_pose.pose.pose.orientation.x = quaternion_from_euler(0,0,yaw)[0]
			self.init_pose.pose.pose.orientation.y = quaternion_from_euler(0,0,yaw)[1]
			self.init_pose.pose.pose.orientation.z = quaternion_from_euler(0,0,yaw)[2]
			self.init_pose.pose.pose.orientation.w = quaternion_from_euler(0,0,yaw)[3]
			# print(quaternion_from_euler(0,0,yaw))
			rospy.loginfo("Base link: {}  yaw: {}".format(self.state,degrees(yaw)))
			self.pub_init_pose.publish(self.init_pose)
			self.state = "rotation to init angle"

		elif self.state == "rotation to init angle":
			rospy.loginfo("state: {}".format(self.state))
			# Slow robot
			self.robot_cmd_vel.linear.x  = 0
			self.robot_cmd_vel.angular.z = self.rotation_vel
			self.pub_cmd_vel.publish(self.robot_cmd_vel)
			if abs(self.robot_angle_init - self.robot_odom_angle_yaw) < self.robot_angle_tolerance:
				# Stop robot
				self.robot_cmd_vel.linear.x  = 0
				self.robot_cmd_vel.angular.z = 0
				self.pub_cmd_vel.publish(self.robot_cmd_vel)
				self.state = "init_pose_complete"
				rospy.loginfo("state: {}".format(self.state))


	def final_cmd_vel_cb(self,msg):
		self.robot_cmd_vel_linear = msg.linear.x
		self.robot_cmd_vel_rotary = msg.angular.z

	def get_odom_cb(self,msg):
		self.robot_odom_angle_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
														msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]


	"""
	##        #######   #######  ########
	##       ##     ## ##     ## ##     ##
	##       ##     ## ##     ## ##     ##
	##       ##     ## ##     ## ########
	##       ##     ## ##     ## ##
	##       ##     ## ##     ## ##
	########  #######   #######  ##
	"""

	def loop(self):
		_state = MainState.NONE
		_prev_state = MainState.NONE
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			if _state != _prev_state:
				print_debug('State: {} -> {}'.format(_prev_state.toString(), _state.toString()))
				_prev_state = _state

			if _state == MainState.STATE_1:
				_state = MainState.STATE_2
			elif _state == MainState.STATE_2:
				_state = MainState.STATE_1
			rate.sleep()


def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-d", "--ros_debug",
                    action="store_true", dest="log_debug", default=False, help="log_level=rospy.DEBUG")
    parser.add_option("-c", "--config_file", dest="config_file",
                    default=os.path.join(rospkg.RosPack().get_path('ips_transform'), 'cfg', 'config.yaml'))

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
	(options, args) = parse_opts()
	log_level = None
	if options.log_debug:
		log_level = rospy.DEBUG
	rospy.init_node('pose_init_with_ips', log_level=log_level)
	rospy.loginfo('Init node ' + rospy.get_name())
	PoseEstimate(rospy.get_name(), **vars(options))
	rospy.spin()

if __name__ == '__main__':
    main()