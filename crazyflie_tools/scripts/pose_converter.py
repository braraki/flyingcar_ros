#!/usr/bin/env python

#Node for converting msg types for localization
#author: Sarah Pohorecky - spohorec@mit.edu

#ARGUMENTS: name (Crazyflie name); number (Robot_X mocap number)

#------------------------------------

#Conversions:
##### /Robot_X/pose --> /name/pose_mocap (PoseStamped  --> PoseWithCovarianceStamped)

##### /name/pose_localization --> /name/pose (PoseWithCovarianceStamped --> PoseStamped)

#------------------------------------

import rospy

import sys

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import tf


class Converter():

	def __init__(self,name,number):
		self.name=name
		self.number=number
		self.frame = "/vicon/CF" + self.number + "/CF" + self.number

		self.pose_converted = rospy.Publisher("pose_mocap",PoseWithCovarianceStamped,queue_size=10) 
		self.pose_new = rospy.Publisher("pose",PoseStamped,queue_size=10)
		rospy.Subscriber(self.frame,TransformStamped,self._push_mocap_pose) #gets mocap data (PoseStamped)
		rospy.Subscriber("pose_localization",Odometry,self._push_localized_pose) #gets localized data (PoseWithCovarianceStampted)

		self.tf = tf.TransformListener()
			
	#Converts mocap data to PoseWithCovarianceStamped
	def _push_mocap_pose(self,data):
		if self.tf.frameExists(self.frame) and self.tf.frameExists("/world"):
			t = self.tf.getLatestCommonTime(self.frame, "/world")
			position, quaternion = self.tf.lookupTransform("/world", self.frame, t)
			msg = PoseWithCovarianceStamped()
			pose = PoseWithCovariance()
			pose.pose.position.x = position[0]
			pose.pose.position.y = position[1]
			pose.pose.position.z = position[2]
			pose.pose.orientation.x = quaternion[0]
			pose.pose.orientation.y = quaternion[1]
			pose.pose.orientation.z = quaternion[2]
			pose.pose.orientation.w = quaternion[3]
			pose.covariance = [0] * 36 #could tune the covariance matrices?
			pose.covariance[0] = -1
			msg.pose = pose
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = "/bug/base_link"
			self.pose_converted.publish(msg) 

	#Converts localized data to PoseStamped (easier to use in other nodes)
	def _push_localized_pose(self,data):
		msg = PoseStamped()
		msg.header = data.header
		msg.pose = data.pose.pose
		self.pose_new.publish(msg)


if __name__ == '__main__':
	rospy.init_node("pose_converter")

	name = sys.argv[1]
	number = sys.argv[2]
	
	converter = Converter(name,number)
	
	rospy.spin()