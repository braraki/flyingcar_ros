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
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class Converter():

	def __init__(self,name,number):
		self.name=name
		self.number=number		
		
		rospy.Subscriber("/Robot_"+self.number+"/pose",PoseStamped,self._push_mocap_pose) #gets mocap data (PoseStamped)
		rospy.Subscriber("pose_localization",Odometry,self._push_localized_pose) #gets localized data (PoseWithCovarianceStampted)

	#Converts mocap data to PoseWithCovarianceStamped
	def _push_mocap_pose(self,data):
		msg = PoseWithCovarianceStamped()
		msg.header = data.header
		pose = PoseWithCovariance()
		pose.pose = data.pose
		pose.covariance = [0] * 36 #could tune the covariance matrices?
		pose.covariance[0] = -1
		msg.pose = pose
		pose_converted = rospy.Publisher("pose_mocap",PoseWithCovarianceStamped,queue_size=10) 
		pose_converted.publish(msg) 

	#Converts localized data to PoseStamped (easier to use in other nodes)
	def _push_localized_pose(self,data):
		pose_new = rospy.Publisher("pose",PoseStamped,queue_size=10)
		msg = PoseStamped()
		msg.header = data.header
		msg.pose = data.pose.pose
		pose_new.publish(msg)


if __name__ == '__main__':
	rospy.init_node("pose_converter")

	name = sys.argv[1]
	number = sys.argv[2]
	
	converter = Converter(name,number)
	
	rospy.spin()