#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import sys


class Converter():

	def __init__(self):
		self.name=sys.argv[1]
		self.number=sys.argv[2]			
		
		rospy.Subscriber("/Robot_"+self.number+"/pose",PoseStamped,self._push_mocap_pose)
		rospy.Subscriber("pose_localization",Odometry,self._push_localized_pose)

	def _push_mocap_pose(self,data):
		msg = PoseWithCovarianceStamped()
		msg.header = data.header
		pose = PoseWithCovariance()
		pose.pose = data.pose
		pose.covariance = [0] * 36
		pose.covariance[0] = -1
		msg.pose = pose
		pose_converted = rospy.Publisher("pose_mocap",PoseWithCovarianceStamped,queue_size=10) 
		pose_converted.publish(msg) 

	def _push_localized_pose(self,data):
		pose_new = rospy.Publisher("pose",PoseStamped,queue_size=10)
		msg = PoseStamped()
		msg.header = data.header
		msg.pose = data.pose.pose
		pose_new.publish(msg)


if __name__ == '__main__':
	rospy.init_node("pose_converter")
	converter = Converter()
	rospy.spin()