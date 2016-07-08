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
		pose_topic = rospy.Subscriber("/Robot_"+self.number+"/pose",PoseStamped,self._push_pose)
		back_to_pose = rospy.Subscriber("odometry/filtered",Odometry,self._push_new_pose)

	def _push_pose(self,data):
		msg = PoseWithCovarianceStamped()
		msg.header = data.header
		pose = PoseWithCovariance()
		pose.pose = data.pose
		pose.covariance = [0] * 36
		msg.pose = pose
		pose_converted = rospy.Publisher("/"+self.name+"/pose",PoseWithCovarianceStamped,queue_size=10) #NTS this could be better resolved through use of namespaces
		pose_converted.publish(msg) 
		#NTS update to any flie, instead of just igo

	def _push_new_pose(self,data):
		pose_new = rospy.Publisher("/pose_new",PoseStamped,queue_size=10)
		msg = PoseStamped()
		msg.header = data.header
		msg.pose = data.pose.pose
		pose_new.publish(msg)


if __name__ == '__main__':
	rospy.init_node("pose_converter")
	converter = Converter()
	rospy.spin()