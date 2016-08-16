#!/usr/bin/env python

#Node for displaying Crazyflie's actual path 
#author: Sarah Pohorecky - spohorec@mit.edu

import rospy
import tf
import sys

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import threading

class PathDisplay():

	def __init__(self,in_topic,out_topic):
		self.in_topic = in_topic
		self.out_topic = out_topic

		#--------------------------------------------------------

		#Inits. Path message with a pose of (0,0,0,0,0,0,1)
		#So publisher doesn't freak out with no data
		self.init_pose = PoseStamped()

		self.init_pose.header.seq = 0
		self.init_pose.header.stamp = rospy.Time.now()
		self.init_pose.header.frame_id = "/world"
		self.init_pose.pose.position.x = 0
		self.init_pose.pose.position.y = 0
		self.init_pose.pose.position.z = 0
		
		yaw = 0
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		
		self.init_pose.pose.orientation.x = quaternion[0]
		self.init_pose.pose.orientation.y = quaternion[1]
		self.init_pose.pose.orientation.z = quaternion[2]
		self.init_pose.pose.orientation.w = quaternion[3]

		self.paths = []
		self.paths.append(self.init_pose)

		self.msg = Path()

		self.msg.header.seq = 0
		self.msg.header.stamp = rospy.Time.now()
		self.msg.header.frame_id = "/world"
		self.msg.poses.append(self.init_pose)

		#--------------------------------------------------------

		rospy.Subscriber(self.in_topic,PoseStamped,self.make_path)

		self.pub = rospy.Publisher(self.out_topic,Path,queue_size=10)

		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			self.pub_path()
			r.sleep()

	#Takes in pose data and adds it to the path
	def make_path(self,data):
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose
		self.msg.poses.append(data)

	def pub_path(self):
		self.msg.header.stamp = rospy.Time.now()
		self.pub.publish(self.msg)

if __name__ == '__main__':
	rospy.init_node("path_display")
	display_pose = PathDisplay("pose","path")
	



