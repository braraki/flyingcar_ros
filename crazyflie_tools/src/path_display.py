#!/usr/bin/env python

import rospy
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathDisplay():

	def __init__(self):
		
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

		#--------------------------------------------------------

		self.paths = []
		self.paths.append(self.init_pose)

		#--------------------------------------------------------		

		self.msg = Path()

		self.msg.header.seq = 0
		self.msg.header.stamp = rospy.Time.now()
		self.msg.header.frame_id = "/world"
		self.msg.poses.append(self.init_pose)

		print self.msg

		#--------------------------------------------------------

		rospy.Subscriber("pose_new",PoseStamped,self.make_path)


		self.pub = rospy.Publisher("path",Path,queue_size=10)
		while not rospy.is_shutdown():
			self.pub_path()

	def make_path(self,data):
		self.msg.poses.append(data)
		#print "got data!"

	def pub_path(self):
		self.msg.header.stamp = rospy.Time.now()
		#self.msg.poses = self.paths
		self.pub.publish(self.msg)
		#print "published"

if __name__ == '__main__':
	rospy.init_node("path_display")
	display = PathDisplay()

