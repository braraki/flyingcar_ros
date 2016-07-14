#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
import math

class test_drive():

	def __init__(self):
		self.x = 0
		self.y = 0
		self.theta = 0

		self.goal_x = 0
		self.goal_y = 0
		self.goal_theta = 0
		
		# rospy.Subscriber("/Robot_1/ground_pose", Pose2D, self.callback)
		# rospy.Subscriber("goal_point", Pose2D, self.update_goal)
		rospy.Subscriber("/Robot_1/ground_pose", Pose2D, self.update_pos)

	def callback(self, data):
		self.x = data.x
		self.y = data.y
		self.theta = data.theta

		rospy.loginfo("x is %s", self.x)
		rospy.loginfo("y is %s", self.y)
		rospy.loginfo("theta is %s", self.theta)

	def update_pos(self, data):
		self.x = data.x
		self.y = data.y
		self.theta = data.theta

		rospy.loginfo("x is %s", self.x)
		rospy.loginfo("y is %s", self.y)
		rospy.loginfo("theta is %s", self.theta)

	def update_goal(self, data):
		self.goal_x = data.x
		self.goal_y = data.y

		if self.goal_x == 0:
			if self.goal_y > 0:
				self.goal_theta = pi/2
			else:
				self.goal_theta = -pi/2
		else:
			self.goal_theta = math.atan2(self.goal_y, self.goal_x)

		rospy.loginfo("goal_x is %s", self.goal_x)
		rospy.loginfo("goal_y is %s", self.goal_y)
		rospy.loginfo("goal_theta is %s", self.goal_theta)

		rospy.set_param('wheels/state', 1)
		rospy.set_param('wheels/pwm_1', 150) # implement PID control
		rospy.set_param('wheels/pwm_2', 150)

	def print_info(self, data):
		rospy.loginfo("goal_x is %s", self.goal_x)
		rospy.loginfo("goal_y is %s", self.goal_y)
		rospy.loginfo("goal_theta is %s", self.goal_theta)
		rospy.loginfo("x is %s", self.x)
		rospy.loginfo("y is %s", self.y)
		rospy.loginfo("theta is %s", self.theta)

	def update_pos(self, data):
		self.x = data.x # is this actually how you get the data / is ground_pose needed
		self.y = data.y
		self.theta = data.theta

	def car_go(self, data):
		self.x = data.x
		self.y = data.y
		self.theta = data.theta

		rospy.loginfo("x is %s", self.x)
		rospy.loginfo("y is %s", self.y)
		rospy.loginfo("theta is %s", self.theta)

		rospy.set_param('wheels/state', 1)
		rospy.set_param('wheels/pwm_1', 150) # implement PID control
		rospy.set_param('wheels/pwm_2', 150)

if __name__ == '__main__':
	rospy.init_node('test_drive', anonymous=True)
	test = test_drive()
	rospy.spin()