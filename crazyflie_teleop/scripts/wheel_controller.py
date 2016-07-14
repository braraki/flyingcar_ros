#!/usr/bin/env python

import rospy
from gemoetry_msgs.msg import Pose2D
import math

class wheel_controller:

	def __init__(self):
		self.x = 0
		self.y = 0
		self.theta = 0

		self.goal_x = 0
		self.goal_y = 0

		# margin of error
		self.pos_error = 0.5
		self.theta_error = 0.1

		rospy.Subscriber('wheel_command', Pose2D, update_goal)
		rospy.Subscriber('eff/ground_pose', Pose2D, wheel_command)

	def update_goal(self, data):
		self.goal_x = data.x
		self.goal_y = data.y

	'''
	def update_pos(self, data):
		self.x = data.ground_pose.x
		self.y = data.ground_pose.y
		self.theta = data.ground_pose.theta
	'''

	def check_goal(self):
		if abs(self.x - self.goal_x) < self.pos_error and abs(self.y - self.goal_y) < self.pos_error:
			return True
		else:
			return False

	def wheel_command(self, data):
		self.x = data.ground_pose.x # is this actually how you get the data / is ground_pose needed
		self.y = data.ground_pose.y
		self.theta = data.ground_pose.theta

		if check_goal():
			rospy.set_param('wheels/state', 0)
		else:
			theta_error = math.atan2(self.goal_y / self.goal_x) - self.theta
			dist_error = sqrt(self.goal_x ** 2 + self.goal_y ** 2) - sqrt(self.x ** 2 + self.y ** 2) # better way to calculate euclidean distance

			if theta_error > self.theta_error:
				rospy.set_param('wheels/state', 1)
				rospy.set_param('wheels/pwm_1', 230) # implement PID control
				rospy.set_param('wheels/pwm_2', 170)
			else:
				rospy.set_param('wheels/state', 1)
				rospy.set_param('wheels/pwm_1', 170) # implement PID control
				rospy.set_param('wheels/pwm_2', 170)

if __name__ = '__main__':
	rospy.init_node('waypoint_drive')
	wheel_ctrl = wheel_controller()
	rospy.spin()