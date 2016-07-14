#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
import math

class wheel_controller:

	def __init__(self):
		rospy.init_node('waypoint_drive', anonymous=True)

		self.x = 0
		self.y = 0
		self.theta = 0

		self.goal_x = 0
		self.goal_y = 0
		self.goal_theta = 0

		# margin of error
		self.pos_error = 0.5
		self.theta_error = 0.1

		# proportionality constants
		self.k_d = 150
		self.k_a = 10

		rospy.Subscriber("goal_point", Pose2D, self.update_goal)
		rospy.Subscriber("/Robot_1/ground_pose", Pose2D, self.wheel_command)

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

	# def update_pos(self, data):
	# 	self.x = data.ground_pose.x
	# 	self.y = data.ground_pose.y
	# 	self.theta = data.ground_pose.theta

	def check_goal(self):
		if abs(self.x - self.goal_x) < self.pos_error and abs(self.y - self.goal_y) < self.pos_error:
			return True
		else:
			return False


	def wheel_command(self, data):
		self.x = data.x # is this actually how you get the data / is ground_pose needed
		self.y = data.y
		self.theta = data.theta

		theta_error = self.goal_theta - self.theta
		dist_error = math.sqrt(self.goal_x ** 2 + self.goal_y ** 2) - math.sqrt(self.x ** 2 + self.y ** 2) # better way to calculate euclidean distance

		if self.check_goal():
			rospy.set_param('eff/wheels/state', 0)

		elif theta_error > self.theta_error:
			rospy.set_param('eff/wheels/state', 1)
			rospy.set_param('eff/wheels/pwm_1', 230) # implement PID control
			rospy.set_param('eff/wheels/pwm_2', 170)
		else:
			rospy.set_param('eff/wheels/state', 1)
			rospy.set_param('eff/wheels/pwm_1', 170) # implement PID control
			rospy.set_param('eff/wheels/pwm_2', 170)

if __name__ == '__main__':
	wheel_ctrl = wheel_controller()
	rospy.spin()