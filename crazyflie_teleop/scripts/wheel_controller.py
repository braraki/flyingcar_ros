#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
import math

class wheel_controller:

	def __init__(self):
		rospy.wait_for_service('update_params')
		rospy.loginfo("found update_params service")
		self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

		rospy.loginfo("waiting for emergency service")
		rospy.wait_for_service('emergency')
		rospy.loginfo("found emergency service")
		self._emergency = rospy.ServiceProxy('emergency', Empty)

		self.x = 0
		self.y = 0
		self.theta = 0
		#self.prev_state = 0

		self.goal_x = 0
		self.goal_y = 0

		self.offset = 0

		# margin of error
		self.pos_error = 0.05

		rospy.Subscriber("/goal_point", Pose2D, self.update_goal)
		rospy.Subscriber("/Robot_1/ground_pose", Pose2D, self.wheel_command)

	def update_goal(self, data):
		self.goal_x = data.x
		self.goal_y = data.y
		#self.prev_state = rospy.get_param('wheels/state')

		if self.check_goal():
			rospy.set_param('wheels/state', 0)
			# if self.prev_state != 0:
			# 	rospy.set_param('wheels/state', 0)
			# 	try:
			# 		self._update_params(["wheels/state"])
			# 	except rospy.ServiceException as exc:
			# 		print("Service did not process request: " + str(exc))

		else:
			rospy.set_param('wheels/state', 1)
			# if self.prev_state != 1:
			# 	rospy.set_param('wheels/state', 1)
			# 	try:
			# 		self._update_params(["wheels/state"])
			# 	except rospy.ServiceException as exc:
			# 		print("Service did not process request: " + str(exc))
			self.offset = int(self.offset)
			#print self.offset
			rospy.set_param('wheels/pwm_1', 100 - self.offset)
			rospy.set_param('wheels/pwm_2', 100 + self.offset)

		try:
			self._update_params(["wheels/state"])
			self._update_params(["wheels/pwm_1"])
			self._update_params(["wheels/pwm_2"])
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

		# rospy.loginfo("goal_x is %s", self.goal_x)
		# rospy.loginfo("goal_y is %s", self.goal_y)
		# rospy.loginfo("goal_theta is %s", self.goal_theta)


	def check_goal(self):
		if abs(self.x - self.goal_x) < self.pos_error and abs(self.y - self.goal_y) < self.pos_error:
			return True
		else:
			return False


	def wheel_command(self, data):
		self.x = data.x
		self.y = data.y
		self.theta = data.theta

		offset_constant = 1.0

		x_e = self.goal_x - self.x
		y_e = self.goal_y - self.y

		if x_e == 0:
			if y_e > 0:
				theta_heading = math.pi/2
			else:
				theta_heading = -math.pi/2
		else:
			theta_heading = math.atan2(y_e, x_e)

		theta_error = theta_heading - self.theta

		#print theta_heading, theta_error

		self.offset = theta_error/math.pi * 170 * offset_constant
		self.offset = min(self.offset, 85)
		self.offset = max(self.offset, -85)

		#print x_e, y_e, self.offset

		#print "offset: %f" % (self.offset)

		# rospy.loginfo("x is %s", self.x)
		# rospy.loginfo("y is %s", self.y)
		# rospy.loginfo("theta is %s", self.theta)

if __name__ == '__main__':
	rospy.init_node('wheel_control')
	wheel_ctrl = wheel_controller()
	rospy.spin()