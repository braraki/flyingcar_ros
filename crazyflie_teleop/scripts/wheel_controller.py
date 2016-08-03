#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from crazyflie_teleop.msg import DriveCmd
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

		self.prev_speeds = []
		self.speed = 0

		self.prev_update_time = 0

		self.goal_x = 0
		self.goal_y = 0
		self.goal_speed = 0

		self.baseline = 0
		self.theta_offset = 0
		self.speed_offset = 0

		self.theta_heading = 0
		self.theta_error = 0

		# margin of error
		self.pos_error = 0.05

		rospy.Subscriber("goal_point", DriveCmd, self.update_goal)
		#rospy.Subscriber("goal_point", Pose2D, self.update_goal)
		rospy.Subscriber("/Robot_3/pose", PoseStamped, self.calculate_speed)
		rospy.Subscriber("/Robot_3/ground_pose", Pose2D, self.wheel_command)

	def update_goal(self, data):
		self.goal_x = data.x
		self.goal_y = data.y
		self.goal_speed = data.speed
		#self.goal_speed = data.theta

		if self.check_goal():
			rospy.set_param('wheels/state', 0)
		else:
			# print "x is ", self.x
			# print "y is ", self.y
			# print "theta is " , self.theta
			# print "theta heading is ", self.theta_heading
			# print "theta error is " , self.theta_error
			# print "theta offset is " , self.theta_offset
			rospy.set_param('wheels/state', 1)
			# self.theta_offset = int(self.theta_offset)
			# print "int theta offset is ", self.theta_offset
			# print 100 - self.theta_offset
			# print 100 + self.theta_offset
			# rospy.set_param('wheels/pwm_1', 100 + self.theta_offset)
			# rospy.set_param('wheels/pwm_2', 100 - self.theta_offset)
			# rospy.set_param('wheels/pwm_1', min(100 - self.theta_offset, 255))
			# rospy.set_param('wheels/pwm_2', min(100 + self.theta_offset, 255))
			# rospy.set_param('wheels/pwm_1', min(self.baseline - self.theta_offset, 255))
			# rospy.set_param('wheels/pwm_2', min(self.baseline + self.theta_offset, 255))
			rospy.set_param('wheels/pwm_1', min(self.baseline + self.theta_offset + self.speed_offset, 255))
			rospy.set_param('wheels/pwm_2', min(self.baseline - self.theta_offset + self.speed_offset, 255))

		try:
			self._update_params(["wheels/state"])
			self._update_params(["wheels/pwm_1"])
			self._update_params(["wheels/pwm_2"])
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

	def calculate_speed(self, data):
		prev_x = self.x
		prev_y = self.y

		self.x = data.pose.position.x
		self.y = data.pose.position.y

		update_time = data.header.stamp.nsecs
		speed = math.sqrt((self.x-prev_x)**2 + (self.y-prev_y)**2)/(update_time-self.prev_update_time) * 10**9
		if len(self.prev_speeds) == 10:
			del self.prev_speeds[0]
		self.prev_speeds.append(speed)
		self.prev_update_time = update_time

		#print speed


	def check_goal(self):
		if abs(self.x - self.goal_x) < self.pos_error and abs(self.y - self.goal_y) < self.pos_error:
			return True
		else:
			return False

	def wheel_command(self, data):
		self.x = data.x
		self.y = data.y
		self.theta = data.theta

		# print "x is ", self.x
		# print "y is ", self.y
		# print "theta is " , self.theta

		#calculate baseline
		self.baseline = (self.goal_speed + 0.0092033)/0.00084351
		self.baseline = min(255, self.baseline)
		self.baseline = max(0, self.baseline)
		self.baseline = int(self.baseline)
		#print self.baseline

		# calculate theta offset
		theta_offset_constant = 1

		x_e = self.goal_x - self.x
		y_e = self.goal_y - self.y

		if x_e == 0:
			if y_e > 0:
				self.theta_heading = math.pi/2
			else:
				self.theta_heading = -math.pi/2
		else:
			self.theta_heading = math.atan2(y_e, x_e)

		# print "theta heading is ", theta_heading

		self.theta_error = self.theta_heading - self.theta

		# print "theta error is " , theta_error

		self.theta_offset = self.theta_error/math.pi * 170 * theta_offset_constant
		self.theta_offset = min(self.theta_offset, 85)
		self.theta_offset = max(self.theta_offset, -85)

		#print "theta offset is " , self.theta_offset

		self.theta_offset = int(self.theta_offset)

		# calculate speed offset
		speed_offset_constant = 10

		self.speed = float(sum(self.prev_speeds))/len(self.prev_speeds) if len (self.prev_speeds) > 0 else 0
		speed_error = self.goal_speed - self.speed

		self.speed_offset = speed_error * speed_offset_constant
		self.speed_offset = int(self.speed_offset)


if __name__ == '__main__':
	rospy.init_node('wheel_control')
	wheel_ctrl = wheel_controller()
	rospy.spin()