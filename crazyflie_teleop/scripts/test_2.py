#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

class test_drive():

	def __init__(self):
		self.x = 0
		self.y = 0

		self.pwm = 0

		print "Hello"

		rospy.wait_for_service('update_params')
		rospy.loginfo("found update_params service")
		self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

		rospy.loginfo("waiting for emergency service")
		rospy.wait_for_service('emergency')
		rospy.loginfo("found emergency service")
		self._emergency = rospy.ServiceProxy('emergency', Empty)


		rospy.Subscriber("goal_point", Pose2D, self.drive)

	def drive(self, data):
		self.x = data.x
		self.y = data.y

		rospy.loginfo("x is %s", self.x)
		rospy.loginfo("y is %s", self.y)
		rospy.loginfo(self.pwm)

		rospy.set_param("wheels/state", 1)
		rospy.set_param("wheels/pwm_1", self.pwm)
		rospy.set_param("wheels/pwm_2", self.pwm)

		try:
			self._update_params(["wheels/state"])
			self._update_params(["wheels/pwm_1"])
			self._update_params(["wheels/pwm_2"])
			self.pwm += 10
			if self.pwm > 255:
				self.pwm = 0
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
			self.pwm = 0

if __name__ == '__main__':
	rospy.init_node('test_2', anonymous=True)
	test = test_drive()
	rospy.spin()