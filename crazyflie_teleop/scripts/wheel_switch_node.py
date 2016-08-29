#!/usr/bin/env python

import rospy
from crazyflie_teleop.msg import DriveCmd
from crazyflie_teleop.msg import WheelParams
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
import sys
import threading
import time
from enum import Enum
import copy

class CF:
	def __init__(self,state,pwm1,pwm2):
		self.state = state
		self.pwm1 = pwm1
		self.pwm2 = pwm2

class WheelSwitchNode:

	def __init__(self):
		rospy.Subscriber("update_params", WheelParams, self.new_param)

		self.cfs = {}
		self.update_services = {}

		self.publish_parameters()

	def new_param(self, data):
		name = data.tf_prefix
		if name in self.cfs:
			self.cfs[name].state = data.state
			self.cfs[name].pwm1 = data.pwm1
			self.cfs[name].pwm2 = data.pwm2
		else:
			self.cfs[name] = CF(data.state,data.pwm1,data.pwm2)
			cf_name = name[4:]
			rospy.wait_for_service(cf_name + '/update_params')
			rospy.loginfo("found update_params service")
			self.update_services[name] = rospy.ServiceProxy(cf_name + '/update_params', UpdateParams)


	def publish_parameters(self):
		r = rospy.Rate(3)

		while not rospy.is_shutdown():
			if self.cfs and self.update_services:
				cfs = copy.copy(self.cfs)
				for name in cfs:
					cf = cfs[name] 
					cf_name = name[4:]
					rospy.set_param(cf_name + '/wheels/state', cf.state)
					rospy.set_param(cf_name + '/wheels/pwm_1', cf.pwm1)
					rospy.set_param(cf_name + '/wheels/pwm_2', cf.pwm2)
					try:
						print "sending commands to " + name + ": " + str(cf.state) + " " + str(cf.pwm1) + " " + str(cf.pwm2)
						self.update_services[name](["wheels/state"])
						#rospy.sleep(0.01)
						self.update_services[name](["wheels/pwm_1"])
						#rospy.sleep(0.01)
						self.update_services[name](["wheels/pwm_2"])
						rospy.sleep(0.07)
					except rospy.ServiceException as exc:
						print("Service did not process request: " + str(exc))
				r.sleep()
		return


if __name__ == '__main__':
	rospy.init_node('wheel_switch_node')
	wheel_switch_node = WheelSwitchNode()
	rospy.spin()