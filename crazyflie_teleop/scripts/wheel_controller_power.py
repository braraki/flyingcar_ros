#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from crazyflie_teleop.msg import DriveCmd
from crazyflie_teleop.msg import WheelParams
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
import math
import sys
import tf
import threading
import time

class wheel_controller:

	def __init__(self, cf_num, name, radio_id):
		#rospy.wait_for_service('update_params')
		#rospy.loginfo("found update_params service")
		#self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
		self.param_pub = rospy.Publisher('/id' + str(radio_id) + '/update_params', WheelParams, queue_size=10)

		rospy.set_param('in_air', False)

		self.cf_num = cf_num
		self.name = name
		self.radio_id = radio_id

		self.wheel_command()


	def wheel_command(self): #NTS could this be having bad timing interactions? It's completely possible that the goal updates during a runthrough.
		r = rospy.Rate(30)
		while not rospy.is_shutdown():	#NTS if goals and positions/angles change while running, this could affect calculation? Add a lock to be safe?
			# print max(0,min(self.baseline + self.theta_offset + self.speed_offset, 255)), max(0,min(self.baseline - self.theta_offset + self.speed_offset, 255))
			wheel_params = WheelParams()
			wheel_params.tf_prefix = self.name
			wheel_params.state = 1
			wheel_params.pwm1 = 230
			wheel_params.pwm2 = 230
			self.param_pub.publish(wheel_params)
			r.sleep()
		return


if __name__ == '__main__':
	rospy.init_node('wheel_control')
	cf_num = sys.argv[1]
	name = sys.argv[2]
	radio_id = sys.argv[3]
	wheel_ctrl = wheel_controller(cf_num,name,radio_id)
	rospy.spin()