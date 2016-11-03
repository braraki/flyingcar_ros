#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from crazyflie_teleop.msg import DriveCmd
from crazyflie_teleop.msg import WheelParams

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3

import sys
import tf
import threading
import time

from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty


class CrawlerController:

	def __init__(self):
		print "waiting for update params..."
		rospy.wait_for_service('update_params')
		rospy.loginfo("found update_params service")
		self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

		#desired position/orientation
		self.d_phi = 0
		self.d_x = 0.5
		self.d_y = 0.001

		#actual position
		self.phi = 0
		self.x = 0
		self.y = 0

		#controller gains
		self.k_phi = 5
		self.k_pos = 250

		#self.d_phi_dot = 0

		#error tolerences
		self.phi_tolerence = 0.025
		self.pos_tolerence = 0.05

		#msg def
		self.msg = Twist()
		self.msg.linear = Vector3(0,0,0)
		self.msg.angular = Vector3(0,0,0)

		self.yaw_rate = 200 #cmd_vel yaw rate. 200 is max system will take from PS3 controller

		print "starting up!"

		rospy.Subscriber("pose_localization", Odometry, self._get_pos)

		self.pub_cmd = rospy.Publisher("/lam/cmd_vel",Twist,queue_size=10)

		self.update = True

		cmd_thread  = threading.Thread(target=self.publish_cmd_vel)
		ctrl_thread = threading.Thread(target=self.controller)

		cmd_thread.daemon = True
		ctrl_thread.daemon = True

		cmd_thread.start()
		ctrl_thread.start()

	def _get_pos(self,data):
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		q = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		self.phi = tf.transformations.euler_from_quaternion(q)[2] - math.pi/2.0
		if self.phi < -math.pi:
			self.phi = self.phi + 2*math.pi

		#print "yaw: " + str(180*self.theta/math.pi)

		#self.vel_x = data.twist.twist.linear.x
		#self.vel_y = data.twist.twist.linear.y


	def controller(self):
		r = rospy.Rate(30)

		while not rospy.is_shutdown():

			e_x = self.x - self.d_x
			e_y = self.y - self.d_y

			self.d_phi = math.atan((self.d_x-self.x)/(self.d_y-self.y))
			if (e_x > 0):
				self.d_phi = -(math.pi-self.d_phi)

			e_phi = self.phi - self.d_phi
			

			#print self.phi, self.d_phi, e_phi

			e_pos = math.sqrt(e_x**2+e_x**2)

			if not (abs(e_phi) < self.phi_tolerence):
				#print "TRUE"
				#u = -self.k_phi*math.sin(e_phi) + self.d_phi_dot

				thrust_range = (15000,15000)

				yaw = math.sin(e_phi/2)*600 + 300#+ 100#- 100 #gets sign of yaw (turning direction)
				thrust = 18000 # thrust_range[0] + abs(u)/self.k_phi*(thrust_range[1]-thrust_range[0]) #rescales thrust from 0-k_phi to thrust_range

			else:
				#print "FALSE"
				yaw = 200
				thrust = 15000

			print self.phi, self.d_phi, e_phi, yaw
			if not (abs(e_x) < self.pos_tolerence and abs(e_y) < self.pos_tolerence):
				wheel_speed = self.k_pos*(e_pos)
				rospy.set_param("wheels/state",4)
				rospy.set_param("wheels/pwm_2", 100)
			else:
				self.at_path()

			#print "update params!"
			try:
				self._update_params(["wheels/state"])
				self._update_params(["wheels/pwm_2"])

			except rospy.ServiceException as exc:
						print("Service did not process request: " + str(exc))
			self.msg.linear.z = thrust
			self.msg.angular.z = yaw

			r.sleep()
		return


		#self.publish_cmd_vel()
	def at_path(self):
		rospy.set_param("wheels/state",0)
		rospy.set_param("wheels/pwm_2",0)
		try:
				self._update_params(["wheels/state"])
				self._update_params(["wheels/pwm_2"])

		except rospy.ServiceException as exc:
					print("Service did not process request: " + str(exc))

		self.d_phi = -(math.pi-self.phi)
		e_phi = self.phi - self.d_phi

		while not e_phi < self.phi_tolerence:
				print "stuck"
				yaw = math.sin(e_phi/2)*600 + 200#+ 100#- 100 #gets sign of yaw (turning direction)
				thrust = 8000 
				e_phi = self.phi - self.d_phi
		
		thrust = 0
		yaw = 0 

		while 1:
			print "at path!"
			try:
				self._update_params(["wheels/state"])
				self._update_params(["wheels/pwm_2"])

			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))


		
	def publish_cmd_vel(self):
		r = rospy.Rate(3)
		while not rospy.is_shutdown():
			#print "published cmd vel!"
			self.pub_cmd.publish(self.msg)
			r.sleep()
		return

if __name__ == "__main__":
	rospy.init_node("crawler",anonymous=True)
	ctrl = CrawlerController()
	rospy.spin()