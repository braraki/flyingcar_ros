#!/usr/bin/env python

import rospy
import math
import numpy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3

import threading 

class CrawlerController:

	def __init__(self):
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
		self.phi_tolerence = 0.1
		self.pos_tolerence = 0.05

		#msg def
		self.msg = Twist()
		self.msg.linear = Vector3(0,0,0)
		self.msg.angular = Vector3(0,0,0)

		self.yaw_rate = 100 #cmd_vel yaw rate. 200 is max system will take from PS3 controller

		print "starting up!"

		self.pub_cmd = rospy.Publisher("/cap/cmd_vel",Twist,queue_size=10)

		self.update = True

		pos_thread	= threading.Thread(target=self._sub_pos)
		cmd_thread  = threading.Thread(target=self.publish_cmd_vel)
		ctrl_thread = threading.Thread(target=self.controller)

		pos_thread.daemon = True
		cmd_thread.daemon = True 
		ctrl_thread.daemon = True

		pos_thread.start()
		cmd_thread.start()
		ctrl_thread.start()

	def _sub_pos(self):
		rospy.Subscriber("/Robot_2/ground_pose",Pose2D,self._get_pos)
		return


	def _get_pos(self,data):
		self.x = data.x
		self.y = data.y
		self.phi = data.theta#-math.pi/4

	def controller(self):
		while not rospy.is_shutdown():
				self.d_phi = -math.atan((self.d_y-self.y)/(self.d_x-self.x))


				e_phi = self.phi - self.d_phi
				e_x = self.x - self.d_x
				e_y = self.y - self.d_y

				#print self.x, self.y, self.phi, e_phi	

				if not (abs(e_x) <= self.pos_tolerence and abs(e_y) <= self.pos_tolerence):
					#angle error correction
					if not abs(e_phi) <= self.phi_tolerence:
						print "angle and position WRONG! Phi error: ", e_phi
						if e_phi > 0:
							yaw = -100 #signed!
						else:
							yaw = 100
						thrust = 28000
					else:
						yaw = 0
						print "position WRONG! Phi Error: ", e_phi
						thrust = 30000				

					#adjust phi
					#adjust thrust

				else:
					"-----GOOD TO GO-----"
					thrust = 0
					yaw = 0

				# if not (abs(e_phi) < self.phi_tolerence):
				# 	u = -self.k_phi*math.sin(e_phi) #+ self.d_phi_dot

				# 	thrust_range = (25000,25000)

				# 	yaw = numpy.sign(e_phi)*100#25000#-u/abs(u)*self.yaw_rate #gets sign of yaw (turning direction)
				# 	thrust = 30000#thrust_range[0] + abs(u)/self.k_phi*(thrust_range[1]-thrust_range[0]) #rescales thrust from 0-k_phi to thrust_range
				# #else:
				# 	#yaw = 0

				# elif not (abs(e_x) < self.pos_tolerence and abs(e_y) < self.pos_tolerence):
				# 	yaw = 0
				# 	thrust = 30000
				# 	#wheel_speed = self.k_pos*(e_pos)

				# else:
				# 	yaw = 0
				# 	thrust = 0

				self.msg.linear.z = thrust
				self.msg.angular.z = yaw

				rospy.Rate(1)
		return

	def publish_cmd_vel(self):
		while not rospy.is_shutdown():
			self.pub_cmd.publish(self.msg)
			rospy.Rate(1) #try a different rate?
		return

if __name__ == "__main__":
	rospy.init_node("crawler",anonymous=True)
	ctrl = CrawlerController()
	rospy.spin()




