#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import math

class Spedometer():

	def __init__(self):
		self.v = 0
		self.speed = 0

		self.x = 0
		self.y = 0

		self.prev_update_time = 0

		#rospy.Subscriber('eff/pose_localization', Odometry, self.find_speed)
		#rospy.Subscriber('eff/battery', Float32, self.find_voltage)
		rospy.Subscriber("Robot_1/pose", PoseStamped, self._position_updated)

		self.speed_pub = rospy.Publisher('speed',Float32, queue_size=1)

	def find_speed(self, data):
		while not rospy.is_shutdown():
			x = data.twist.twist.linear.x
			y = data.twist.twist.linear.y
			self.speed = math.sqrt(x**2 + y**2) 
			msg =  math.sqrt(x**2 + y**2)
			#print self.speed
			#print x, y
			self.speed_pub.publish(msg)
			rospy.Rate(10)

	def find_voltage(self, data):
		self.v = data
		#print self.v

	def _position_updated(self,data): #gonna wanna adjust for decimal places?
		#print "position updated!"
		
		prev_x = self.x
		prev_y = self.y

		self.x = data.pose.position.x
		self.y = data.pose.position.y

		update_time = data.header.stamp.nsecs

		velocity = math.sqrt((self.x-prev_x)**2 + (self.y-prev_y)**2)/(update_time-self.prev_update_time) * 10**9 

		print velocity

		self.prev_update_time = update_time

if __name__ == '__main__':
	rospy.init_node('measure_speed', anonymous=True)
	speedy = Spedometer()
	rospy.spin()