#!/usr/bin/env python

import rospy
import numpy as np

from time import sleep
import time, threading

from std_msgs.msg import Empty
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry

import std_msgs.msg
import tf
from numpy.linalg import inv
import math

from crazyflie_teleop import kalmanr3

rate = 30.0

class LocalizationNode:

	def __init__(self, name, number):
		# statemachine constants
		self.name = name
		self.number = number

		self.odom_pub = rospy.Publisher("pose_localization",Odometry,queue_size=10)
		self.pose_pub = rospy.Publisher("pose",PoseStamped,queue_size=10)
		self.tf = tf.TransformListener()

		self.frame = "/vicon/CF" + self.number + "/CF" + self.number

		dt = 1.0/rate

		self.kr3 = kalmanr3.KalmanR3(dt)

	def process_mocap_data(self):
		r = rospy.Rate(rate)
		while not rospy.is_shutdown():
			if self.tf.frameExists(self.frame) and self.tf.frameExists("/world"):
				t = self.tf.getLatestCommonTime(self.frame, "/world")
				z, q = self.tf.lookupTransform("/world", self.frame, t)
				#yawtest = tf.transformations.euler_from_quaternion(q)[2]
				self.kr3.step(z)
				#yaw = self.getYaw(q)
				
				#print yaw, yawtest
				#print roll_test pitch_test yawtest
				odom = Odometry()
				odom.header.stamp = rospy.Time.now()
				odom.header.frame_id = "/world"
				odom.child_frame_id = "/" + self.name + "/base_link"

				#quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
				posecov = PoseWithCovariance()
				pose = Pose()
				pose.position.x = self.kr3.pos[0]
				pose.position.y = self.kr3.pos[1]
				pose.position.z = self.kr3.pos[2]
				pose.orientation.x = q[0]
				pose.orientation.y = q[1]
				pose.orientation.z = q[2]
				pose.orientation.w = q[3]
				posecov.pose = pose
				posecov.covariance = [0] * 36
				odom.pose = posecov
				twistcov = TwistWithCovariance()
				twist = Twist()
				twist.linear.x = self.kr3.vel[0]
				twist.linear.y = self.kr3.vel[1]
				twist.linear.z = self.kr3.vel[2]
				twist.angular.x = 0.0
				twist.angular.y = 0.0
				twist.angular.z = 0.0
				twistcov.twist = twist
				twistcov.covariance = [0] * 36
				odom.twist = twistcov
				self.odom_pub.publish(odom)

				poseStamped = PoseStamped()
				poseStamped.pose = pose
				poseStamped.header = odom.header
				poseStamped.header.frame_id = "/" + self.name + "/base_link"
				self.pose_pub.publish(poseStamped)

				r.sleep()

	def getYaw(self,q):
		R = np.array([[q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2, 2 * (q[0] * q[1] + q[2]  * q[3]), 2 * (q[0] * q[2]  - q[1] * q[3])],
					[2 * (q[0] * q[1] - q[2]  * q[3]), -q[0] ** 2 + q[1] ** 2 - q[2]  ** 2 + q[3] ** 2, 2 * (q[1] * q[2]  + q[0] * q[3])],
					[2 * (q[0] * q[2]  + q[1] * q[3]), 2 * (q[1] * q[2]  - q[0] * q[3]), -q[0] ** 2 - q[1] ** 2 + q[2]  ** 2 + q[3] ** 2]])
		yaw = np.arctan2(R[1,2],R[1,1])
		print yaw
		return yaw


if __name__ == '__main__':
	rospy.init_node("localization")
	
	name = sys.argv[1]
	number = sys.argv[2]
	
	localization = LocalizationNode(name, number)
	localization.process_mocap_data()
	
	rospy.spin()