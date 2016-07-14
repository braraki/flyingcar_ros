#!/usr/bin/env python

#NTS NEED TO ADD SUPPORT FOR MODE 4

import rospy
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

import sys 
import os
import threading

'''
four modes:

(1) Go to next position when usr clicks/presses button
(2) Go to next position as soon as stabilized at last position
(3) Go to next position after t seconds of stabilization at last position
(4) direct input [TO BE IMPLEMENTED]
'''


class WaypointNode:

	def __init__(self, flight_mode=1, path_file=None):
		rospy.init_node("waypoint_nav")

		# gets langing service
		rospy.loginfo("waiting for land service")
		rospy.wait_for_service('land')
		rospy.loginfo("found land service")
		self._land = rospy.ServiceProxy('land', Empty)


		self.flight_mode = flight_mode
		self.path_file = path_file

		self.flight_path = []

		self.repeat = True #NTS should be user-settable

		#Checks if flight path file exists for modes (1,2,3)
		if flight_mode != 4 and self.path_file == None:
			print "No flight path file!"
			quit()

		#-----------------------------------

		#Vars for determining whether CF has reached its goal
		self.x = 0
		self.y = 0
		self.z = 0

		self.dx = 0
		self.dy = 0
		self.dz = 0

		self.prev_update_time = 0

		self.pos_error_margin = 0.14 #errors between goal and current position/velocity for "good enough" position reaching
		self.vel_error_margin = 0.005

		#-----------------------------------

		#CF's Current goal
		self.goal_x = 0
		self.goal_y = 0 
		self.goal_z = 0
		self.goal_pause = 0

		#-----------------------------------

		#defines goal message
		self.msg = PoseStamped()
		self.msg.header.seq = 0
		self.msg.header.stamp = rospy.Time.now()
		self.msg.header.frame_id = "/world"
		self.msg.pose.position.x = self.x
		self.msg.pose.position.y = self.y
		self.msg.pose.position.z = self.z
		yaw = 0
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		self.msg.pose.orientation.x = quaternion[0]
		self.msg.pose.orientation.y = quaternion[1]
		self.msg.pose.orientation.z = quaternion[2]
		self.msg.pose.orientation.w = quaternion[3] #NTS should make yaw settable from the flight path (optionally?)

		#-----------------------------------


		self. goal_pub = rospy.Publisher("goal",PoseStamped, queue_size=1)

		if self.flight_mode != 4:
			self._get_flight_path(self.path_file)

		pub_thread = threading.Thread(target=self._publish_goal)
		sub_thread = threading.Thread(target=self._listen_to_pos)
		flight_thread = threading.Thread(target=self.auto_flight)

		sub_thread.daemon = True
		flight_thread.daemon = True
		pub_thread.daemon = True

		sub_thread.start()
		pub_thread.start()
		flight_thread.start()


	def _listen_to_pos(self):
		rospy.Subscriber("pose",PoseWithCovarianceStamped, self._position_updated)
		return

	def _position_updated(self,data): #gonna wanna adjust for decimal places?
		#print "position updated!"
		prev_x = self.x
		prev_y = self.y
		prev_z = self.z

		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.z = data.pose.pose.position.z

		update_time = data.header.stamp.nsecs

		self.dx = (self.x-prev_x)/(update_time-self.prev_update_time)
		self.dy = (self.y-prev_y)/(update_time-self.prev_update_time)
		self.dz = (self.z-prev_z)/(update_time-self.prev_update_time)
		#print "updated at ", update_time
		#print self.dx,self.dy,self.dz

		self.prev_update_time = update_time

	def _get_flight_path(self, path_file): #file format should be x pos,y pos,z pos(,wait time). New line for each waypoint. 
		f = open(path_file,'r')
		for line in f:
			point = line.strip().split(',')
			point = [float(i) for i in point]
			self.flight_path.append(point)
		if flight_mode == 3 and len(self.flight_path[0]) != 4:
			print "Incorrectly formatted path file! Flight mode 3 requires 4 args!"
			quit()
		elif len(self.flight_path[0]) < 3 or len(self.flight_path[0]) > 4:
			print "Incorrectly formatted path file! Incorrect number of args!"
			quit()
		f.close()
		print "Got Flight Path!"

	def _change_goal(self):
		if len(self.flight_path) > 0:
			new_goal = self.flight_path.pop(0)
			self.goal_x = new_goal[0]
			self.goal_y = new_goal[1]
			self.goal_z = new_goal[2]
			if len(new_goal) > 3:
				self.goal_pause = new_goal[3]
		else: #Reloads flight file if repeat is True, else lands. 
			if self.repeat == True:
				self._get_flight_path(self.path_file)
			else:
				self._land()
				print "finished!"
				return
		print "Updated Goal!"
		
	def _publish_goal(self):
		while not rospy.is_shutdown():
			self.msg.header.stamp = rospy.Time.now()
			self.msg.pose.position.x = self.goal_x
			self.msg.pose.position.y = self.goal_y
			self.msg.pose.position.z = self.goal_z
			self.goal_pub.publish(self.msg)
			#print "Published Goal!"
			rospy.Rate(30).sleep()
		return

	def auto_flight(self):
		while not rospy.is_shutdown():
			#print "current goal: ", self.goal_x,self.goal_y, self.goal_z
			if self.flight_mode == 1:
				raw_input()
				self._change_goal()
			if self._check_goal() == True:
				print "At goal!"
				if self.flight_mode == 2:
					self._change_goal()
				elif self.flight_mode == 3:
					rospy.sleep(float(self.goal_pause))
					self._change_goal()
			rospy.Rate(30).sleep() #NTS are these rates set somewhere/ Check!
		return

	def _check_goal(self):
		#print self.goal_x, self.goal_y, self.goal_z
		if (abs(self.x - self.goal_x) < self.pos_error_margin and abs(self.y - self.goal_y) < self.pos_error_margin 
		  		and abs(self.z - self.goal_z) < self.pos_error_margin and abs(self.dx) < self.vel_error_margin 
		  		and abs(self.dy) < self.vel_error_margin and abs(self.dz) < self.vel_error_margin):
			return True
		else:
			return False

if __name__ == '__main__':

	flight_mode = int(sys.argv[1]) #grabs sys args for mode and flight path
	flight_path = sys.argv[2]

	#print flight_mode, flight_path

	waypointer = WaypointNode(flight_mode,flight_path)

	while not rospy.is_shutdown():
		rospy.spin()
