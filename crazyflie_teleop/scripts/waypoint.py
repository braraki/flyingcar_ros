#!/usr/bin/env python

#NTS NEED TO ADD SUPPORT FOR MODE 4

import rospy
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty

import sys 
import os
import threading
import time

#---------
import numpy as np
from map_maker.srv import *
from map_maker.msg import HiPathTime
from nav_msgs.msg import Path
from enum import Enum
#---------

class Category(Enum): #stuff for Jack's planner
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}


class MakePath: #mkpath.path = current path for CF
	def __init__(self,nodes_map,cf_num):
		print "Initializing converter..."
		rospy.Subscriber("/si_planner/time_path_topic",HiPathTime,self._convert_path)
		self.nodes_map = nodes_map
		self.cf_num = cf_num
		self.path = []
		print "Initialized..."

	def _convert_path(self,data):
		#print "Converting path..."
		if int(data.ID) == int(self.cf_num):
			#print "Converting path!"
			self.path = []
			nodes = data.path
			times = data.times
			for i in range(len(nodes)):
				self.path.append((self.nodes_map[nodes[i]][0],times[i]))#gives waypoint coordinates + time		
		#else:
			#print "Wrong ID! Data ID is: ", data.ID, ". CF_ID is: ", self.cf_num

def map_maker_client(): #gets conversion dictionary 
	print "Waiting for 'send_complex_map' service"
	rospy.wait_for_service('/send_complex_map')
	print "Got map service!"
	try:
		print('calling')
		global info_dict
		func = rospy.ServiceProxy('/send_complex_map', MapTalk)
		resp = func()
		print('recieved')
		x_list = resp.x_list
		y_list = resp.y_list
		z_list = resp.z_list
		num_IDs = resp.num_IDs
		adjacency_array = resp.adjacency_array
		A = np.array(adjacency_array)
		A.shape = (num_IDs, num_IDs)
		info_dict = {}
		for ID in range(num_IDs):
			x = (x_list[ID])
			y = (y_list[ID])
			z = (z_list[ID])
			c = static_category_dict[resp.category_list[ID]]
			info_dict[ID] = ((x, y, z),c)
		#s = full_system(info_dict, A)
		#fs.runner()
	except rospy.ServiceException, e:
		print("service call failed")
	return info_dict

class WaypointNode:

	def __init__(self,cf_num):
		rospy.init_node("waypoint_nav")

		self.cf_num = cf_num

		#-----------------------------------

		self.flight_path = []

		#CF's Current goal
		self.goal_index = 0
		self.goal_x = 0
		self.goal_y = 0 
		self.goal_z = 0
		self.goal_t = 0

		#-----------------------------------

		#defines goal message
		self.msg = PoseStamped()
		self.msg.header.seq = 0
		self.msg.header.stamp = rospy.Time.now()
		self.msg.header.frame_id = "/world"
		self.msg.pose.position.x = 0
		self.msg.pose.position.y = 0
		self.msg.pose.position.z = 0
		yaw = 0
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		self.msg.pose.orientation.x = quaternion[0]
		self.msg.pose.orientation.y = quaternion[1]
		self.msg.pose.orientation.z = quaternion[2]
		self.msg.pose.orientation.w = quaternion[3] #NTS should make yaw settable from the flight path (optionally?)

		#-----------------------------------

		#gets node map 
		print "Getting node map..."
		self.node_map = map_maker_client()
		print "Got node map!"

		#-----------------------------------


		self.goal_pub = rospy.Publisher("goal",PoseStamped, queue_size=1) #NTS Namespace

		#-----------------------------------

		#Threading

		self.lock = threading.RLock()

		path_thread = threading.Thread(target=self._path_generator)
		pub_thread = threading.Thread(target=self._publish_goal)
		flight_thread = threading.Thread(target=self.auto_flight)

		path_thread.daemon = True
		flight_thread.daemon = True
		pub_thread.daemon = True

		path_thread.start()
		pub_thread.start()
		flight_thread.start()


	def _path_generator(self): #returns list of waypoints as tuples with time [((x,y,z),t),...]
		print "Starting path maker..."
		mkpath = MakePath(self.node_map,self.cf_num)
		while not rospy.is_shutdown():
			if mkpath.path != self.flight_path:
				self.lock.acquire()
				print "lock acquired by 1!"
				self.flight_path = mkpath.path
				self.goal_index = 0
				self.lock.release()
				print "lock released by 1!"
			rospy.Rate(1)

	def _change_goal(self):
			print "2 trying to acquire lock..."
			self.lock.acquire()
			print "lock acquired by 2!"
			try:
				new_goal = self.flight_path[self.goal_index]
				self.goal_x = new_goal[0][0]#/3.0
				self.goal_y = new_goal[0][1]#/3.0
				self.goal_z = new_goal[0][2]#/2.0
				self.goal_t = new_goal[1]
			except:
				pass
			self.goal_index += 1
			self.lock.release()
			print "lock released by 2!"
			return
			# print self.goal_x, self.goal_y, self.goal_z
		

	def _publish_goal(self):
		while not rospy.is_shutdown():
			self.msg.header.stamp = rospy.Time.now()
			self.msg.header.frame_id = "/world"
			self.msg.pose.position.x = self.goal_x
			self.msg.pose.position.y = self.goal_y
			self.msg.pose.position.z = self.goal_z

			self.goal_pub.publish(self.msg)
			# print "Published Goal!"
			rospy.Rate(30)
		return

	def auto_flight(self):
		while not rospy.is_shutdown():
			if time.time() >= self.goal_t:
				# print "Changing Goal!"
				self._change_goal()
			rospy.Rate(30) #NTS are these rates set somewhere? Check!
		return


if __name__ == '__main__':

	#print flight_mode, flight_path

	cf_num = sys.argv[1]

	waypointer = WaypointNode(cf_num)

	while not rospy.is_shutdown():
		rospy.spin()
