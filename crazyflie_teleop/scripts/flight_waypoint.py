#!/usr/bin/env python

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

from map_maker import gen_adj_array_info_dict as infoDict
#---------

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
				self.path.append((self.nodes_map[nodes[i]][0],times[i],self.nodes_map[nodes[i]][1])) #gives waypoint coordinates + time + waypoint type		

class WaypointNode:

	def __init__(self,cf_num):
		rospy.init_node("waypoint_nav")

		rospy.loginfo("waiting for emergency service")
		rospy.wait_for_service('emergency')
		rospy.loginfo("found emergency service")
		self._emergency = rospy.ServiceProxy('emergency', Empty)

		rospy.loginfo("waiting for land service")
		rospy.wait_for_service('land')
		rospy.loginfo("found land service")
		self._land = rospy.ServiceProxy('land', Empty)

		rospy.loginfo("waiting for takeoff service")
		rospy.wait_for_service('takeoff')
		rospy.loginfo("found takeoff service")
		self._takeoff = rospy.ServiceProxy('takeoff', Empty)

		self.in_air = False

		self.cf_num = cf_num
		self.flight_path = []

		#-----------------------------------

		#CF's Current goal
		self.goal_index = 0
		self.goal_x = 0
		self.goal_y = 0 
		self.goal_z = 0
		self.goal_t = 0
		self.goal_type = 'None'

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
		self.msg.pose.orientation.w = quaternion[3] 

		#-----------------------------------

		#gets node map 
		print "Getting node map..."
		self.node_map = infoDict.map_maker_client('/send_complex_map')[0]
		print "Got node map!"

		#-----------------------------------

		#Publisher for goal
		self.goal_pub = rospy.Publisher("goal",PoseStamped, queue_size=1) #NTS Namespace

		#-----------------------------------

		#Threading
		self.goal_lock = threading.RLock() 	#this lock used to control access to self.goal_index
										#prevents indexing problems caused by conflicting accesses 
										#from _path_generator and _change goal.
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
		print "Starting path generator..."
		mkpath = MakePath(self.node_map,self.cf_num)
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			if mkpath.path != self.flight_path:
				self.goal_lock.acquire()

				self.flight_path = mkpath.path
				self.goal_index = 0
				
				self.goal_lock.release()
			r.sleep()
		return

	def _change_goal(self):
			self.goal_lock.acquire()
			try: #prevents failure in the case of bad syncronization (if path hasn't updated but goal_index has gone beyond end-of-list)
				new_goal = self.flight_path[self.goal_index]
				self.goal_x = new_goal[0][0]
				self.goal_y = new_goal[0][1]
				self.goal_z = new_goal[0][2]
				self.goal_t = new_goal[1]
				self.goal_type = new_goal[2]
			except:
				pass
			self.goal_index += 1
			self.goal_lock.release()
			return		

	def _publish_goal(self):
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			self.msg.header.stamp = rospy.Time.now()
			self.msg.header.frame_id = "/world"

			self.msg.pose.position.x = self.goal_x
			self.msg.pose.position.y = self.goal_y
			self.msg.pose.position.z = self.goal_z

			self.goal_pub.publish(self.msg)
			r.sleep()
		return

	def auto_flight(self):
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			#print "In Air: ",self.in_air
			if not self.in_air and ( self.goal_type == 3 or self.goal_type == 4 ): # self.goal_z != 0:
				self._takeoff()
				#print "Takeoff requested!"
				self.in_air = True
			elif self.in_air and  not ( self.goal_type == 3 or self.goal_type == 4 ): # self.goal_z == 0:
				self._land()
				#print "Landing requested!"
				self.in_air = False
			if time.time() >= self.goal_t:
				self._change_goal()
			r.sleep()
		return


if __name__ == '__main__':

	cf_num = sys.argv[1] #pass cf_num as argument to script 

	waypointer = WaypointNode(cf_num)

	while not rospy.is_shutdown():
		rospy.spin()
