#!/usr/bin/env python

#Node for waypoint following with wheels and flight
#author: Sarah Pohorecky - spohorec@mit.edu

#ARGUMENTS: cf_num (corresponding simulated Crazyflie number)

#------------------------------------

import rospy
import tf

import sys 
import threading
import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

from std_srvs.srv import Empty

#---------

from crazyflie_driver.srv import UpdateParams

from planner.msg import HiPathTime
from crazyflie_teleop.msg import DriveCmd

from map_maker import map_maker_helper as map_helper

#---------

#Gets path from planner
class MakePath: 
	def __init__(self,nodes_map,cf_num):
		print "Initializing converter..."
		rospy.Subscriber("/si_planner/time_path_topic",HiPathTime,self._convert_path)
		self.nodes_map = nodes_map
		self.cf_num = cf_num
		self.path = []
		print "Initialized..."

	def _convert_path(self,data):
		if int(data.ID) == int(self.cf_num):
			path = []
			nodes = data.path
			times = data.times
			for i in range(len(nodes)):
				path.append((self.nodes_map[nodes[i]][0],times[i],self.nodes_map[nodes[i]][1])) #gets waypoint coordinates + time + waypoint type		
			self.path = path

class WaypointNode:

	def __init__(self,cf_num):

		self.cf_num = cf_num
		self.cf_path = []

		#-----------------------------------

		#update_param service
		rospy.wait_for_service('update_params')
		rospy.loginfo("found update_params service")
		self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

		#e-stop service (currently un-implemented in this node, but would be a good thing to add)
		rospy.loginfo("waiting for emergency service")
		rospy.wait_for_service('emergency')
		rospy.loginfo("found emergency service")
		self._emergency = rospy.ServiceProxy('emergency', Empty)

		#land service
		rospy.loginfo("waiting for land service")
		rospy.wait_for_service('land')
		rospy.loginfo("found land service")
		self._land = rospy.ServiceProxy('land', Empty)

		#takeoff service
		rospy.loginfo("waiting for takeoff service")
		rospy.wait_for_service('takeoff')
		rospy.loginfo("found takeoff service")
		self._takeoff = rospy.ServiceProxy('takeoff', Empty)

		#-----------------------------------

		self.in_air = False
		rospy.set_param('in_air',self.in_air) #param that lets wheel controller know if the Crazyflie is in the air

		#-----------------------------------

		#CF's Current goal
		self.goal_index = 0
		self.goal_x = 0
		self.goal_y = 0 
		self.goal_z = 0
		self.goal_t = 0
		self.goal_type = 'None'

		self.next_goal_x = 0
		self.next_goal_y = 0
		self.next_goal_z = 0
		self.next_goal_t = 0

		#-----------------------------------

		#defines goal message
		self.flight_msg = PoseStamped()
		self.flight_msg.header.seq = 0
		self.flight_msg.header.stamp = rospy.Time.now()
		self.flight_msg.header.frame_id = "/world"
		self.flight_msg.pose.position.x = 0
		self.flight_msg.pose.position.y = 0
		self.flight_msg.pose.position.z = 0
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
		self.flight_msg.pose.orientation.x = quaternion[0]
		self.flight_msg.pose.orientation.y = quaternion[1]
		self.flight_msg.pose.orientation.z = quaternion[2]
		self.flight_msg.pose.orientation.w = quaternion[3]

		self.drive_msg = DriveCmd()
		self.drive_msg.x = 0
		self.drive_msg.y = 0
		self.drive_msg.t = rospy.Time.now()
		self.drive_msg.x2 = 0
		self.drive_msg.y2 = 0
		self.drive_msg.t2 = rospy.Time.now()

		#-----------------------------------

		#gets node map 
		print "Getting node map..."
		self.node_map = map_helper.map_maker_client('/send_complex_map')[0]
		print "Got node map!"

		#-----------------------------------

		#Publisher for goals
		self.flight_goal_pub = rospy.Publisher("flight_goal",PoseStamped, queue_size=1) #NTS Namespace
		self.drive_goal_pub = rospy.Publisher("drive_goal",DriveCmd, queue_size=1) #NTS Namespace
		self.drive_next_goal_pub = rospy.Publisher("drive_next_goal",DriveCmd, queue_size=1) #NTS Namespace

		#-----------------------------------

		#Threading
		self.goal_lock = threading.RLock() 	#this lock used to control access to self.goal_index
										#prevents indexing problems caused by conflicting accesses 
										#from _path_generator and _change goal.
		
		path_thread = threading.Thread(target=self._path_generator)
		pub_flight_thread = threading.Thread(target=self._publish_flight_goal)
		pub_drive_thread = threading.Thread(target=self._publish_drive_goal)
		nav_thread = threading.Thread(target=self.auto_nav)

		path_thread.daemon = True
		nav_thread.daemon = True
		pub_flight_thread.daemon = True
		pub_drive_thread.daemon = True

		path_thread.start()
		pub_flight_thread.start()
		pub_drive_thread.start()
		nav_thread.start()

	#returns list of waypoints as triples with time,type [((x,y,z),t,ty),...]
	def _path_generator(self): 
		print "Starting path generator..."
		mkpath = MakePath(self.node_map,self.cf_num)
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			path = mkpath.path
			if not path == self.cf_path:
				print "updating path"
				self.goal_lock.acquire()

				self.cf_path = mkpath.path
				self.goal_index = 0
				
				self.goal_lock.release()

			r.sleep()
		return

	def _change_goal(self):
			self.goal_lock.acquire()
			try: #prevents failure in the case of bad syncronization (shouldn't happen anymore, but kept to be safe)
				new_goal = self.cf_path[self.goal_index]
				self.goal_x = new_goal[0][0]
				self.goal_y = new_goal[0][1]
				self.goal_z = new_goal[0][2]
				self.goal_t = new_goal[1]
				self.goal_type = new_goal[2]

				if self.goal_index < len(self.cf_path):
					next_goal = self.cf_path[self.goal_index + 1]
					if not map_helper.is_air(next_goal[2]):
						self.next_goal_x = next_goal[0][0]
						self.next_goal_y = next_goal[0][1]
						self.next_goal_z = next_goal[0][2]
						self.next_goal_t = next_goal[1]
					else:
						self.next_goal_x = self.goal_x
						self.next_goal_y = self.goal_y
						self.next_goal_z = self.goal_z
						self.next_goal_t = self.goal_t
				else:
					self.next_goal_x = self.goal_x
					self.next_goal_y = self.goal_y
					self.next_goal_z = self.goal_z
					self.next_goal_t = self.goal_t

			except:
				pass
			self.goal_index += 1
			self.goal_lock.release()
			return		

	def _publish_flight_goal(self):
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			self.flight_msg.header.stamp = rospy.Time.now()
			self.flight_msg.header.frame_id = "/world"

			self.flight_msg.pose.position.x = self.goal_x
			self.flight_msg.pose.position.y = self.goal_y
			self.flight_msg.pose.position.z = self.goal_z

			self.flight_goal_pub.publish(self.flight_msg)
			r.sleep()
		return

	def _publish_drive_goal(self):
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			self.drive_msg.x = self.goal_x
			self.drive_msg.y = self.goal_y
			self.drive_msg.t = rospy.Time.from_sec(self.goal_t)

			self.drive_msg.x2 = self.next_goal_x
			self.drive_msg.y2 = self.next_goal_y
			self.drive_msg.t2 = rospy.Time.from_sec(self.next_goal_t)

			self.drive_goal_pub.publish(self.drive_msg)

			r.sleep()
		return
		
	#changes waypoint and takes off/lands 
	def auto_nav(self):
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			if not self.in_air and ( map_helper.is_air(self.goal_type) ):
				self.in_air = True										
				rospy.set_param('in_air', self.in_air)
				self._takeoff()
			elif self.in_air and  not (  map_helper.is_air(self.goal_type) ): 
				self.in_air = False
				rospy.set_param('in_air', self.in_air)
				self._land()
			if time.time() >= self.goal_t:
				self._change_goal()
			r.sleep()
		return


if __name__ == '__main__':
	rospy.init_node("waypoint_nav")

	cf_num = sys.argv[1]

	waypointer = WaypointNode(cf_num)

	rospy.spin()
