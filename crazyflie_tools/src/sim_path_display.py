#!/usr/bin/env python

import rospy
import tf

import sys
#---------
import numpy as np
from geometry_msgs.msg import PoseStamped
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
				self.path = []
				nodes = data.path
				for i in range(len(nodes)):
					self.path.append(self.nodes_map[nodes[i]][0]) #gives waypoint coordinates + time

def map_maker_client(): #planner stuff
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

class PathDisplay():

	def __init__(self,in_topic,out_topic,cf_num):

		self.cf_num = cf_num

		self.node_map = map_maker_client()

		self.mkpath = MakePath(self.node_map, self.cf_num)

		self.in_topic = in_topic
		self.out_topic = out_topic

		self.path_msg = Path()
		self.path_msg.header.stamp = rospy.Time.now()
		self.path_msg.header.seq = 0
		self.path_msg.header.frame_id = "/world"

		self.init_pose = PoseStamped()
		self.init_pose.header.seq = 0
		self.init_pose.header.stamp = rospy.Time.now()
		self.init_pose.header.frame_id = "/world"
		self.init_pose.pose.position.x = 0
		self.init_pose.pose.position.y = 0
		self.init_pose.pose.position.z = 0
		
		yaw = 0
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		
		self.init_pose.pose.orientation.x = quaternion[0]
		self.init_pose.pose.orientation.y = quaternion[1]
		self.init_pose.pose.orientation.z = quaternion[2]
		self.init_pose.pose.orientation.w = quaternion[3]

		self.path_msg.poses.append(self.init_pose)

		self.pub = rospy.Publisher(self.out_topic,Path,queue_size=10)
		while not rospy.is_shutdown():
			self.make_path()
			self.pub_path()
			rospy.Rate(30)

	def make_path(self):
		self.path_msg.poses = []
		for coord in self.mkpath.path:
			pose = PoseStamped()
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "/world"
			pose.pose.position.x = coord[0]#/3.0
			pose.pose.position.y = coord[1]#/3.0
			pose.pose.position.z = coord[2]#/2.0
			self.path_msg.poses.append(pose)


	def pub_path(self):
		self.path_msg.header.stamp = rospy.Time.now()
		self.pub.publish(self.path_msg)

if __name__ == '__main__':
	rospy.init_node("sim_path_display")
	cf_num = sys.argv[1]
	display_pose = PathDisplay("null","d_path",cf_num)
	

