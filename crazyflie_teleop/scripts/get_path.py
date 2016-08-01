#!/usr/bin/env python

import rospy
import numpy as np
from map_maker.srv import *
from map_maker.msg import HiPath
from nav_msgs.msg import Path
from enum import Enum


class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}

class MakePath:
	def __init__(self,nodes_map):
		rospy.Subscriber("/highlighter/path_topic",HiPath,self._convert_path)
		self.nodes_map = nodes_map
		self.path = []

	def _convert_path(self,data):
		self.path = []
		print "here!"
		nodes = data.path
		for node in nodes:
			self.path.append(self.nodes_map[node][0])
		print self.path


def map_maker_client():
	rospy.wait_for_service('send_map')
	try:
		print('calling')
		global info_dict
		func = rospy.ServiceProxy('send_map', MapTalk)
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
			x = (x_list[ID])/2000.0
			y = (y_list[ID])/2000.0
			z = (z_list[ID])/2000.0
			c = static_category_dict[resp.category_list[ID]]
			info_dict[ID] = ((x, y, z),c)
		#s = full_system(info_dict, A)
		#fs.runner()
	except rospy.ServiceException, e:
		print("service call failed")
	return info_dict

if __name__ == '__main__':
	rospy.init_node("path_grabber")
	nodes = map_maker_client()
	mkpath = MakePath(nodes)
	rospy.spin()