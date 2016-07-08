#!/usr/bin/env python

#NEED TO ADD SUPPORT FOR MODE 4

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys 
import os

'''
four modes:

(1) Go to next position when usr clicks/presses button
(2) Go to next position as soon as stabilized at last position
(3) Go to next position after t seconds of stabilization at last position
(4) direct input
'''
#needs to publish to goal
#needs to subscribe to current position.

class WaypointCtrl:

	def __init__(self, flight_mode=1, path_file=None):

		self.flight_mode = flight_mode
		self.flight_path = []

		self.path_file = path_file
		if flight_mode != 4 and self.path_file == None:
			print "Unacceptable flight path file!"
			quit()

		self.x = 0
		self.y = 0
		self.z = 0

		self.last_update_time = 0

		self.dx = 0
		self.dy = 0
		self.dz = 0

		self.goal_x = 0
		self.goal_y = 0 
		self.goal_z = 0
		self.goal_pause = 0

		self.repeat = True #should be settable

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

		self.prev_update_time = 1

		self.pos_error_margin = 0.05
		self.vel_error_margin = 0.005

		rospy.Subscriber("/kal/pose",PoseWithCovarianceStamped, self._position_updated)	
		self. goal_pub = rospy.Publisher("/kal/goal",PoseStamped, queue_size=1)

		if self.flight_mode != 4:
			self._get_flight_path(self.path_file)


	def _position_updated(self,data): #gonna wanna adjust for decimal places?
		prev_x = self.x
		prev_y = self.y
		prev_z = self.z

		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.z = data.pose.pose.position.z

		update_time = data.header.stamp.nsecs

		#print self.prev_update_time

		self.dx = (self.x-prev_x)/(update_time-self.prev_update_time)
		self.dy = (self.y-prev_y)/(update_time-self.prev_update_time)
		self.dz = (self.z-prev_z)/(update_time-self.prev_update_time)
		#print "updated at ", update_time

		self.prev_update_time = update_time


	def _get_flight_path(self, path_file):
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

	def _change_goal(self):
		if len(self.flight_path) > 0:
			new_goal = self.flight_path.pop(0)
			self.goal_x = new_goal[0]
			self.goal_y = new_goal[1]
			self.goal_z = new_goal[2]
			if len(new_goal) > 3:
				self.goal_pause = new_goal[3]
		else:
			if self.repeat == True:
				self._get_flight_path(self.path_file)
			else:
				#land and shutdown? Signal landing
				print "finished!"
		self._publish_goal()
		
	def _publish_goal(self):
		print self.dx, self.dy, self.dz
		self.msg.header.stamp = rospy.Time.now()
		self.msg.pose.position.x = self.goal_x
		self.msg.pose.position.y = self.goal_y
		self.msg.pose.position.z = self.goal_z
		self.goal_pub.publish(self.msg)

	def auto_flight(self):
		print "current goal: ", self.goal_x,self.goal_y, self.goal_z
		if self.flight_mode == 1:
			raw_input()
			self._change_goal()
		if self._check_goal() == True:
			if self.flight_mode == 2:
				self._change_goal()
			elif self.flight_mode == 3:
				rospy.sleep(float(self.goal_pause))
				self._change_goal()

	def _check_goal(self):
		if (abs(self.x - self.goal_x) < self.pos_error_margin and abs(self.y - self.goal_y) < self.pos_error_margin 
		  		and abs(self.z - self.goal_z) < self.pos_error_margin and abs(self.dx) < self.vel_error_margin 
		  		and abs(self.dy) < self.vel_error_margin and abs(self.dz) < self.vel_error_margin):
			return True
		else:
			return False

if __name__ == '__main__':
	rospy.init_node("waypoint_nav")

	rate = rospy.Rate(rospy.get_param("/kal/pose/rate")) #NTS this should be reset to ~/pose/rate once node is running in correct NS

	flight_mode = int(sys.argv[1])
	flight_path = sys.argv[2]

	print flight_mode, flight_path

	waypointer = WaypointCtrl(flight_mode,flight_path)

	while not rospy.is_shutdown():
		waypointer.auto_flight()
		waypointer._publish_goal()
		rate.sleep()
