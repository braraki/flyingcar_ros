#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from crazyflie_teleop.msg import DriveCmd
from crazyflie_driver.srv import UpdateParams
import sys
import threading

class Waypoint():

	def __init__(self, path_file=None):
		rospy.init_node('wheel_command')

		self.path_file = path_file
		self.drive_path = []

		self.msg = DriveCmd()
		#self.msg = Pose2D()

		self.x = 0
		self.y = 0
		
		self.goal_x = 0
		self.goal_y = 0
		self.goal_speed = 0

		self.pos_error = 0.05

		self.goal_pub = rospy.Publisher('goal_point',DriveCmd, queue_size=1)
		#self.goal_pub = rospy.Publisher('goal_point',Pose2D, queue_size=1)
		self._get_drive_path(self.path_file)
		
		pub_thread = threading.Thread(target=self._publish_goal)
		sub_thread = threading.Thread(target=self._listen_to_pos)
		drive_thread = threading.Thread(target=self.auto_drive)

		sub_thread.daemon = True
		drive_thread.daemon = True
		drive_thread.daemon = True

		sub_thread.start()
		pub_thread.start()
		drive_thread.start()

	def _listen_to_pos(self):
		rospy.Subscriber("/Robot_3/ground_pose",Pose2D, self._position_updated)
		return

	def _position_updated(self, data):
		self.x = data.x
		self.y = data.y

	def _get_drive_path(self, path_file): #file format should be x pos,y pos, desired speed. New line for each waypoint. 
		f = open(path_file,'r')
		for line in f:
			point = line.strip().split(',')
			point = [float(i) for i in point]
			self.drive_path.append(point)
		if len(self.drive_path[0]) != 3:
			print "Incorrectly formatted path file! Incorrect number of args!"
			quit()
		f.close()
		print "Got Driving Path!"

	def _change_goal(self):
		if len(self.drive_path) > 0:
			new_goal = self.drive_path.pop(0)
			self.goal_x = new_goal[0]
			self.goal_y = new_goal[1]
			self.goal_speed = new_goal[2]
			# print "self.goal_x is " , self.goal_x
			# print "self.goal_y is " , self.goal_y
		print "Updated Goal!"

	def _publish_goal(self):
		while not rospy.is_shutdown():
			self.msg.x = self.goal_x
			self.msg.y = self.goal_y
			self.msg.speed = self.goal_speed
			#self.msg.theta = self.goal_speed
			# print "self.msg.x is ", self.msg.x
			# print "self.msg.y is ", self.msg.y
			self.goal_pub.publish(self.msg)
			rospy.sleep(0.5)
			#print "Published Goal!"
			#rospy.Rate(30).sleep()
		return

	def auto_drive(self):
		while not rospy.is_shutdown():
			#print "current goal: ", self.goal_x,self.goal_y, self.goal_z
			if self.check_goal():
				print "At goal!"
				self._change_goal()
			rospy.sleep(.5) #NTS are these rates set somewhere/ Check!
		return

	def check_goal(self):
		if abs(self.x - self.goal_x) < self.pos_error and abs(self.y - self.goal_y) < self.pos_error:
			return True
		else:
			return False


if __name__ == '__main__':
	drive_path = sys.argv[1]

	waypoint_drive = Waypoint(drive_path)

	while not rospy.is_shutdown():
		rospy.spin()


# def waypoint():
# 	pub = rospy.Publisher('goal_point', Pose2D, queue_size=10)
# 	# r = rospy.Rate(10)
# 	msg = Pose2D()
# 	msg.x = 1 # make this modifiable
# 	msg.y = 1
# 	msg.theta = 0

# 	while not rospy.is_shutdown():
# 		#rospy.loginfo(msg)
# 		pub.publish(msg)
# 		rospy.sleep(1)
# 		#rospy.Rate(30)

# if __name__ == '__main__':
# 	rospy.init_node('wheel_command')
# 	waypoint()
