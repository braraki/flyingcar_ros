#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

class Selector():
	def __init__(self):
		self.current_cf = 0
		self.crazyflies = rospy.get_param('cf_list').keys()
		self.relay = rospy.Publisher(current_joy,Joy,queue_size=10)
		rospy.Subscriber('joy', Joy, self.relay_joy_ctrl)


	def relay_joy_ctrl(self,joy_data):
		self.crazyflies = rospy.get_param('cf_list').keys()
		if joy_data.buttons[3] == 1:
			self.current_cf+=1
			if self.current_cf == len(self.crazyflies):
				self.current_cf=0
		current_joy=self.crazyflies[self.current_cf]+'/joy'
		
		self.relay.publish(joy_data)


if __name__ == '__main__':
	rospy.init_node('cf_selector')
	selector = Selector()
	rospy.spin()