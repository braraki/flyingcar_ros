#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def waypoint():
	pub = rospy.Publisher('goal_point', Pose2D, queue_size=10)
	rospy.init_node('wheel_command', anonymous=True)
	r = rospy.Rate(10)
	msg = Pose2D()
	msg.x = 10 # make this modifiable
	msg.y = 10
	msg.theta = 0

	while not rospy.is_shutdown():
		rospy.loginfo(msg)
		pub.publish(msg)
		r.sleep()

if __name__ == '__main__':
	try:
		waypoint()
	except ros.ROSInteruptException: pass