#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def waypoint():
	pub = rospy.Publisher('/goal_point', Pose2D, queue_size=10)
	# r = rospy.Rate(10)
	msg = Pose2D()
	msg.x = 0 # make this modifiable
	msg.y = .2
	msg.theta = 0

	while not rospy.is_shutdown():
		#rospy.loginfo(msg)
		pub.publish(msg)
		rospy.sleep(0.3)
		#rospy.Rate(30)

if __name__ == '__main__':
	try:
		rospy.init_node('wheel_command', anonymous=False)
		waypoint()
	except rospy.ROSInterruptException: pass