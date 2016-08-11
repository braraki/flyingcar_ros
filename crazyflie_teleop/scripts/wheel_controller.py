#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from crazyflie_teleop.msg import DriveCmd
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
import math
import sys
import tf
import threading
import time

class wheel_controller:

	def __init__(self, cf_num):
		rospy.wait_for_service('update_params')
		rospy.loginfo("found update_params service")
		self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

		self.cf_num = cf_num

		self.x = 0
		self.y = 0
		self.theta = 0

		# speed
		self.prev_speeds = []
		self.speed = 0
		self.prev_update_time = 0

		# goals
		self.goal_x = 0
		self.goal_y = 0
		self.goal_speed = 0
		self.goal_t = 0

		self.prev_goal_x = 0
		self.prev_goal_y = 0
		self.prev_goal_t = 0

		# controls
		self.baseline = 0
		self.theta_offset = 0
		self.speed_offset = 0

		self.theta_heading = 0
		self.theta_error = 0

		self.theta_offset_constant = 0.5
		self.speed_offset_constant = 10

		# margin of error
		self.pos_error = 0.03

		rospy.Subscriber("drive_goal", DriveCmd, self.update_goal)
		rospy.Subscriber("pose", PoseStamped, self.update_position)

		ctrl_thread = threading.Thread(target=self.wheel_command)
		ctrl_thread.daemon = True
		ctrl_thread.start()

	def update_goal(self, data):
		if self.goal_x != data.x or self.goal_y != data.y or self.goal_t != float(data.t.secs+data.t.nsecs*10**(-9)):
			#print "Goal has changed!"
			self.prev_goal_x = self.goal_x
			self.prev_goal_y = self.goal_y
			self.prev_goal_t = self.goal_t
		self.goal_x = data.x
		self.goal_y = data.y
		self.goal_t = float(data.t.secs+data.t.nsecs*10**(-9))

		#print data.x, self.goal_x, data.y, self.goal_y


		try:
			self.goal_speed = math.sqrt((self.goal_x-self.prev_goal_x)**2+(self.goal_y-self.prev_goal_y)**2)/float(self.goal_t-self.prev_goal_t)
			#self.goal_speed = math.sqrt((self.goal_x-self.x)**2+(self.goal_y-self.y)**2)/float(self.goal_t-self.prev_goal_t) * 10**9 #time.time())
		except:
			pass
		#print self.goal_speed
		#print float(self.goal_t-rospy.Time.now().nsecs)
		#print self.goal_t, self.prev_goal_t

	def update_position(self,data):
		if self.x != data.pose.position.x or self.y != data.pose.position.y:
			prev_x = self.x
			prev_y = self.y

			self.x = data.pose.position.x
			self.y = data.pose.position.y
			q = (data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w)
			self.theta = tf.transformations.euler_from_quaternion(q)[2]

			update_time = data.header.stamp.nsecs

			speed = math.sqrt((self.x-prev_x)**2 + (self.y-prev_y)**2)/(update_time-self.prev_update_time) * 10**9

			if len(self.prev_speeds) == 10: #NTS Potential threading issue
				del self.prev_speeds[0]

			self.prev_speeds.append(speed)
			self.prev_update_time = update_time

	def check_goal(self):
		if abs(self.x - self.goal_x) < self.pos_error and abs(self.y - self.goal_y) < self.pos_error:
			return True
		else:
			return False

	def wheel_command(self): #NTS could this be having bad timing interactions? It's completely possible that the goal updates during a runthrough.
		while not rospy.is_shutdown():	#NTS if goals and positions/angles change while running, this could affect calculation? Add a lock to be safe?
			# print "entering commander!"	# NTS A lock might also mess up the subscribers because of lag?
			# calculate baseline
			self.baseline = (self.goal_speed + 0.0092033)/0.00084351
			self.baseline = min(255, self.baseline)
			self.baseline = max(0, self.baseline)
			self.baseline = int(self.baseline)

			# calculate theta offset
			x_e = self.goal_x - self.x
			y_e = self.goal_y - self.y

			if x_e == 0:
				if y_e > 0:
					self.theta_heading = math.pi/2
				else:
					self.theta_heading = -math.pi/2
			else:
				self.theta_heading = math.atan2(y_e, x_e)


			self.theta_error = self.theta_heading - self.theta
			self.theta_offset = self.theta_error/math.pi * 170 * self.theta_offset_constant
			self.theta_offset = min(self.theta_offset, 85)
			self.theta_offset = max(self.theta_offset, -85)
			self.theta_offset = int(self.theta_offset)

			# calculate speed offset
			self.speed = float(sum(self.prev_speeds))/len(self.prev_speeds) if len (self.prev_speeds) > 0 else 0
			#print self.speed, self.goal_speed
			speed_error = self.goal_speed - self.speed

			self.speed_offset = speed_error * self.speed_offset_constant
			self.speed_offset = int(self.speed_offset)

			# rospy.set_param('wheels/state', 1) #NTS might need to mess with this
			# rospy.set_param('wheels/pwm_1', max(0,min(self.baseline + self.theta_offset + self.speed_offset, 255)))
			# rospy.set_param('wheels/pwm_2', max(0,min(self.baseline - self.theta_offset + self.speed_offset, 255)))

			# print max(0,min(self.baseline + self.theta_offset + self.speed_offset, 255)), max(0,min(self.baseline - self.theta_offset + self.speed_offset, 255))
			
			if not rospy.get_param('in_air'):
				if self.check_goal():
					rospy.set_param('wheels/state', 0)
					#print "state set to 0."
				else:
					rospy.set_param('wheels/state', 1) #NTS might need to mess with this
					rospy.set_param('wheels/pwm_1', max(0,min(self.baseline + self.theta_offset + self.speed_offset, 255)))
					rospy.set_param('wheels/pwm_2', max(0,min(self.baseline - self.theta_offset + self.speed_offset, 255)))
					# print "parameters updated!"
					print max(0,min(self.baseline + self.theta_offset + self.speed_offset, 255)), max(0,min(self.baseline - self.theta_offset + self.speed_offset, 255))
				try:
					self._update_params(["wheels/state"])
					self._update_params(["wheels/pwm_1"])
					self._update_params(["wheels/pwm_2"])
				except rospy.ServiceException as exc:
					print("Service did not process request: " + str(exc))
			rospy.Rate(2)
		return


if __name__ == '__main__':
	rospy.init_node('wheel_control')
	cf_num = sys.argv[1]
	wheel_ctrl = wheel_controller(cf_num)
	rospy.spin()