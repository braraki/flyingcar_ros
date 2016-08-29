#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from crazyflie_teleop.msg import DriveCmd
from crazyflie_teleop.msg import WheelParams
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
import math
import sys
import tf
import threading
import time

class wheel_controller:

	def __init__(self, cf_num, name,channel):
		#rospy.wait_for_service('update_params')
		#rospy.loginfo("found update_params service")
		#self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
		self.param_pub = rospy.Publisher('/c' + channel + '/update_params', WheelParams, queue_size=10)

		rospy.set_param('in_air', False)

		self.cf_num = cf_num
		self.name = name
		self.channel = channel

		self.x = 0
		self.y = 0
		self.theta = 0

		# speed
		self.prev_speeds = []
		self.speed = 0
		self.prev_update_time = 0

		self.vel_x = 0
		self.vel_y = 0

		# goals
		self.goal_x = 0
		self.goal_y = 0
		self.goal_speed = 0
		self.goal_t = 0
		self.next_x = 0
		self.next_y = 0
		self.next_t = 0

		# controls
		self.baseline = 0
		self.theta_offset = 0
		self.speed_offset = 0

		self.theta_heading = 0
		self.theta_error = 0

		self.theta_offset_constant = 2.5
		self.speed_offset_constant = 10

		# margin of error
		self.pos_error = 0.01

		rospy.Subscriber("drive_goal", DriveCmd, self.update_goal)
		rospy.Subscriber("pose_localization", Odometry, self.update_position)
		self.wheel_command()

		# ctrl_thread = threading.Thread(target=self.wheel_command)
		# ctrl_thread.daemon = True
		# ctrl_thread.start()

	def update_goal(self, data):
		self.goal_x = data.x
		self.goal_y = data.y
		self.goal_t = float(data.t.secs+data.t.nsecs*10**(-9))

		self.next_x = data.x2
		self.next_y = data.y2
		self.next_t = float(data.t2.secs+data.t.nsecs*10**(-9))

		#print "goal:" + str(self.goal_x) + " | " + str(self.goal_y)

		self.goal_speed = math.sqrt((self.goal_x-self.x)**2+(self.goal_y-self.y)**2)/float(self.goal_t-time.time())

		#print self.goal_speed
		#print float(self.goal_t-rospy.Time.now().nsecs)
		#print self.goal_t, self.prev_goal_t

	def update_position(self,data):
			self.x = data.pose.pose.position.x
			self.y = data.pose.pose.position.y
			q = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
			self.theta = tf.transformations.euler_from_quaternion(q)[2] - math.pi/2.0
			if self.theta < -math.pi:
				self.theta = self.theta + 2*math.pi

			#print "yaw: " + str(180*self.theta/math.pi)

			self.vel_x = data.twist.twist.linear.x
			self.vel_y = data.twist.twist.linear.y


	def check_goal(self):
		if abs(self.x - self.goal_x) < self.pos_error and abs(self.y - self.goal_y) < self.pos_error:
			return True
		else:
			return False

	def wheel_command(self): #NTS could this be having bad timing interactions? It's completely possible that the goal updates during a runthrough.
		r = rospy.Rate(30)
		while not rospy.is_shutdown():	#NTS if goals and positions/angles change while running, this could affect calculation? Add a lock to be safe?
			# print "entering commander!"	# NTS A lock might also mess up the subscribers because of lag?
			# calculate baseline
			self.baseline = (self.goal_speed + 0.0092033)/0.0008
			self.baseline = min(255, self.baseline)
			self.baseline = max(0, self.baseline)
			self.baseline = int(self.baseline)

			# calculate theta offset
			x_e = self.next_x - self.x
			y_e = self.next_y - self.y

			distance_to_goal = math.sqrt((self.x-self.goal_x)**2 + (self.y-self.goal_y)**2)
			distance_constant = 1.0
			if distance_to_goal < 0.15:
				distance_constant = distance_to_goal/0.15;

			if x_e == 0:
				if y_e > 0:
					self.theta_heading = math.pi/2
				else:
					self.theta_heading = -math.pi/2
			else:
				self.theta_heading = math.atan2(y_e, x_e)


			self.theta_error = self.theta_heading - self.theta
			if self.theta_error > math.pi:
				self.theta_error -= 2*math.pi
			elif self.theta_error < -math.pi:
				self.theta_error += 2*math.pi
			self.theta_offset = self.theta_error/math.pi * 170 * self.theta_offset_constant
			#self.theta_offset = min(self.theta_offset, 85)
			#self.theta_offset = max(self.theta_offset, -85)
			self.theta_offset = int(self.theta_offset)

			#print x_e, y_e, self.theta_error*180/math.pi

			# calculate speed offset
			self.speed = math.sqrt(self.vel_x**2 + self.vel_y**2)
			#print "speed: " + str(self.speed)
			#print "yaw: " + str(180.0*self.theta/math.pi)
			#print self.speed, self.goal_speed
			speed_error = self.goal_speed - self.speed

			self.speed_offset = speed_error * self.speed_offset_constant
			self.speed_offset = int(self.speed_offset)

			# print max(0,min(self.baseline + self.theta_offset + self.speed_offset, 255)), max(0,min(self.baseline - self.theta_offset + self.speed_offset, 255))
			wheel_params = WheelParams()
			wheel_params.tf_prefix = 'c' + self.channel + '/' + self.name
			if not rospy.get_param('in_air'):
				if self.check_goal():
					state = 0
					pwm1 = 0
					pwm2 = 0
				else:
					# pwm1 is on the right
					# pwm 2 is on the left
					pwm = max(0,min(self.baseline + self.speed_offset, 255))
					#print "theta error:" + str(self.theta_error)
					if self.theta_error > math.pi/3.0:
						#print "turning left"
						#turn left (state 2)
						state = 2
						pwm1 = 70
						pwm2 = 70
					elif self.theta_error < -math.pi/3.0:
						#turn right (state 3)
						#print "turning right"
						state = 3
						pwm1 = 70
						pwm2 = 70
					else:
						theta_percent = self.theta_offset/math.pi
						pwm_right = max(0,min(distance_constant*(pwm + self.theta_offset), 255))
						pwm_left = max(0,min(distance_constant*(pwm - self.theta_offset), 255))
						state = 1
						pwm1 = pwm_right
						pwm2 = pwm_left
						#print "parameters updated!"
						#print self.theta_offset, self.speed_offset, self.baseline
						#print pwm_left, pwm_right
			else:
				state = 0
				pwm1 = 0
				pwm2 = 0

			wheel_params.state = state
			wheel_params.pwm1 = pwm1
			wheel_params.pwm2 = pwm2
			self.param_pub.publish(wheel_params)
			r.sleep()
		return


if __name__ == '__main__':
	rospy.init_node('wheel_control')
	cf_num = sys.argv[1]
	name = sys.argv[2]
	channel = sys.argv[3]
	wheel_ctrl = wheel_controller(cf_num,name,channel)
	rospy.spin()