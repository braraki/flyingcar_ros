#!/usr/bin/env python

import rospy
import numpy as np

from time import sleep
import time, threading

import std_msgs.msg
import tf
from numpy.linalg import inv
import math


class KalmanR3:

	def __init__(self, dt):
		# statemachine constants
		self.pos = np.array([0.0,0.0,0.0])
		self.vel = np.array([0.0,0.0,0.0])

		qx = 0.01
		qy = 0.01
		qz = 0.01

		rx = 0.002
		ry = 0.002
		rz = 0.002

		self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

		# process noise
		self.Q = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
						   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
						   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
						   [0.0, 0.0, 0.0,  qx, 0.0, 0.0],
						   [0.0, 0.0, 0.0, 0.0,  qy, 0.0],
						   [0.0, 0.0, 0.0, 0.0, 0.0,  qz]])

		# measurement noise
		self.R = np.array([[rx, 0.0, 0.0],
						   [0.0, ry, 0.0],
						   [0.0, 0.0, rz]])

		# covariance?...
		self.P = np.eye(6)

		# transition matrix x_(t+1) = A * x_t + Q
		self.A = np.array([[1.0, 0.0, 0.0,  dt, 0.0, 0.0],
						   [0.0, 1.0, 0.0, 0.0,  dt, 0.0],
						   [0.0, 0.0, 1.0, 0.0, 0.0,  dt],
						   [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
						   [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
						   [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

		# y = H * x + R
		self.H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
						   [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
						   [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])

	def step(self,z):

		# callback for the joy node
		x = self.x
		A = self.A
		H = self.H
		P = self.P
		Q = self.Q
		R = self.R

		# prediction
		x = A.dot(x)
		tp = A.dot(P)
		P = tp.dot(A.transpose()) + Q

		# update
		y = z-H.dot(x)
		Sp = H.dot(P)
		S = Sp.dot(H.transpose()) + R
		Ht = H.transpose()
		tmp = Ht.dot(inv(S))
		K = P.dot(tmp)
		x = x + K.dot(y)
		tmp = (np.eye(6)-K.dot(H))
		P = tmp.dot(P)

		self.P = P
		self.x = x

		self.pos[0] = x[0]
		self.pos[1] = x[1]
		self.pos[2] = x[2]

		self.vel[0] = x[3]
		self.vel[1] = x[4]
		self.vel[2] = x[5]