#! /usr/bin/env python3

import rospy, sys
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MoveToPoint:
	def __init__(self, tx, ty):
		rospy.init_node("move_to_point")

		self.Rate = rospy.Rate(50)

		self.target_x = tx
		self.target_y = ty

		self.vel = Twist()
		self.vel.linear = Vector3(0.0, 0.0, 0.0)
		self.vel.angular = Vector3(0.0, 0.0, 0.0)

		self.odo_x = 0
		self.odo_y = 0
		self.odo_yaw = 0

		self.max_linear_vel = 0.2
		self.max_angular_vel = 0.5

		self.distance_tolerance = 0.05
		self.isDone = False

		rospy.on_shutdown(self.shutdown)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)


	def angle_to_point(self, targetx, targety):
		y1 = (targety - self.odo_y)
		x1 = (targetx - self.odo_x)

		(y2, x2) = (np.sin(self.odo_yaw), np.cos(self.odo_yaw))

		angle_to_target = np.arctan2(y1*x2 - y2*x1, x1*x2 + y1*y2)

		return angle_to_target


	def distance_to_point(self, targetx, targety):
		diffx = targetx - self.odo_x
		diffy = targety - self.odo_y

		return np.sqrt(diffx*diffx + diffy*diffy)


	def odom_callback(self, msg):
		if self.isDone:
			return

		self.odo_x = msg.pose.pose.position.x
		self.odo_y = msg.pose.pose.position.y
		quaternion = msg.pose.pose.orientation
		(_r, _p, self.odo_yaw)   = euler_from_quaternion((quaternion.x, quaternion.y, 
														  quaternion.z, quaternion.w))

		err_dist = self.distance_to_point(self.target_x, self.target_y)
		err_yaw = self.angle_to_point(self.target_x, self.target_y)


		# check if target is within tolerence
		if err_dist < self.distance_tolerance:
			self.isDone = True
			return

		gain_dist = 0.5
		gain_angle = 2

		# set constant linear velocity, but P controller for angular
		vel_linear = self.max_linear_vel
		vel_angular = gain_angle * err_yaw

		self.vel.linear.x = vel_linear
		self.vel.angular.z = vel_angular

		self.vel_pub.publish(self.vel)


	def shutdown(self):
		self.vel.linear.x = 0
		self.vel.angular.z = 0

		self.vel_pub.publish(self.vel)

		rospy.sleep(1)


# this function is imported
def execute_path(path):
	for i in path:
		f = MoveToPoint(i[0], i[1])
		while not f.isDone:
			rospy.Rate(50).sleep()

