#! /usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Follow:
	def __init__(self):
		rospy.init_node("follow")

		self.vel = Twist()
		self.vel.linear = Vector3(0.0, 0.0, 0.0)
		self.vel.angular = Vector3(0.0, 0.0, 0.0)

		self.odo_x = 0
		self.odo_y = 0
		self.odo_yaw = 0

		self.max_linear_vel = 0.2
		self.max_angular_vel = 0.6

		self.distance_tolerance = 0.05

		rospy.on_shutdown(self.shutdown)

		# for now, we fix that pearl will follow allen, so this node is only executed on pearl
		# so we have a callback for Allen's odom (published through node 'comm')
		self.allen_odom_callback = rospy.Subscriber('/allen/odom', Odometry, self.allen_odom_callback)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


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


	def odom_callback(self,msg):
		self.odo_x = msg.pose.pose.position.x
		self.odo_y = msg.pose.pose.position.y
		quaternion = msg.pose.pose.orientation
		(_r, _p, self.odo_yaw)   = euler_from_quaternion((quaternion.x, quaternion.y, 
														  quaternion.z, quaternion.w))


	def allen_odom_callback(self, msg):

		# follow at a distance of 0.7 meters
		# (consider this point as target point)
		target_x = msg.pose.pose.position.x - 0.7
		target_y = msg.pose.pose.position.y

		# We use a standard P controller

		err_dist = self.distance_to_point(target_x, target_y)
		err_yaw = self.angle_to_point(target_x, target_y)

		# if error is small, we consider that we have reached target
		if err_dist < self.distance_tolerance:
			self.vel.linear.x = 0
			self.vel.angular.z = 0
			self.vel_pub.publish(self.vel)
			return

		gain_dist = 0.7
		gain_angle = 3

		vel_linear = gain_dist * err_dist
		vel_angular = gain_angle * err_yaw

		# clamp velocities
		if vel_linear > self.max_linear_vel:
			vel_linear = self.max_linear_vel
		if vel_angular > self.max_angular_vel:
			vel_angular = self.max_angular_vel


		self.vel.linear.x = vel_linear
		self.vel.angular.z = vel_angular

		self.vel_pub.publish(self.vel)


	def shutdown(self):
		self.vel.linear = Vector3(0.0, 0.0, 0.0)
		self.vel.angular = Vector3(0.0, 0.0, 0.0)

		self.vel_pub.publish(self.vel)

		rospy.sleep(1)

