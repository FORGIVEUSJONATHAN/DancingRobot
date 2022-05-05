#! /usr/bin/env python3

import rospy, sys
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class SetOdom:
	def __init__(self, x, y, z, w):
		rospy.init_node("set_odom")

		self.vel = Twist()
		self.vel.linear = Vector3(0.0, 0.0, 0.0)
		self.vel.angular = Vector3(0.0, 0.0, 0.0)

		self.odom = None

		self.isDone = False
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

		while self.odom == None:
			rospy.Rate(50).sleep()

		self.isDone = True
		self.odom_sub.unregister()

		self.odom.pose.pose.position.x = x
		self.odom.pose.pose.position.y = y
		self.odom.pose.pose.orientation.z = z
		self.odom.pose.pose.orientation.w = w

		odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)
		odom_pub.publish(self.odom)
		print("Done")



	def distance_to_point(self, targetx, targety):
		diffx = targetx - self.odo_x
		diffy = targety - self.odo_y

		return np.sqrt(diffx*diffx + diffy*diffy)


	def odom_callback(self, msg):
		if self.isDone:
			return
		self.odom = msg


if __name__ == "__main__":

	try:
		SetOdom(5.180136680603027, -13.695976257324219, -0.7172034978866577, -0.6968637704849243)
		
		while not rospy.is_shutdown():
			rospy.spin()
		
	except:
		msg = f'[{rospy.get_name()}] ERROR: Node terminated'
		rospy.loginfo(msg)
		error_type = sys.exc_info()[0]
		error_msg = sys.exc_info()[1]
		rospy.logerr(error_type)
		rospy.logerr(error_msg)

