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

		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		rospy.on_shutdown(self.shutdown)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)



	def angle_to_point(self, targetx, targety):
		y1 = (targety - self.odo_y)
		x1 = (targetx - self.odo_x)

		(y2, x2) = (np.sin(self.odo_yaw), np.cos(self.odo_yaw))

		angle_to_target = np.arctan2(y1*x2 - y2*x1, x1*x2 + y1*y2)

		# print(self.odo_x, self.odo_y, x1, y1, x2, y2, y1*x2 - y2*x1, x1*x2 + y1*y2, angle_to_target)
		# print(self.odo_x, self.odo_y, x1, y1, angle_to_target)
		# print(targetx, targety, x1, y1, angle_to_target)

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

		# print("target:", (self.target_x, self.target_y), "err_dist:", err_dist, "err_yaw", err_yaw)

		if err_dist < self.distance_tolerance:
			self.isDone = True
			# self.vel.linear.x = 0
			# self.vel.angular.z = 0
			# self.vel_pub.publish(self.vel)
			return

		gain_dist = 0.5
		gain_angle = 2

		# vel_linear = gain_dist * err_dist
		vel_linear = self.max_linear_vel
		vel_angular = gain_angle * err_yaw

		# print(vel_angular)

		# if vel_linear > self.max_linear_vel:
		# 	vel_linear = self.max_linear_vel

		# if vel_linear < self.min_linear_vel:
		# 	vel_linear = self.min_linear_vel

		# if vel_angular > self.max_angular_vel:
		# 	vel_angular = self.max_angular_vel

		self.vel.linear.x = vel_linear
		self.vel.angular.z = vel_angular

		self.vel_pub.publish(self.vel)


	def shutdown(self):
		# print ("Shutdown!")
		# rospy.loginfo("Stop TurtleBot")

		self.vel.linear.x = 0
		self.vel.angular.z = 0

		self.vel_pub.publish(self.vel)

		rospy.sleep(1)

def execute_path(path):
	for i in path:
		# print("moving to", i)
		f = MoveToPoint(i[0], i[1])
		while not f.isDone:
			rospy.Rate(50).sleep()
		# print("Done")


# if __name__ == "__main__":

# 	try:
# 		path = [(0.5,0.3), (1,0), (1.5,-0.3), (2,0.3), (0,0)]
# 		# path = [(1,0), (1,1), (0,0)]
# 		for i in path:
# 			f = MoveToPoint(i[0], i[1])
# 			while not f.isDone:
# 				rospy.Rate(50).sleep()
# 			print("Done")

# 		# vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
# 		# vel = Twist()
# 		# vel.linear = Vector3(0.0, 0.0, 0.0)
# 		# vel.angular = Vector3(0.0, 0.0, 0.0)

# 		# for _ in range(50):
# 		# 	vel_pub.publish(vel)

# 		# rospy.sleep(2)

# 	except:
# 		msg = f'[{rospy.get_name()}] ERROR: Node terminated'
# 		rospy.loginfo(msg)
# 		error_type = sys.exc_info()[0]
# 		error_msg = sys.exc_info()[1]
# 		rospy.logerr(error_type)
# 		rospy.logerr(error_msg)

