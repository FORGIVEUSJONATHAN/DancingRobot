#! /usr/bin/env python3

import rospy, sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class AmclOdom:
	def __init__(self):
		rospy.init_node("amcl_odom")
		self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
		

		self.amcl_odom_pub = rospy.Publisher('/amcl/odom', Odometry, queue_size=5)



		self.odom = None
		while self.odom == None:
			rospy.spin()

		self.pose = self.odom.pose
		
		self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.clbk_amcl)
		self.scan_sub = rospy.Subscriber('/odom', Odometry, self.clbk_odom)


	def clbk_amcl(self, msg):
		self.pose = msg.pose
		# self.amcl_odom_pub.publish(self.odom)

	def clbk_odom(self, msg):
		self.odom = msg
		# self.odom.pose = self.pose
		self.amcl_odom_pub.publish(self.odom)





if __name__ == "__main__":

	try:
		a = AmclOdom()
		while not rospy.is_shutdown():
			rospy.spin()

	except:
		msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
		rospy.loginfo(msg)
		error_type = sys.exc_info()[0]
		error_msg = sys.exc_info()[1]
		rospy.logerr(error_type)
		rospy.logerr(error_msg)