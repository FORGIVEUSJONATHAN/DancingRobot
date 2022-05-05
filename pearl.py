#! /usr/bin/env python3

import rospy, sys, os, json
import numpy as np
from RRT import RRT
from follow_path import execute_path
from follow import Follow

# map parameters
resolution = 0.050
origin_x = 327
origin_y = 348
origin_yaw = -2.12


# env variables
NAME = os.getenv('ROBOT')
robot_postion = json.loads(os.getenv('START_POS'))
target_position = json.loads(os.getenv('TARGET_POS'))


def console_log(msg):
	print(f'[{NAME}] {msg}')


# TRANSFORMATION FUNCTIONS


# calculate angle to pont in robot frame
def angle_to_point (target):
	y1 = (target[1] - origin_y)
	x1 = (target[0] - origin_x)

	# robot orientations
	(y2, x2) = (np.sin(origin_yaw), np.cos(origin_yaw))
	angle_to_target = np.arctan2 (y1*x2 - y2*x1, x1*x2 + y1*y2)

	return angle_to_target

def transform_robot_frame_pixel(target):
	a = angle_to_point(target)
	d = np.sqrt((target[0] - origin_x)**2 + (target[1] - origin_y)**2)

	# the output points were flipped on x axis so its -y instead of y (according to desmos)
	return (d*np.cos(a)*resolution, -d*np.sin(a)*resolution)



# use RRT
def calculate_path(start, target):
	image_path_base = '/home/parallels/workspaces/catkin_ws/src/project/src'
	path_object = RRT(start, target, image_path_base)

	last_node, tree = path_object.find_path()
	current = last_node
	lists = []

	while current!=None:
		lists.append(current.value)
		current = current.parent

	final_path = lists
	final_path.reverse()

	final_path = [transform_robot_frame_pixel(i) for i in final_path]
	return final_path


if __name__ == '__main__':

	try:

		# execute sequence for Pearl
		console_log("Following allen!!")
		Follow()
		while not rospy.is_shutdown():
			rospy.spin()


	except:
		msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
		rospy.loginfo(msg)
		error_type = sys.exc_info()[0]
		error_msg = sys.exc_info()[1]
		rospy.logerr(error_type)
		rospy.logerr(error_msg)

