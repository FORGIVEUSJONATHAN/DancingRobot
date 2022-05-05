#! /usr/bin/env python3

import rospy, sys, os, json
from follow import Follow

# env variables
NAME = os.getenv('ROBOT')


def console_log(msg):
	print(f'[{NAME}] {msg}')


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

