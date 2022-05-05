#! /usr/bin/env python3

import rospy, os, json, pickle
from nav_msgs.msg import Odometry
from multiprocessing.connection import Listener
from tf.transformations import euler_from_quaternion

# get the list of other robots in network (set as env variable)
robot_list = []
if os.getenv('ROBOT_LIST') != None:
    robot_list = json.loads(os.environ['ROBOT_LIST'])

# robot name
robot = os.getenv('ROBOT')


class Comm:
    def __init__(self):
        rospy.init_node("comm")

        self.odo_x = 0
        self.odo_y = 0
        self.odo_yaw = 0

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)


    def odom_callback(self,msg):
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_r, _p, self.odo_yaw)   = euler_from_quaternion((quaternion.x, quaternion.y, 
                                                          quaternion.z, quaternion.w))


        try:
            # open file and write odometry data to it
            with open(f'shared/{robot}.pkl', 'wb') as outf:
                pickle.dump(msg, outf, pickle.HIGHEST_PROTOCOL)

            # read odometry data for the other robots and publish in their respective topics
            for r in robot_list:
                pub = rospy.Publisher(f'/{r}/odom', Odometry, queue_size=5)
                data = None
                with open(f'shared/{r}.pkl', 'rb') as inp:
                    data = pickle.load(inp)
                pub.publish(data)
        except:
            pass


if __name__ == '__main__':

    try:
        Comm()
        while not rospy.is_shutdown():
            rospy.spin()
        
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)

 
