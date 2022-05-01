#! /usr/bin/env python3

import rospy, os, json, socket, pickle
from nav_msgs.msg import Odometry
from multiprocessing.connection import Listener
from tf.transformations import euler_from_quaternion


robot_list = []

if os.getenv('ROBOT_LIST') != None:
    robot_list = json.loads(os.environ['ROBOT_LIST'])

robot = os.getenv('ROBOT')


def get_free_port():
    sock = socket.socket()
    sock.bind(('', 0))
    return sock.getsockname()[1]



class Listener:
    def __init__(self):
        rospy.init_node("listener")

        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.odo_x = 0
        self.odo_y = 0
        self.odo_yaw = 0


    def odom_callback(self,msg):
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_r, _p, self.odo_yaw)   = euler_from_quaternion((quaternion.x, quaternion.y, 
                                                                  quaternion.z, quaternion.w))
        try:
            with open(f'shared/{robot}.pkl', 'wb') as outf:
                pickle.dump(msg, outf, pickle.HIGHEST_PROTOCOL)

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
        l = Listener()
        while not rospy.is_shutdown():
            rospy.spin()
        
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)

 
