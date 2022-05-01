#! /usr/bin/env python3

#roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

import rospy

from geometry_msgs.msg import Twist, Vector3

from std_msgs.msg import Float32MultiArray, Empty

from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan

import numpy as np 
from numpy.linalg import pinv
from numpy.linalg import inv

from tf.transformations import euler_from_quaternion
from collections import deque

import sys
    
################################# class Kalman Filter ########################
class KF():
    def __init__(self):
        rospy.init_node('KF')
    
        # use 'mobile_base' for turtlebot2?
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
        
        print ('Robot model: ', self.robot_model)

        if 'turtlebot2' in self.robot_model:
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        

        # where to read laser scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.scan_pub = rospy.Publisher('/kf_scan', LaserScan, queue_size=5)
        self.vel = Twist()
        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(20)

        ## initialize the setting of grid map
        self.resolution = 0.01
        self.width = 300
        self.height = 300
        self.center = (self.width//2,self.height//2)

    
        self.V = np.identity(360) * 9**(-5)
        self.W = np.identity(360) * 10**(-1)

        self.P = np.eye(360)        
        self.Ek0 = np.zeros((360,1),float)
        print("0",np.unique(self.Ek0))

        self.Ek1 = np.zeros((360,1), float)

        self.G = np.matmul((self.P + self.V),inv((self.P + self.V)+self.W))

    def prediction(self):

        self.Ek1 = self.Ek0
        print("0",np.unique(self.Ek0))

        print("1",np.unique(self.Ek1))

        self.Pk1 = self.P + self.V
    def update(self):
        self.Zk1 = self.Ek 
        self.G = np.matmul(self.Pk1, inv(self.Pk1+self.W))
        self.P = self.Pk1 - np.matmul(self.G,self.Pk1)
        # print("P", np.unique(self.P))
        self.Ek0 = self.Ek1 + np.matmul(self.G,(self.Zk1-self.Ek1))

        # print(np.unique(self.Ek1))
        # print(np.unique(np.dot(self.G,(self.Zk1-self.Ek1))))

    def clbk_laser(self,msg):
        msg.ranges = list(msg.ranges)
        for i in range(len(msg.ranges)):
            if msg.ranges[i] == float("inf"):
                msg.ranges[i] = 0
        # self.Ek = np.array([msg.ranges]).T
        self.Ek = np.array([msg.ranges]).reshape((len(msg.ranges),1))
        self.prediction()
        self.update()
        # print(self.Ek1)
        msg.ranges = self.Ek0.T.tolist()[0]
        # print(msg.ranges)

        # msg.header.frame_id = "base_footprint"

        self.scan_pub.publish(msg)

    def shutdown(self):
        print ("Shutdown!")
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")

        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)
        
        self.vel_pub.publish(self.vel)

	# sleep just makes sure TurtleBot receives the stop command prior to shutting down 
        rospy.sleep(1)    


################################# Main() ########################
                            
if __name__ == '__main__':

    try:
        KFfilter = KF()
        while not rospy.is_shutdown():
            rospy.spin()
        
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
        

