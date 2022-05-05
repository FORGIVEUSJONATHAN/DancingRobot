#! /usr/bin/env python3

import rospy
from numpy.linalg import inv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math
import numpy as np
import sys


class KF():

    def __init__(self):

        rospy.init_node('kf_scalar_filters')
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        rospy.on_shutdown(self.shutdown)        
        self.rate = rospy.Rate(100)


        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.kf_pub = rospy.Publisher('/kf_scan', LaserScan, queue_size=10)

        SIZE = 360

        self.V = 0.00001*np.identity(SIZE)
        self.W = 0.00001*np.identity(SIZE)

        self.P_k0 = np.eye(SIZE)
        self.prev_P_kp1gk = self.P_k0
        self.e_k = np.zeros( (SIZE,1) )


        
    def clbk_laser(self, msg):
        sensor_data = list(msg.ranges)

        for i in range(len(sensor_data)):
            if sensor_data[i] == float("inf"):
                sensor_data[i] = 0.0

        s_data = np.array([sensor_data]).reshape( (len(sensor_data), 1) )

        P_kp1gk = self.prev_P_kp1gk + self.V
        self.prev_P_kp1gk = P_kp1gk

        G_kp1 = np.matmul(P_kp1gk, inv(P_kp1gk + self.W))

        z_kp1 = s_data
        e_kp1 = (self.e_k + np.matmul(G_kp1, np.subtract(z_kp1,self.e_k)))
        self.e_k = e_kp1

        res = (e_kp1).flatten().tolist()
        msg.ranges = res #[0]

        self.kf_pub.publish(msg)
        
           

    def shutdown(self):
        v = Twist()
        v.linear.x = 0.0
        v.angular.z = 0.0

        self.vel_pub.publish(v)

        rospy.sleep(1)


    
if __name__ == '__main__':

    try:
        KF()
        
        while not rospy.is_shutdown():
            print("[KF] Running.... :)")
            rospy.spin()
            
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
        
