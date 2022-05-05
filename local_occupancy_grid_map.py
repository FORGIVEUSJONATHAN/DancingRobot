#! /usr/bin/env python3

from time import sleep
import rospy

from geometry_msgs.msg import Twist, Vector3, Pose

from std_msgs.msg import Float32MultiArray, Empty
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np 
from tf.transformations import euler_from_quaternion
from collections import deque

import sys


class occupancy_map():
    def __init__(self):
        rospy.init_node('local_occupancy_grid_map')
    
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size=5)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.vel = Twist()
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(20)

        # initialize the settings of grid map
        self.resolution = 0.05
        self.width = int(3/self.resolution)
        self.height = self.width
        self.center = (self.width//2,self.height//2)

        
    def clbk_laser(self,msg):

        ## creating the binary grid map
        Occ_map = OccupancyGrid()
        Occ_map.header.frame_id = "base_footprint" 
        Occ_map.info.width = self.width
        Occ_map.info.height = self.height
        Occ_map.info.resolution = self.resolution
        robot_pose = Pose()
        robot_pose.position.x = -1.5
        robot_pose.position.y = -1.5
        robot_pose.position.z = 0.0
        Occ_map.info.origin = robot_pose


        Occ_map.data = np.zeros((self.width,self.height), dtype=np.int).flatten()

        grid_map = np.zeros((self.width,self.height), dtype=np.int)

        self.range = msg.ranges
        for i in range(len(self.range)):

            if self.range[i]==float("inf"):
                x = 0
                y = 0
            else:
                x = np.sin(np.radians(i)) * self.range[i] / self.resolution
                y = np.cos(np.radians(i)) * self.range[i] / self.resolution

            if x!=0 and y!=0 and x<(self.width//2) and y<(self.width//2):
                grid_map[int(x//1)+self.center[0], int(y//1)+self.center[1]-1] = 100

        grid_map = grid_map.flatten()
        Occ_map.data = list(grid_map)
        
        self.map_pub.publish(Occ_map)


    def shutdown(self):
        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)
        
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

################################# Main() ########################
                            
if __name__ == '__main__':

    try:
        occupancy_map()
        while not rospy.is_shutdown():
            rospy.spin()

        
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
        

