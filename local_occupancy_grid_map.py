#! /usr/bin/env python

#roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

import imp
from mmap import MAP_ANON, MAP_ANONYMOUS
from re import T
from time import sleep
import rospy

from geometry_msgs.msg import Twist, Vector3, Pose

from std_msgs.msg import Float32MultiArray, Empty
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from nav_msgs.msg import OccupancyGrid
import numpy as np 
from tf.transformations import euler_from_quaternion
from collections import deque

import sys



class occupancy_map():
    def __init__(self):
        rospy.init_node('local_occupancy_grid_map')
    
        # use 'mobile_base' for turtlebot2?
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
        
        print ('Robot model: ', self.robot_model)

        if 'turtlebot2' in self.robot_model:
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        self.map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size=5)

        # where to read laser scan data
        self.scan_sub = rospy.Subscriber('/kf_scan', LaserScan, self.clbk_laser)
        self.vel = Twist()
        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(20)

        ## initialize the setting of grid map
        self.resolution = 0.01
        self.width = 300
        self.height = 300
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
        print(len(self.range))
        for i in range(len(self.range)):
            if self.range[i]==float("inf"):
                x = 0
                y = 0
            else:
                x = np.sin(np.radians(i)) * self.range[i] / self.resolution
                y = np.cos(np.radians(i)) * self.range[i] / self.resolution
            if x!=0 and y!=0 and x<150 and y<150:
                # grid_map[int(x//1)+self.center[0], self.center[1]-int(y//1)] = 100 
                # grid_map[-self.center[1]-int(y//1), -int(x//1)-self.center[0]] = 100 
                grid_map[int(x//1)+self.center[0], int(y//1)-9+self.center[1]] = 100 

                print(i, self.range[i])
                print(np.unique(grid_map, return_counts=True))

        grid_map = grid_map.flatten()

        Occ_map.data = list(grid_map)
        
        print(np.unique(Occ_map.data, return_counts=True))
        self.map_pub.publish(Occ_map)

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
        occ_map = occupancy_map()
        while not rospy.is_shutdown():
            rospy.spin()

        
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
        

