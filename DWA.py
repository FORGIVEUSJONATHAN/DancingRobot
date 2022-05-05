#! /usr/bin/env python3


from local_occupancy_grid_map import occupancy_map
import rospy, time

from geometry_msgs.msg import Twist, Vector3, Quaternion, Point
from std_msgs.msg import Float32MultiArray, Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from tf2_geometry_msgs import PointStamped
from tf2_geometry_msgs import do_transform_point, do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
import tf2_ros

import sys


class DWA:
	def __init__(self, target_list):
		rospy.init_node("DWA")

        self.target_list = target_list

        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')   # use 'mobile_base' for turtlebot2
        if ('turtlebot2' in self.robot_model):
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


        # TF2 stuff
        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(5))
        tf2_ros.TransformListener(self.tf2_buffer)

        # initial target (fort item in list)
        self.target_x = target_list[0][0]
        self.target_y = target_list[0][1]

        # odometry params
        self.odo_x = 0
        self.odo_y = 0
        self.odo_yaw = 0

        # Twist object to set velocities
        self.vel = Twist()
        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)

        # Max velocities for the robot
        self.linear_vel_max = rospy.get_param('~max_lin_vel', 0.25)
        self.angular_vel_max = rospy.get_param('~max_ang_vel', 0.5)


        # Action rate
        self.time_window = 3
        self.Rate = rospy.Rate(100)

        # velocity pair array which will be used to select the best velocity
        self.velocity_pair = np.empty((0,4),float)


        # callbacks
        rospy.on_shutdown(self.shutdown)
        self.map_sub = rospy.Subscriber('/local_map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/amcl/odom', Odometry, self.odom_callback)


   	def distance_cost(self,x,y):
        local_in_reference = self.tf2_buffer.lookup_transform('base_footprint', 'odom',
                                                              rospy.Time(0), rospy.Duration(5))
        # create a PointStamped object to encapsulate the point to reach since
        # do_transform_point requires such an object
        point_stamped = PointStamped()
        point_stamped.point = Point(self.target_x, self.target_y, 0)
        point_stamped.header.frame_id = 'odom'
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.seq = 0

        relative_position_of_point = do_transform_point(point_stamped, local_in_reference)

        delta_y = relative_position_of_point.point.y
        delta_x = relative_position_of_point.point.x

        relative_distance = np.sqrt((x-delta_x)**2 + (y-delta_y)**2)

        return relative_distance


    def map_callback(self, msg):
        if not self.is_map_started:
            self.is_map_started = True

        self.occupancy_grid = list(msg.data)
        self.occ_grid_height = msg.info.height
        self.occ_grid_width = msg.info.width

        self.occupancy_grid = np.array(msg.data).reshape((self.occ_grid_width,self.occ_grid_height), order='F')



    def odom_callback(self,msg):
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_r, _p, self.odo_yaw) = euler_from_quaternion((quaternion.x, quaternion.y, 
                                                          quaternion.z, quaternion.w))


    def periodic_broadcast_start_frame(self, timer):
        self.start_frame.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(self.start_frame)


    def shutdown(self):
        print ("Shutdown!")
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")

        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)
        
        self.vel_pub.publish(self.vel)
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down 

        rospy.sleep(1)



if __name__ == '__main__':
	try:
		a = DWA([(4,0)])
		a.run()
		while not rospy.is_shutdown():
            rospy.spin()

        
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg) 