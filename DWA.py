#! /usr/bin/env python

#roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch


from local_occupancy_grid_map import occupancy_map
import rospy

from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray, Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry 

import numpy as np 
from tf.transformations import euler_from_quaternion

import sys

################################# class PID ########################


    
################################# class FollowPathPID ########################

class DWA():

    def __init__(self, target):
        
        #### Initialization of robot- and ros-specific variables
        rospy.init_node("DWA")

        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')   # use 'mobile_base' for turtlebot2
        
        print ('Robot model: ', self.robot_model)
        
        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        if ('turtlebot2' in self.robot_model):
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        

        self.map_sub = rospy.Subscriber('/local_map', OccupancyGrid, self.map_callback)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rel_pose_sub = rospy.Subscriber('/to_target', Odometry, self.rel_pose_callback)

       
        # initilize the DWA parameters
        self.alpha = 1.0 # parameter for the heading 
        self.beta = 50.0 # parameter for the dist
        self.gamma = 0 # parameter for the velocity
        
        self.target_x = target[0]
        self.target_y = target[1]

        self.odo_x = 0
        self.odo_y = 0
        self.odo_yaw = 0
        # Twist object to set velocities
        self.vel = Twist()
        self.vel.linear = Vector3(0.0, 0.0, 0.0)
        self.vel.angular = Vector3(0.0, 0.0, 0.0)

        # Max velocities for the robot
        self.linear_vel_max = rospy.get_param('~max_lin_vel', 0.28)
        self.angular_vel_max = rospy.get_param('~max_ang_vel', 0.5)

        # Action rate
        self.time_window = 5
        self.frequency_updates = 50
        self.Rate = rospy.Rate(self.frequency_updates)
        self.velocity_pair = np.empty((0,4),float)

    def map_callback(self,msg):
        self.occupancy_grid = msg.data
        self.occ_grid_height = msg.info.height
        self.occ_grid_width = msg.info.width

        self.occupancy_grid = np.array(msg.data).reshape((self.occ_grid_width,self.occ_grid_height))
        # print(type(self.occupancy_grid))
        # print(self.occupancy_grid[0,0])
        for v_linear in np.arange(-self.linear_vel_max, self.linear_vel_max, 0.1):
            for w_angular in np.arange(-self.angular_vel_max, self.angular_vel_max,0.1):
                x_k = 0.0
                y_k = 0.0
                theta_k = 0.0
                theta_k1 = 0.0
                flag_obs = False ## flag to indicate the obs, False meaning not crashing
                for _ in range(int(self.time_window/0.01)):  ## for 1s, we use numerical integral with interval 0.01
                    theta_k1 = theta_k + w_angular * 0.01
                    # print("theta_k", theta_k1)
                    delta_theta = theta_k1 - theta_k
                    x_k = x_k + v_linear * 0.01 * np.cos(theta_k + delta_theta/2)
                    y_k = y_k + v_linear * 0.01 * np.sin(theta_k + delta_theta/2)
                    theta_k = theta_k1

                    y_idx = int(self.occ_grid_height//2 + y_k * 100)
                    x_idx = int(self.occ_grid_width//2 + x_k * 100)
                    # print((x_idx, y_idx))
                    if (self.occupancy_grid[x_idx,y_idx] >= 70): ## if the grid is not occupied
                        print("AMOGUS")
                        flag_obs = True
                        break

                if not flag_obs:
                    self.velocity_pair = np.append(self.velocity_pair,[[v_linear,w_angular,x_k,y_k]], axis=0) ## velocity pairs with x and y

        min_cost = np.Inf
        index = np.Inf
        # print(self.velocity_pair)
        for i in range(self.velocity_pair.shape[0]):
            v_cost = self.velocity_cost(self.velocity_pair[i,0],self.velocity_pair[i,1])
            d_cost = self.distance_cost(self.velocity_pair[i,2],self.velocity_pair[i,3])
            total_cost = self.gamma*v_cost + self.beta*d_cost
            if float(total_cost) < min_cost:
                min_cost = total_cost
                index = i
        print(" -------", self.velocity_pair[index])

        vel_final_vel = float(self.velocity_pair[index,0])
        vel_final_ang = float(self.velocity_pair[index,1])

        self.vel.linear.x = vel_final_vel
        self.vel.angular.z = vel_final_ang

        self.vel_pub.publish(self.vel)
        self.velocity_pair = np.empty((0,4),float)

    ## velocity cost function of pairs of (v,w)
    def velocity_cost(self,v,w): 
        return -(abs(v))

    ## distance cost function
    def distance_cost(self,x,y):
        # print("odo_yaw",self.odo_yaw)
        x_y_distance = np.sqrt(x**2 + y**2)
        
        if x > 0:
            if y > 0:
                x_rel_global_frame = x_y_distance * np.cos(self.odo_yaw) + self.odo_x
                y_rel_global_frame = - x_y_distance * np.sin(self.odo_yaw) + self.odo_y
            else:
                x_rel_global_frame = - x_y_distance * np.cos(self.odo_yaw) + self.odo_x
                y_rel_global_frame = - x_y_distance * np.sin(self.odo_yaw) + self.odo_y
        else:
            if y > 0:
                x_rel_global_frame = x_y_distance * np.cos(self.odo_yaw) + self.odo_x
                y_rel_global_frame = x_y_distance * np.sin(self.odo_yaw) + self.odo_y
            else:
                x_rel_global_frame = - x_y_distance * np.cos(self.odo_yaw) + self.odo_x
                y_rel_global_frame = x_y_distance * np.sin(self.odo_yaw) + self.odo_y

        relative_distance = np.sqrt((self.target_x - x_rel_global_frame)**2 + (self.target_y - y_rel_global_frame)**2)

        # print(x,y,self.target_x, self.target_y, relative_distance)
        return relative_distance

    def odom_callback(self,msg):
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        (_r, _p, self.odo_yaw)   = euler_from_quaternion((quaternion.x, quaternion.y, 
                                                                  quaternion.z, quaternion.w))


    def rel_pose_callback(self, msg): ## relative pose from robot
        pass

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
        DWA((2,2))
        while not rospy.is_shutdown():
            rospy.spin()

        
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
        

