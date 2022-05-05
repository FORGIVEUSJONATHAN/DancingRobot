#! /usr/bin/env python3

#roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch


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

################################# class PID ########################


    
################################# class FollowPathPID ########################

class DWA():

    def __init__(self, target_list):
        
        #### Initialization of robot- and ros-specific variables
        rospy.init_node("DWA", log_level=rospy.DEBUG)


        self.isDone = False
        self.target_list = target_list

        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')   # use 'mobile_base' for turtlebot2

        self.is_map_started = False
        
        print ('Robot model: ', self.robot_model)
        
        # What function to call when ctrl + c is issued

        if ('turtlebot2' in self.robot_model):
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        

        

        # TF2 stuff
        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(5))
        tf2_ros.TransformListener(self.tf2_buffer)

       
        # initilize the DWA parameters
        self.alpha = 1.0 # parameter for the heading 
        self.beta = 1.0 # parameter for the dist
        self.gamma = 0 # parameter for the velocity
        
        self.target_x = target_list[0][0]
        self.target_y = target_list[0][1]

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
        self.frequency_updates = 50
        self.Rate = rospy.Rate(self.frequency_updates)
        self.velocity_pair = np.empty((0,4),float)


        self.map_sub = rospy.Subscriber('/local_map', OccupancyGrid, self.map_callback)
        rospy.on_shutdown(self.shutdown)

        self.odom_sub = rospy.Subscriber('/amcl/odom', Odometry, self.odom_callback)
        self.rel_pose_sub = rospy.Subscriber('/to_target', Odometry, self.rel_pose_callback)

    def map_callback(self, msg):
        if not self.is_map_started:
            self.is_map_started = True

        self.occupancy_grid = list(msg.data)
        self.occ_grid_height = msg.info.height
        self.occ_grid_width = msg.info.width

        self.occupancy_grid = np.array(msg.data).reshape((self.occ_grid_width,self.occ_grid_height), order='F')


    def run(self):
        while not self.is_map_started:
            self.Rate.sleep()
        print("RUNNING")
        # if self.isDone:
        #     print("DONE")
        #     # self.vel.linear.x = 0
        #     # self.vel.angular.z = 0
        #     # self.vel_pub.publish(self.vel)
        #     self.map_sub.unregister()
        #     return
        for i in self.target_list:

            print("MOVING TO", i)
            self.target_x = i[0]
            self.target_y = i[1]

            d_cost = np.Inf
            while d_cost > 0.7:

                for v_linear in np.arange(-self.linear_vel_max, self.linear_vel_max+0.1, 0.1):
                # for v_linear in np.arange(0, self.linear_vel_max+0.1, 0.05):
                    for w_angular in np.arange(-self.angular_vel_max, self.angular_vel_max+0.1,0.1):
                        #v_linear = self.linear_vel_max
                        #w_angular = 0

                        x_k = 0.0
                        y_k = 0.0
                        theta_k = 0.0
                        theta_k1 = 0.0
                        flag_obs = False ## flag to indicate the obs, False meaning not crashing

                        dt = 0.1

                        rospy.logdebug("Checking %.2f %.2f", v_linear, w_angular)
                        rospy.logdebug("Using dt %.2f time window %.2f", dt, self.time_window)

                        for _ in range(int(self.time_window/dt)):  ## for 1s, we use numerical integral with interval 0.01
                            theta_k1 = theta_k + w_angular * dt
                            #rospy.logdebug("theta_k1 %.4f", theta_k1)
                            #rospy.logdebug("theta_k %.4f", theta_k)

                            delta_theta = theta_k1 - theta_k
                            x_k = x_k + v_linear * dt * np.cos(theta_k + delta_theta/2)
                            y_k = y_k + v_linear * dt * np.sin(theta_k + delta_theta/2)
                            theta_k = theta_k1

                            # rospy.logdebug("Pred loc %.4f %.4f", x_k, y_k)

                            x_idx = int(self.occ_grid_height//2 + x_k * 10)
                            y_idx = int(self.occ_grid_width//2 + y_k * 10)

                            
                            check = 0

                            for i in range(-2, 3):
                                for j in range(-2, 3):
                                    check += self.occupancy_grid[y_idx+i, x_idx+j]

                            rospy.logdebug("checking %d %d (%d) (%d)", x_idx, y_idx, 
                                self.occupancy_grid[y_idx, x_idx], check)


                            if (check >= 70): ## if the grid is occupied
                                rospy.logdebug("Invalid vel: occupied")
                                flag_obs = True
                                break

                        if not flag_obs:
                            self.velocity_pair = np.append(self.velocity_pair,[[v_linear,w_angular,x_k,y_k]], axis=0) ## velocity pairs with x and y
                
                    min_cost = np.Inf
                    index = np.Inf

                    with open("dgb.txt", 'w') as wrtf:
                        for i in range(60):
                            wrtf.write(f'{i} ')
                            for j in range(60):
                                wrtf.write(f'{self.occupancy_grid[i,j]} ')
                            wrtf.write("\n")

                    costs = {}
                    for i in range(self.velocity_pair.shape[0]):
                        v_cost = self.velocity_cost(self.velocity_pair[i,0],self.velocity_pair[i,1])
                        d_cost = self.distance_cost(self.velocity_pair[i,2],self.velocity_pair[i,3])

                        # if d_cost < 0.3:
                        #     self.isDone = True
                        #     continue

                        total_cost = self.gamma*v_cost + self.beta*d_cost
                        rospy.logdebug("COST: %.4f", total_cost)
                        costs[i] = (total_cost, (self.velocity_pair[i,2],self.velocity_pair[i,3]))

                        if float(total_cost) < min_cost:
                            min_cost = total_cost
                            index = i

                    # for k in costs:
                    #     vel_vel = float(self.velocity_pair[k,0])
                    #     vel_ang = float(self.velocity_pair[k,1])

                    # if vel_final_vel < 0:
                    #     rospy.logerr(f"Cost for {vel_vel} : {vel_ang} is {costs[k]}, all: {costs}")

                    # print(" -------", self.velocity_pair[index])        
                    # print("OK", self.velocity_pair,  index)
                    vel_final_vel = float(self.velocity_pair[index,0])
                    vel_final_ang = float(self.velocity_pair[index,1])

                    rospy.logerr("selected %.4f, %.4f, %.4f", vel_final_vel, vel_final_ang, min_cost)

                    self.vel.linear.x = vel_final_vel
                    self.vel.angular.z = vel_final_ang

                    self.vel_pub.publish(self.vel)
                    self.Rate.sleep()

                self.velocity_pair = np.empty((0,4),float)

        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        # rospy.logerr("TIME: %.5f", (time.time() - start_time))

    ## velocity cost function of pairs of (v,w)
    def velocity_cost(self,v,w): 
        return -(abs(v))

    ## distance cost function
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
        #print '\n ***> Point stamped: \n', point_stamped

        # ---> transform the point coordinates into the local (i.e., robot) reference frame
        relative_position_of_point = do_transform_point(point_stamped, local_in_reference)

        delta_y = relative_position_of_point.point.y
        delta_x = relative_position_of_point.point.x

        relative_distance = np.sqrt((x-delta_x)**2 + (y-delta_y)**2)

        return relative_distance

    def periodic_broadcast_start_frame(self, timer):
        self.start_frame.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(self.start_frame)


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
        

