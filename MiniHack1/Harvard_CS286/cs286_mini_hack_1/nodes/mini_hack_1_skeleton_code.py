#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #


# yyy
'''
    For part 1: python mini_hack_1_skeleton_code.py --position_topic \odom --velocity_topic \cmd_vel --scan_topic \scan
'''
# yyy

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, isnan
from tf.transformations import euler_from_quaternion
import numpy as np
import time

# yyy
from nav_msgs.msg import Odometry
# yyy



#New===
import argparse
#Import the package corresponding to the LiDAR data from sensor_msgs
#Import the package corresponding to the message type of the position_topic from nav_msgs
#New===

STOP_DISTANCE = 0.4
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

print_msg = """
control your Turtlebot3 for Waypoint!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""


class GotoPoint():
    def __init__(self, position_topic, velocity_topic, scan_topic):
        rospy.init_node('pointop_key', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.r = rospy.Rate(10)
        
        #======New=====
        self.scan_topic = scan_topic
        self.cmd_vel = rospy.Publisher(velocity_topic, Twist, queue_size=5)
	
        '''
        Create a subscriber for the position_topic make sure to use the correct message type.        
        Reference: "Writing a subscriber" section in http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29"
        '''
        # yyy
        self.sub_pos = rospy.Subscriber(position_topic, Odometry, self.get_position_cb)
        # yyy
            
        self.rot = Quaternion()
        self.curr_position = Point()
        self.curr_rotation = 0
        self.waypoints = [[2.1,0,0],[2.0,2.1,0],[0,2.1,0],[0,0,0]] #These are the rough dimensions of the testbed
        self.waypoint_counter = 0
        #=====New=====
        
	
        
    def get_scan(self):
        '''
	Use the code in *_obstacle.py to update the scan_filter value which is returned.
	'''
    	# xxx

        return scan_filter


    def go_to_waypoint(self):
        position = Point()
        move_cmd = Twist()
        # yyy
        r = rospy.Rate(10)
        # yyy

        print("Starting new waypoint")
        position = self.curr_position
        rotation = self.curr_rotation
        last_rotation = 0
        linear_speed = 1 
        angular_speed = 1
        
        (goal_x, goal_y, goal_z) = self.getWaypoint()
        time.sleep(1)
        print("Got waypoints")
            
        if goal_z > 180 or goal_z < -180:
                print("you input wrong z range.")
                self.shutdown()
        
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance



        while distance > 0.05:
            position = self.curr_position
            rotation = self.curr_rotation
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)
	    
            '''
            Get the lidar distance using get_scan() function and calculate min distance
	    update the _distance variable.
	    '''
            # xxx
            # xxx
            # if _distance < SAFE_STOP_DISTANCE and _distance > 0:
                # move_cmd.linear.x = 0.0
                # move_cmd.angular.z = 0.0
                # rospy.loginfo('Obstacle detected!')
            # else:
                # '''
                # Use the logic of the *_pointop_key.py code here to move the robot from one position to another
		# '''
        # yyy
            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        position, rotation = self.curr_position, self.curr_rotation

        while abs(rotation - goal_z) > 0.05:
            position, rotation = self.curr_position, self.curr_rotation
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()


        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        # yyy


    def getkey(self):
        x, y, z = raw_input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z


    def getWaypoint(self):
        val = self.waypoints[self.waypoint_counter]
        self.waypoint_counter+=1
        
        x = val[0]
        y = val[1]
        z = val[2]
        print(x, y, z)
        return x, y, z


    def get_position_cb(self, msg):
        '''
        Update the self.curr_position and self.curr_rotation in this callback function to the position_topic.
        Reference: 
        1. get_odom() function in the *_pointop_key.py
        2. "Writing a subscriber" section in http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29"
        3. HW1 turtles_assemble.py	 
        '''  

        # yyy      
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        rpy = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.curr_position.x = x
        self.curr_position.y = y
        self.curr_rotation = rpy[2]
        # yyy
        


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--position_topic', type=str)
    parser.add_argument('--velocity_topic', type=str)
    parser.add_argument('--scan_topic', type=str)
    args=parser.parse_args()

    try:
        obj = GotoPoint(args.position_topic, args.velocity_topic, args.scan_topic)
        while not rospy.is_shutdown():
            #print(print_msg)
            obj.go_to_waypoint()
    except Exception as e: 
        print(e)
        rospy.loginfo("shutdown program now.")
