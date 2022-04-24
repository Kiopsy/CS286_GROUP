import rospy
import numpy as np
import pandas as pd
import tf

import sys

import actionlib
from constants import c
from grid import Grid
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler

from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal


counter = 0

class ROSMap:
    def __init__(self, map_msg):
        self.origin = (-map_msg.info.origin.position.y, map_msg.info.origin.position.x)
        self.origin_theta = map_msg.info.origin.orientation.z

        self.w, self.h = map_msg.info.width, map_msg.info.height # the width and height of map in terms of number of cells
        self.resolution = map_msg.info.resolution # the edge size of a cell in meters 

        map_arr = np.array(map_msg.data, np.float) # 1D array describing map
        self.grid = map_arr.reshape((self.h, self.w)) # reshape to 2D array
    
    # convert grid coordinates to real coordinates
    def grid_to_real(self, pos):
        gx, gy = pos 
        gx = -gx

        # find location (x,y) in meters relative to the map
        mx, my = gx * self.resolution, gy * self.resolution 
        ox, oy = self.origin

        # shift by map origin to get location with respect to the global frame
        return (ox + mx, oy + my)

    # convert real coordinates to grid coordinates
    def real_to_grid(self, pos):
        rx, ry = pos # real coordinates
        ox, oy = self.origin

        mx, my = rx - ox, ry - oy  # calculate position relative to the map origin 
        ix, iy = int(-rel_x/self.resolution), int(rel_y/self.resolution)

        return (ix, iy) # we need to index into the grid as grid[iy][ix]

    def plot(self, robot_pos=None, frontier_pos=None, real_pos=True):
        plot_map = np.copy(self.grid)

        # add robot position to the map
        if not robot_pos is None:
            if real_pos:
                robot_pos = self.real_to_grid(robot_pos)
            iy, ix = robot_pos
            for i in range(5):
                for j in range(5):
                    try:
                        plot_map[ix + i][iy+j] = 101.0
                    except:
                        print("out of bounds")
        
        # add frontier position to the map
        if not frontier_pos is None:
            if real_pos:
                frontier_pos = self.real_to_grid(frontier_pos)
            iy, ix = frontier_pos
            for i in range(5):
                for j in range(5):
                    try:
                        plot_map[ix + i][iy+j] = 3000.0
                    except:
                        print("out of bounds")
        
        plot_map = np.flipud(plot_map)
            
        # plot map
        cmap = mpl.colors.ListedColormap(['blue', 'white','black', 'red', 'yellow'])
        bounds = [-1.0, 0.0, 5.0, 101.0, 3000.0, 10000.0]
        norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
        
        # tell imshow about color map so that only set colors are used
        img = plt.imshow(plot_map, interpolation='nearest', cmap=cmap,norm=norm)
        
        # make a color bar 
        plt.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])
        
        plt.show()

# RobotDriver
# this class manages all the ROS interactions, reading maps,
# reading robot positions, and sending commands to the robot
class RobotDriver:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        # initialize everything required for moving the robot to a goal
        self.client = actionlib.SimpleActionClient("/"+ str(self.robot_name) + "/move_base", MoveBaseAction)
        self.timeout = 60.0 # in seconds
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = str(self.robot_name) + "/map"

        #Create the actionlib server
        print("Waiting for action lib server: /" + str(self.robot_name) + "/move_base")
        self.client.wait_for_server()

        self.map = self.cost_map = None
        
        self.costmap_sub = rospy.Subscriber('/' + str(robot_name) + '/move_base/global_costmap/costmap', OccupancyGrid, self.parse_costmap)
        self.map_sub = rospy.Subscriber('/' + str(robot_name) + '/rtabmap/grid_map', OccupancyGrid, self.parse_map)
        self.pos_sub = rospy.Subscriber('/' + str(robot_name) + '/mobile_base/odom', Odometry, self.parse_odom)

        self.pos = None # tuple (x,y) in real coordinates
    
    # the callback for the map subscriber
    def parse_map(self, msg):
        self.map = ROSMap(msg)
    
    # the callback for the costmap sub
    def parse_costmap(self, msg):
        self.costmap = ROSMap(msg)

    # the callback for the odometry subscriber
    def parse_odom(self, msg):
        self.pos = (msg.pose.pose.position.x, -msg.pose.pose.position.y)

    # the method for sending goal to the robot
    def send_goal(self, pos, angle):
        x, y = pos

        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.goal.target_pose.pose.position.x = float(y)
        self.goal.target_pose.pose.position.y = float(-x)
        self.goal.target_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0,0,float(angle))

        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]
        
        rospy.loginfo("Attempting to move to the goal")
        print("Goal: ", self.goal)

        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result(rospy.Duration(self.timeout))

    # use this to get the map, because it waits until the map is initialized
    def get_map(self):
        num_sleep = 0
        while self.map is None:
            print("[{}] map is not initialized! going to sleep...".format(num_sleep))
            rospy.sleep(1)
            num_sleep += 1
            if rospy.is_shutdown():
                sys.exit(1)
                break
        return self.map
    
    # use this to get the costmap, because it waits until the costmap is initialized
    def get_costmap(self):
        num_sleep = 0
        while self.costmap is None:
            print("[{}] costmap is not initialized! going to sleep...".format(num_sleep))
            rospy.sleep(1)
            num_sleep += 1
            if rospy.is_shutdown():
                sys.exit(1)
                break
        return self.costmap
    
    # use this to get the pos, because it waits until the pos is initialized
    def get_pos(self):
        num_sleep = 0
        while self.pos is None:
            print("[{}] position is not initialized! going to sleep...".format(num_sleep))
            rospy.sleep(1)
            num_sleep += 1
            if rospy.is_shutdown():
                sys.exit(1)
                break
        return self.pos

class PhysicalMap:
    def __init__(self, robot_names):
        #self.m = MoveActions(robot_name)
        #self.robot_name = robot_name 
        # ADDED
        self.robot_names = robot_names
        
        # ADDED
        self.frontiers = []

    def find_frontiers(self):
        g = Grid()
        robot_positions = self.get_loc_in_grid()
        print("RPOS", robot_positions)
        if len(robot_positions) != 0:
            g.define_grid(self.get_grid(), robot_positions)
        
            for robot in g.robots:
                y, x = g.get_frontier_pos(robot)
                frontier = x, y
                if not frontier is None:
                    self.frontiers.append(frontier)
                    print("frontier: ", frontier)

    # get the robot location in the grid, in the format (i,j)
    # where i is the row, j is the column
    # the robot cell is map[i][j]
   


def main():
    rospy.init_node("map_parser")

    ros_robot = RobotDriver("locobot1")
    # while True:
    print(ros_robot.get_map().grid)
    print(ros_robot.get_costmap().grid)
    print(ros_robot.get_pos())
    
    ros_robot.send_goal((1,0), 0)

    rospy.spin()

    # mp = PhysicalMap(["locobot3"])
    # while mp.map_origin is None:
    #     try:
    #         mp.find_frontiers()
    #     except ValueError as e:
    #         print(e)
        
    # #mp.plot()
    # # While there is some frontier left to explore
    # print("FRONTIERS INTI", mp.frontiers)
    # while any(not f is None for f in mp.frontiers):
    #     print("FRONTIERS", mp.frontiers)
    #     for i, move_action in enumerate(mp.move_actions):
    #         '''
    #         while mp.map2d is None or mp.cost_map is None or mp.merged_map is None:
    #             try:
    #                 # send robot position (to make robot stay in place)
    #                 move_action.client.sleep(0.5)
    #             except:
    #                 print("coulnd't send goal 1")
    #         '''

    #         if not mp.frontiers[i] is None:
    #             try:
    #                 print(self.goal)
    #                 move_action.client.send_goal(move_action.goal)
    #                 wait = move_action.client.wait_for_result(rospy.Duration(self.timeout))
    #             except:
    #                 print("coulnd't send goal 2")
    #     mp = PhysicalMap(["locobot3"])
    #     mp.find_frontiers()


    # '''
    # maps = [PhysicalMap("locobot3")]
    # #global_map_sub = TODO
    # # Will need to add a map_sub and cost_map_sub arg (for global map) to PhysicalMap object
    # maps = [PhysicalMap("locobot3", global_map_sub, global_cost_map_sub), PhysicalMap("locobot4", global_map_sub, global_cost_map_sub)]
    # for map in maps:
    #     client = map.m.client
    #     while map.map2d is None or map.cost_map is None or map.merged_map is None:
    #         try:
    #             # send robot position (to make robot stay in place)
    #             client.sleep()
    #         except:
    #             print("coulnd't send goal"

    #     if not gx is None:
    #         try:
    #             self.m.send_goal((gx,gy), 0)
    #         except:
    #             print("coulnd't send goal")
    
    # '''

    # #TODO (04/22)
    # # Clean up error flow/logic, especially when origin isn't found/robot pos are unknown. Do we want to return empty list, [None, ...], or something else?
    # # Fix logic for initial
    # # Test with two robots, figure out exact topics


    # rospy.spin()


if __name__ == '__main__':
    main()

