import rospy

import numpy as np
import pandas as pd
import tf

import random
import sys
import copy
import math

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
from nav_msgs.srv import *

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
        row, col = pos # NOTE might need to change this order
        row = -row

        # find location (x,y) in meters relative to the map
        mx, my = row * self.resolution, col * self.resolution 
        ox, oy = self.origin

        # shift by map origin to get location with respect to the global frame
        return (ox + mx, oy + my)

    # convert real coordinates to grid coordinates
    def real_to_grid(self, pos):
        rx, ry = pos # real coordinates
        ox, oy = self.origin

        mx, my = rx - ox, ry - oy  # calculate position relative to the map origin 
        print("m cord: ", (mx, my))
        row, col = int(-round(mx/self.resolution)), int(round(my/self.resolution))

        return (row, col) # we need to index into the grid as grid[iy][ix]

    def merge(m,c_map):
        assert(m.w == c_map.w and m.h == c_map.h)
        c_map = copy.deepcopy(c_map)
        c_map.grid[m.grid == c.UNEXPLORED] = c.UNEXPLORED

        return c_map

    def plot(self, timestep=0, robot_pos=None, frontier_pos=None, real_pos=True):
        plot_map = np.copy(self.grid)

        # add robot position to the map
        if not robot_pos is None:
            if real_pos:
                robot_pos = self.real_to_grid(robot_pos)
                print("grid pos: ", robot_pos)
            row, col = robot_pos
            for i in range(row - 5, row + 5):
                for j in range(col - 5, col + 5):
                    try:
                        plot_map[i][j] = 101.0
                    except:
                        print("out of bounds")
        
        # add frontier position to the map
        if not frontier_pos is None:
            if real_pos:
                frontier_pos = self.real_to_grid(frontier_pos)
            row, col = frontier_pos
            for i in range(row - 5, row + 5):
                for j in range(col - 5, col + 5):
                    try:
                        plot_map[i][j] = 3001.0
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
        # plt.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])
        
        # plt.show()
        plt.savefig('Frames/Map_' + str(timestep) + '.png')

# RobotDriver
# this class manages all the ROS interactions, reading maps,
# reading robot positions, and sending commands to the robot
class RobotDriver:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        # initialize everything required for moving the robot to a goal
        self.client = actionlib.SimpleActionClient("/"+ str(self.robot_name) + "/move_base", MoveBaseAction)
        self.timeout = 20.0 # in seconds
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = str(self.robot_name) + "/map"

        #Create the actionlib server
        print("Waiting for action lib server: /" + str(self.robot_name) + "/move_base")
        self.client.wait_for_server()

        self.map = self.costmap = self.merged_map = None
        
        self.costmap_sub = rospy.Subscriber('/' + str(robot_name) + '/move_base/global_costmap/costmap', OccupancyGrid, self.parse_costmap)
        self.map_sub = rospy.Subscriber('/' + str(robot_name) + '/rtabmap/grid_map', OccupancyGrid, self.parse_map)
        self.pos_sub = rospy.Subscriber('/' + str(robot_name) + '/mobile_base/odom', Odometry, self.parse_odom)

        # rospy.wait_for_service(str(self.robot_name) + '/move_base/make_plan')
        # self.make_plan_serv = rospy.ServiceProxy(str(self.robot_name) + '/move_base/make_plan', GetPlan)

        self.tolerance = 0.1

        rospy.wait_for_service(str(self.robot_name) + '/rtabmap/get_plan')
        self.make_plan_serv = rospy.ServiceProxy(str(self.robot_name) + '/rtabmap/get_plan', GetPlan)

        self.pos = None # tuple (x,y) in real coordinates
    
    # the callback for the map subscriber
    def parse_map(self, msg):
        self.map = ROSMap(msg)
    
    # the callback for the costmap sub
    def parse_costmap(self, msg):
        self.costmap = ROSMap(msg)

    # the callback for the odometry subscriber
    def parse_odom(self, msg):
        self.pos = (-msg.pose.pose.position.y, msg.pose.pose.position.x)

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
        # print("Goal: ", self.goal)

        self.client.send_goal(self.goal)
        rospy.sleep(1.0)
        wait = self.client.wait_for_result(rospy.Duration(self.timeout))
        
        if not wait:
            print("didnt finish before timeout")
            return False
        else:
            print("finished goal")
            return True

    def goal_is_reachable(self, goal):
        request = GetPlanRequest()
        request.start.header.stamp = rospy.Time.now()
        request.start.header.frame_id = str(self.robot_name) + '/map'
        
        rx, ry = self.get_pos()
        request.start.pose.position.x = ry
        request.start.pose.position.y = -rx

        request.goal.header.stamp = rospy.Time.now()
        request.goal.header.frame_id = str(self.robot_name) + '/map'
        
        gx, gy = goal
        request.goal.pose.position.x = gy
        request.goal.pose.position.y = -gx

        request.tolerance = self.tolerance # meters in x/y
        rospy.logdebug("%s sending request: %s", self, request)
        while True:
            try:
                response = self.make_plan_serv(request)
                rospy.logdebug("%s received response: %s", self.robot_name, response)
                response.plan.header.frame_id = str(self.robot_name) + '/map'
                return len(response.plan.poses) > 0
            except rospy.ServiceException as e:
                rospy.logerr(e)
                rospy.sleep(1)
                if rospy.is_shutdown():
                    sys.exit(1)

    # return the i'th position in the path, if path shorter than i, returns last
    # if path does not exist, return None
    def get_point_on_path(self, goal, i):
        request = GetPlanRequest()
        request.start.header.stamp = rospy.Time.now()
        request.start.header.frame_id = str(self.robot_name) + '/map'
        
        rx, ry = self.get_pos()
        request.start.pose.position.x = ry
        request.start.pose.position.y = -rx

        request.goal.header.stamp = rospy.Time.now()
        request.goal.header.frame_id = str(self.robot_name) + '/map'
        
        gx, gy = goal
        request.goal.pose.position.x = gy
        request.goal.pose.position.y = -gx

        request.tolerance = self.tolerance # meters in x/y
        rospy.logdebug("%s sending request: %s", self, request)
        try:
            response = self.make_plan_serv(request)
            rospy.logdebug("%s received response: %s", self.robot_name, response)
            response.plan.header.frame_id = str(self.robot_name) + '/map'
            if len(response.plan.poses) > 0:
                pose = response.plan.poses[min(i, len(response.plan.poses)-1)].pose
                x, y = -pose.position.y, pose.position.x
                return (x,y)
            else:
                return None
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return None

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

    # use this to get the merged map
    def get_mergedmap(self):
        self.get_map()
        self.get_costmap()
        num_sleep = 0
        while self.map.w != self.costmap.w or self.map.h != self.costmap.h:
            print("[{}] map and costmap are not the same size ({} x {}) ({} x {})! going to sleep...".format(num_sleep, \
             self.map.w, self.map.h, self.costmap.w, self.costmap.h))
            rospy.sleep(1)
            num_sleep += 1
            if rospy.is_shutdown():
                sys.exit(1)
                break
        self.merged_map = ROSMap.merge(self.map, self.costmap)
        return self.merged_map

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

def main():
    rospy.init_node("map_parser")

    ros_robot = RobotDriver("locobot1")

    iter = 0
    num_wanders = 0

    while True:
        robot_pos = ros_robot.get_pos()
        print("robot_pos", robot_pos)
        m = ros_robot.get_mergedmap()

        g = Grid()
        row, col = m.real_to_grid(robot_pos) # convert pos to grid coordinates

        area = 3
        for i in range(row - area, row + area):
            for j in range(col - area, col + area):
                try:
                    m.grid[i][j] = 0.0
                except:
                    continue

        g.define_grid(m.grid, [(col, row)])

        frontiers = g.get_frontier(g.robots[0])
        def get_centroid(frontier):
            n = float(len(frontier))
            x_acc, y_acc = 0.0, 0.0
            for p in frontier:
                x_acc += float(p[0])
                y_acc += float(p[1])
            return (int(round(x_acc/n)), int(round(y_acc/n)))
        frontiers = sorted(frontiers, key= lambda f: len(f), reverse = True)
        centroids = map(get_centroid, frontiers)
        centroid_swap = map(lambda (row, col): (col, row), centroids)
        centroid_real = map(m.grid_to_real, centroid_swap)
        print(centroid_real)
        print("num wanders: {}".format(num_wanders))

        if num_wanders > 2:
            print("safe wander for 3 iterations")
            break
        
        if len(frontiers) == 0:
            print("no frontiers")
            while True:
                i = random.choice(range(16))
                angle = i * math.pi/8
                step_size = 0.3 # in meters
                tx, ty = step_size * math.cos(angle) + robot_pos[0], step_size * math.sin(angle) + robot_pos[1]
                if ros_robot.goal_is_reachable((tx, ty)):
                    print("safe wandering to goal [{}]: {}".format(num_wanders, (tx, ty)))
                    tangle = angle - math.pi/2
                    ros_robot.send_goal((tx, ty), tangle)
                    break
            num_wanders += 1
            continue
        
        # idx = 0
        # max_len = 0
        # for (i, f) in enumerate(frontiers):
        #     if len(f) > max_len:
        #         idx = i
        #         max_len = len(f)
        for f in centroids:
            fcol, frow = f
            # print("frontier:", m.grid_to_real((frow, fcol)))
            if not ros_robot.goal_is_reachable(m.grid_to_real((frow, fcol))):
                print("frontier not reachable")
                continue
                    
            # we know point is reachable at this point
            point = ros_robot.get_point_on_path(m.grid_to_real((frow, fcol)), 15)
            if point != None:
                x, y = point

                angle = math.atan2(y-robot_pos[1], x-robot_pos[0])
                angle -= math.pi/2.0
                print("found frontier")
                #m.plot(robot_pos=robot_pos, frontier_pos=m.grid_to_real((frow, fcol)), timestep=iter)
                if not ros_robot.send_goal((x,y), angle):
                    num_wanders += 1
                else:
                    num_wanders = 0
                    break
      
        if rospy.is_shutdown():
            sys.exit(0)
        iter += 1

    print("Map Finished")   
    sys.exit(0)

if __name__ == '__main__':
    main()

