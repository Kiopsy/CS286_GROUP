import rospy
import numpy as np
import pandas as pd
import tf

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

class PhysicalMap:
    def __init__(self, robot_name):
        rospy.init_node('map_parser_' + str(robot_name)) 
        self.m = MoveActions(robot_name)
        self.robot_name = robot_name 
        # ADDED
        self.mp_sub = rospy.Subscriber('/'+ str(robot_name) +'/rtabmap/grid_map', OccupancyGrid, self.parse_map_cb, queue_size=1)
        self.cost_mp_sub = rospy.Subscriber('/'+ str(robot_name) +'/move_base/global_costmap/costmap', OccupancyGrid, self.parse_cost_map_cb)
        self.loc_sub = rospy.Subscriber('/'+str(robot_name) + '/mobile_base/odom', Odometry, self.parse_loc_cb)
        self.timeout = 60.0
        self.w = None
        self.cost_w = None
        self.h = None
        self.cost_h = None
        self.map_origin = None
        self.cost_map_origin = None
        self.origin_theta = None
        self.cost_origin_theta = None
        self.pos_x, self.pos_y = None, None
        self.ix, self.iy = None, None
        self.mtr_per_cell_map = None
        self.cost_mtr_per_cell_map = None
        # ADDED
        self.map2d = None
        self.cost_map = None
        self.merged_map = None
    
    # ADDED
    def parse_cost_map_cb(self, msg):
        self.cost_map_origin = (-msg.info.origin.position.y, msg.info.origin.position.x)
        self.cost_origin_theta = msg.info.origin.orientation.z
        self.cost_w, self.cost_h = msg.info.width, msg.info.height # the width and height of map in terms of number of cells
        self.cost_mtr_per_cell_map = msg.info.resolution # the edge size of a cell in meters 
        print("set resolution: ", self.cost_mtr_per_cell_map) 
        map_arr = np.array(msg.data, np.float) # 1D array describing map
        self.cost_map = map_arr.reshape((self.cost_h, self.cost_w)) # reshape to 2D array
        self.merged_map = self.cost_map
        self.merged_map[self.map2d == c.UNEXPLORED] = c.UNEXPLORED

    def parse_map_cb(self, msg):
        global counter
        print(msg.info)
        self.map_origin = (-msg.info.origin.position.y, msg.info.origin.position.x)
        self.origin_theta = msg.info.origin.orientation.z
        self.w, self.h = msg.info.width, msg.info.height # the width and height of map in terms of number of cells
        self.mtr_per_cell_map = msg.info.resolution # the edge size of a cell in meters 
        print("set resolution: ", self.mtr_per_cell_map) 
        map_arr = np.array(msg.data, np.float) # 1D array describing map
        self.map2d = map_arr.reshape((self.h, self.w)) # reshape to 2D array
        #self.map2d = self.merged_map
        plot_map = self.map2d
        
        # if not self.merged_map is None:
        #     plot_map = self.merged_map
        


        counter += 1
        if counter % 1 == 0:
            locs = self.get_loc_in_grid()
            print("grid loc: ", locs)
            if locs != None:
                ix, iy = locs
                loc = self.grid_to_real((ix, iy))
                print((self.pos_x, self.pos_y), loc)
                for i in range(5):
                    for j in range(5):
                        try:
                            plot_map[ix + i][iy+j] = 101.0
                        except:
                            print("out of bounds")
            
            frontier = self.find_frontier()
            gx, gy = None, None
            if not frontier is None:
                fx, fy = frontier # in grid, map[fx][fy]
                for i in range(5):
                    for j in range(5):
                        plot_map[fx + i][fy + j] = 3001.0
                gx, gy = self.grid_to_real((fx, fy))
#            if counter % 4 == 0:
#                pd.DataFrame(plot_map).to_csv("map" + str(counter/4)+".csv")

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
            

            # ADDED
            if not gx is None:
               self.m.send_goal((gx,gy), 0)

            # if not self.map2d is None or not self.cost_map is None or not self.merged_map is None and not gx is None:
            
                
            #         try:
            #             self.m.send_goal((gx,gy), 0)
            #             '''
            #             for waypoint in frontier:
            #                 waypoint = fx, fy
            #                 gx, gy = self.grid_to_real((fx, fy))
            #             '''
            #         except:
            #             print("coulnd't send goal")


    def parse_loc_cb(self, msg):
        # this position is with respect to /odom
        self.pos_y, self.pos_x = msg.pose.pose.position.x, -msg.pose.pose.position.y
        # parse into grid position
        # print("robot positions: ", self.pos_x, self.pos_y)

    def find_frontier(self):
        g = Grid()
        if self.get_loc_in_grid() == None:
            return None
        ix, iy = self.get_loc_in_grid()
        robot_positions = [(iy, ix)]
        g.define_grid(self.get_grid(), robot_positions)
        
        frontier = g.get_frontier_pos(g.robots[0])
        if not frontier is None:
            y, x = frontier
            frontier = (x,y)
            '''
            cost_map = self.cost_map
            if cost_map[y][x] > SOME_THRESHOLD:
                return frontier
            else:
                new_grid = self.get_grid()
                new_grid[y][x] = 0
                self.find_frontier(new_grid)
                # Change def to include new_grid and define g with new_grid (maybe redefine self.map2d?)
            '''

            print("frontier: ", frontier)
            return frontier

    # get the robot location in the grid, in the format (i,j)
    # where i is the row, j is the column
    # the robot cell is map[i][j]
    def get_loc_in_grid(self):
        if self.map_origin == None:
            print("map origin unknown")
            return None
        
        if self.pos_x == None:
            print("robot pos unknown")
            return None

        if self.mtr_per_cell_map == None:
            print("map resolution unknown")
            return None
        ox, oy = self.map_origin
        rel_x, rel_y = self.pos_x - ox, self.pos_y - oy  # calculate position relative to the map origin 
        
        print("origin: ", ox, oy)
        print("pos: ", self.pos_x, self.pos_y)
        res = self.mtr_per_cell_map
        return (int(-rel_x/res), int(rel_y/res)) # in terms of integer

    def get_grid(self):
        # ADDED
        # return self.merged_map
        # Uncomment below if not using merged map (cost map with unknowns)
        return self.map2d

    # convert a grid coordinate (i,j) into real coordinates
    # where i is the row, j is the column
    # returns coordinates (in meters) in the frame robot_name/map
    def grid_to_real(self, loc):
        lx, ly = loc 
        lx = -lx

        if self.mtr_per_cell_map == None:
            print("map resolution unknown")
            return None
        if self.map_origin == None:
            print("map origin unknown")
            return None
        mx, my = lx * self.mtr_per_cell_map, ly * self.mtr_per_cell_map 
        print("mx:", mx, my)

        ox, oy = self.map_origin
        pos_x, pos_y = ox + mx, oy + my
        
        return (pos_x, pos_y)

class MoveActions:
    def __init__(self, robot_name):

        self.robot_name = robot_name
        self.client = actionlib.SimpleActionClient("/"+str(robot_name) + "/move_base", MoveBaseAction)
        self.timeout = 60 #secs
        self.step_size = 1.0
        
        #Create the actionlib server
        print("Waiting for action lib server")
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = robot_name + "/map"

        self.pub = rospy.Publisher("/" + str(self.robot_name)+"/move_base/current_goal", PoseStamped, queue_size=1)

    # send goal to the robot, in real coordinates
    # the coordinate frame is robot_name/map
    def send_goal(self, pos, angle):
        x, y = pos

        # p = PoseStamped()
        # p.header.frame_id = str(self.robot_name) + '/map'
        # p.pose.position.x = float(y)
        # p.pose.position.y = float(-x)
        # p.pose.position.z = 0.0

        # q = quaternion_from_euler(0,0,float(angle))

        # p.pose.orientation.x = q[0]
        # p.pose.orientation.y = q[1]
        # p.pose.orientation.z = q[2]
        # p.pose.orientation.w = q[3]
        
        # p.header.stamp = rospy.Time.now()

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = float(y)
        self.goal.target_pose.pose.position.y = float(-x)
            
        q = quaternion_from_euler(0,0,float(angle))

        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]

        print("goal:\n", self.goal)
        # print("goal: ", p)
        rospy.loginfo("Attempting to move to the goal")
        # self.client.cancel_all_goals()
        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result(rospy.Duration(self.timeout))



        # try:
        #     self.pub.publish(p)
        # except:
        #     print("cant publish")
        #     # print("Published to goal topic")
        #     # wait = self.client.wait_for_result(rospy.Duration(self.timeout))
        #     # if not wait:
        #         # print("couldn't move to goal")
    


def main():

    mp = PhysicalMap("locobot3")
    '''
    # ADDED
    maps = [PhysicalMap("locobot3", "locobot4")]
    global_map_sub = TODO
    # Will need to add a map_sub and cost_map_sub arg (for global map) to PhysicalMap object
    maps = [PhysicalMap("locobot3", global_map_sub, global_cost_map_sub), PhysicalMap("locobot4", global_map_sub, global_cost_map_sub)]
    for map in maps:
        frontier = map.find_frontier()
        gx, gy = None, None
        if not frontier is None:
            fx, fy = frontier # in grid, map[fx][fy]
            for i in range(5):
                for j in range(5):
                    plot_map[fx + i][fy + j] = 3001.0
            gx, gy = self.grid_to_real((fx, fy))
#           if counter % 4 == 0:
#               pd.DataFrame(plot_map).to_csv("map" + str(counter/4)+".csv")

        plot_map = np.flipud(plot_map)
        
        # plot map
        cmap = mpl.colors.ListedColormap(['blue', 'white','black', 'red', 'yellow'])
        bounds = [-1.0, 0.0, 5.0, 101.0, 3000.0, 10000.0]
        norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
        
        # tell imshow about color map so that only set colors are used
        img = plt.imshow(plot_map,interpolation='nearest', cmap = cmap,norm=norm)
        
        # make a color bar
            
        plt.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])
        
        plt.show()

        client = map.m.client
        while map.map2d is None or map.cost_map is None or map.merged_map is None:
            try:
                # send robot position (to make robot stay in place)
                client.sleep()
            except:
                print("coulnd't send goal"

        if not gx is None:
            try:
                self.m.send_goal((gx,gy), 0)
            except:
                print("coulnd't send goal")
    
    '''
    
    rospy.spin()


if __name__ == '__main__':
    main()

