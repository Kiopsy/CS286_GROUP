import rospy
import numpy as np
import pandas as pd
import tf
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
import matplotlib as mpl
import matplotlib.pyplot as plt

from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal


counter = 0

class PhysicalMap:
    def __init__(self, robot_name):
        self.robot_name = robot_name 
        rospy.init_node('map_parser_' + str(robot_name))
        self.mp_sub = rospy.Subscriber('/'+ str(robot_name) +'/rtabmap/grid_map', OccupancyGrid, self.parse_map_cb)
        self.loc_sub = rospy.Subscriber('/'+str(robot_name) + '/mobile_base/odom', Odometry, self.parse_loc_cb)
        self.pos_pub = rospy.Publisher('/' + str(robot_name)+'/move_base/current_goal', Pose)

        self.map_origin = None
        self.pos_x, self.pos_y = None, None
        self.ix, self.iy = None, None
        self.mtr_per_cell_map = None

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
        plot_map = self.map2d
        
        counter += 1
        if counter % 1 == 0:
            locs = self.get_loc_in_grid()
            print("grid loc: ", locs)
            if locs != None:
                ix, iy = locs
                for i in range(5):
                    for j in range(5):
                        plot_map[ix + i][iy+j] = 1000
            
            plot_map = np.flipud(plot_map)
            
            if counter % 4 == 0:
                pd.DataFrame(plot_map).to_csv("map" + str(counter/4)+".csv")

            # plot map
            cmap = mpl.colors.ListedColormap(['blue', 'white','black', 'red'])
            bounds=[-1, 0,50, 101, 10000]
            norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
            
            # tell imshow about color map so that only set colors are used
            img = plt.imshow(plot_map,interpolation='nearest', cmap = cmap,norm=norm)
            
            # make a color bar
             
            plt.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])
            
            plt.show()

    def parse_loc_cb(self, msg):
        # this position is with respect to /odom
        self.pos_y, self.pos_x = msg.pose.pose.position.x, -msg.pose.pose.position.y
        # parse into grid position
        # print("positions: ", self.pos_x, self.pos_y)

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
        return self.map2d

    # convert a grid coordinate (i,j) into real coordinates
    # where i is the row, j is the column
    # returns coordinates (in meters) in the frame robot_name/map
    def grid_to_real(self, loc):
        lx, ly = loc 
        lx = -lx

        if self.mtr_per_cell == None:
            print("map resolution unknown")
            return None
        if self.map_origin == None:
            print("map origin unknown")
            return None

        mx, my = lx * self.mtr_per_cell, ly*self.mtr_per_cell
        ox, oy = self.map_origin
        pos_x, pos_y = ox + mx, oy + my
        return (pos_x, pos_y)

class MoveActions:
    def __init__(self, robot_name):
        rospy.init_node(str(robot_name) + "_move_base")
        self.client = actionlib.SimpleActionClient(str(robot_name)+"/move_base/",MoveBaseAction)
        self.timeout = 60 #secs
        self.step_size = 1.0
        
        #Create the actionlib server
        print("Waiting for action lib server")
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = robot_name+"/map"
    
    # send goal to the robot, in real coordinates
    # the coordinate frame is robot_name/map
    def send_goal(self, pos, angle):
        x, y = pos
        
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = float(x)
        self.goal.target_pose.pose.position.y = float(y)
            
        q = quaternion_from_euler(0,0,float(angle))

        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]

        print("goal:\n", self.goal)
        rospy.loginfo("Attempting to move to the goal")
        self.client.send_goal(self.goal)
        
        # wait=self.client.wait_for_result(rospy.Duration(self.timeout)) 


def main():
    mp = PhysicalMap("locobot2")
    rospy.spin()


if __name__ == '__main__':
    main()

