import rospy
import numpy as np
import tf
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
import matplotlib as mpl
import matplotlib.pyplot as plt

class PhysicalMap:
    def __init__(self, robot_name):
        self.robot_name = robot_name 
        rospy.init_node('map_parser_' + str(robot_name))
        self.mp_sub = rospy.Subscriber('/'+ str(robot_name) +'/rtabmap/grid_map', OccupancyGrid, self.parse_map_cb)
        self.loc_sub = rospy.Subscriber('/'+str(robot_name) + '/mobile_base/odom', Odometry, self.parse_loc_cb)
         
        self.map_origin = None
        self.pos_x, self.pos_y = None, None
        self.ix, self.iy = None, None
        self.mtr_per_cell_map = None

    def parse_map_cb(self, msg):
        print(msg.info)
        self.map_origin = (-msg.info.origin.position.y, msg.info.origin.position.x)
        self.origin_theta = msg.info.origin.orientation.z
        self.w, self.h = msg.info.width, msg.info.height # the width and height of map in terms of number of cells
        self.mtr_per_cell_map = msg.info.resolution # the edge size of a cell in meters 
        print("set resolution: ", self.mtr_per_cell_map) 
        map_arr = np.array(msg.data, np.float) # 1D array describing map
        self.map2d = map_arr.reshape((self.h, self.w)) # reshape to 2D array
        plot_map = self.map2d

        locs = self.get_loc_in_grid()
        print("grid loc: ", locs)
        if locs != None:
            ix, iy = locs
            for i in range(10):
                for j in range(10):
                    plot_map[ix + i][iy+j] = 1000
        
        plot_map = plot_map.T

        # plot map
        cmap = mpl.colors.ListedColormap(['blue', 'white','black', 'red'])
        bounds = [-1, 0, 50, 101, 10000]
        norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
        
        # tell imshow about color map so that only set colors are used
        img = plt.imshow(self.map2d,interpolation='nearest', cmap = cmap,norm=norm)
    
        # make a color bar
        
        plt.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])
        
        plt.show()

    def parse_loc_cb(self, msg):
        # this position is with respect to /odom
        self.pos_y, self.pos_x = msg.pose.pose.position.x, -msg.pose.pose.position.y
        # parse into grid position
        # print("positions: ", self.pos_x, self.pos_y)

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
        res = self.mtr_per_cell_map
        print("grid loc: ", int(rel_x/res), int(rel_y/res))
        return (int(rel_x/res), int(rel_y/res)) # in terms of integer

    def get_grid(self):
        return self.map2d

def main():
    mp = PhysicalMap("locobot2")
    rospy.spin()


if __name__ == '__main__':
    main()
