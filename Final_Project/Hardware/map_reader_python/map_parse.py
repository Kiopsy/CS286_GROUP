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
        
        self.tl = tf.TransformListener()
        rospy.Subscriber("/" + str(self.robot_name) + "/map", PointStamped, self.some_message_handler)

        self.pos_x, self.pos_y = None, None
        self.ix, self.iy = None, None
        self.mtr_per_cell_map = None

    def parse_map_cb(self, msg):
        # print(msg.info)
        self.w, self.h = msg.info.width, msg.info.height # the width and height of map in terms of number of cells
        self.mtr_per_cell_map = msg.info.resolution # the edge size of a cell in meters 
        print("set resolution: ", self.mtr_per_cell_map) 
        map_arr = np.array(msg.data, np.float) # 1D array describing map
        self.map2d = map_arr.reshape((self.h, self.w)) # reshape to 2D array
        plot_map = self.map2d

        locs = self.get_loc_in_grid()
        if locs != None:
            ix, iy = locs
            for i in range(5):
                for j in range(5):
                    plot_map[iy + i][ix+j] = 1000

        # plot map
        cmap = mpl.colors.ListedColormap(['blue', 'white','black', 'green'])
        bounds=[-1, 0,50, 101, 10000]
        # norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
        
        # tell imshow about color map so that only set colors are used
        # img = plt.imshow(self.map2d,interpolation='nearest', cmap = cmap,norm=norm)
    
        # make a color bar
        
        # plt.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])
        
        # plt.show()

    def parse_loc_cb(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.pose = msg.pose.pose
        pose_stamped.header.frame_id = "/" + str(self.robot_name) + "/odom"
        
        print(new_pose)
        self.pos_y, self.pos_x = msg.pose.pose.position.x, -msg.pose.pose.position.y
        # parse into grid position
        print("positions: ", self.pos_x, self.pos_y)
        if self.mtr_per_cell_map == None:
            print("map resolution not received!")
            self.ix = None
            self.iy = None
            return None
        ix, iy = self.pos_x/self.mtr_per_cell_map, self.pos_y/self.mtr_per_cell_map
        self.ix, self.iy = int (ix), int(iy)
        

    def get_loc_in_grid(self):
        if self.ix == None:
            print("grid loc unknown")
            return None
        return (self.ix, self.iy) 

    def get_grid(self):
        return self.map2d

def main():
    mp = PhysicalMap("locobot4")
    rospy.spin()


if __name__ == '__main__':
    main()

