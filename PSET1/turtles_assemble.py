'''
Harvard CS 286 Spring 2022
'''

from ninja_turtles import NinjaTurtles
import rospy
import numpy as np
import matplotlib.pyplot as plt
from turtlesim.msg import Pose

'''
Import the module to use msg corresponding to the turtles pose topic
'''

plt.style.use('seaborn-whitegrid')

class NinjaTurtlesViz:
    def __init__(self):
        self.don_data = [[], []]
        self.leo_data = [[], []]
        self.ralph_data = [[], []]
        self.mike_data = [[], []]


    def position_cb_don(self,msg):
        '''
        Store the position data in the msg in self.don_data
        '''
        self.don_data[0].append(msg.x)
        self.don_data[1].append(msg.y)

    def position_cb_leo(self,msg):
        '''
        Store the position data in the msg in self.leo_data
        '''
        self.leo_data[0].append(msg.x)
        self.leo_data[1].append(msg.y)


    def position_cb_ralph(self,msg):
        '''
        Store the position data in the msg in self.ralph_data
        '''
        self.ralph_data[0].append(msg.x)
        self.ralph_data[1].append(msg.y)


    def position_cb_mike(self,msg):
        '''
        Store the position data in the msg in self.mike_data
        '''
        self.mike_data[0].append(msg.x)
        self.mike_data[1].append(msg.y)

    def get_plot(self):
        '''
        Generat the final plot that show the trace of all robot positions
        '''
        
        fig = plt.figure()
        ax = plt.axes()

        ax.plot(self.don_data[0], self.don_data[1])
        ax.plot(self.leo_data[0], self.leo_data[1])
        ax.plot(self.ralph_data[0], self.ralph_data[1])
        ax.plot(self.mike_data[0], self.mike_data[1])

        plt.title("Ninja turtle Sensing")
        plt.show()
        

def main():
    print("Starting")
    rospy.init_node("Cowabunga_plots")
    viz = NinjaTurtlesViz()
    
    '''
    Create subscribers for each turtle's pose topic and use the approaporiate callback functions
    '''

    rospy.Subscriber("/t1/pose", Pose, viz.position_cb_don)
    rospy.Subscriber("/t2/pose", Pose, viz.position_cb_leo)
    rospy.Subscriber("/t3/pose", Pose, viz.position_cb_ralph)
    rospy.Subscriber("/t4/pose", Pose, viz.position_cb_mike)
    
    print("Press CTRL+C once the simulation ends to generate the plot")
    rospy.spin()
    viz.get_plot()

if __name__ == "__main__":
    main()
