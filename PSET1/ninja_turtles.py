'''
Harvard CS 286 Spring 2022
'''

'''
Import the modules required for using ROS turtlesim services.
'''
#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.srv import TeleportAbsolute

class NinjaTurtles:
    def __init__(self, x, y, name):
        self.x = x
        self.y = y
        self.name = name
        self.kill = rospy.ServiceProxy("/kill", Kill)
        self.spawn = rospy.ServiceProxy("/spawn", Spawn)
        self.teleport = rospy.ServiceProxy("/" + str(name) + "/teleport_absolute", TeleportAbsolute)

    def remove_bot(self,name='turtle1'):
        '''
        Use the turtlesim ROS package service that will remove the default turtle in turtlesim
        '''  
        try:
            self.kill(name)
        except rospy.ServiceException as exc:
            print("Turtle "+ self.name +" already removed")

    def add_bot(self):
        '''
        Use the turtlesim ROS package service that will spawn a new turtle
        '''
        try:
            self.spawn(self.x, self.y, 0.0, self.name)
        except rospy.ServiceException as exc:
            print("Turtle "+ self.name +" already exists")

    def go_to(self,new_x=0, new_y=0):
        '''
        Use the turtlesim ROS package service that enables a turtle to 
        directly teleport to a new location (new_x,new_y)
        '''
        """Moves the turtle to the goal."""
        self.teleport(new_x, new_y, 0.0)

            
if __name__ == "__main__":
    rospy.init_node("Cowabunga")
    t1 = NinjaTurtles(1,4,'t1')
    t1.remove_bot('turtle1')
    t1.add_bot()
    try: 
        t1.go_to(5,5)
    except rospy.ROSInterruptException:
        pass