from constants import c
from robot import Robot
import numpy as np
import copy 

class Grid:
    def __init__(self):
        self.robots = []
        self.grid = np.array([[]])
        self.dynamic_grid = np.array([[]])
        
    # def update_grid(self): 
    #     rob = self.robots[0]

    #     # rob.sense()
    #     rob.get_frontier_path()
    #     rob.explore()

    def get_frontier_pos(self):
        rob = self.robots[0]
        rob.get_frontier_path()
        rob.explore()


    def define_grid(self, grid, robots):
        self.grid = grid

        known = np.where(grid != c.UNEXPLORED)
        known = list(zip(known[1], known[0]))

        seen = dict()

        for x, y in known: 
            seen[(x,y)] = grid[x][y]

        for x, y in robots: 
            r = Robot(x, y, grid)
            r.seen = seen
            self.robots.append(Robot(x, y, grid))
