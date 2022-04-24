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

    def get_frontier_pos(self, rob):
        rob.get_frontier_path()
        return rob.explore()
        # return rob.frontier

    def get_frontier(self, rob):
        return rob.get_frontier()

    def define_grid(self, grid, robots):
        self.grid = grid

        if grid is None:
            print("no grid")
            return

        known = np.where(grid != c.UNEXPLORED)
        # print(known)
        rows, cols = known
        known = list(zip(cols, rows))

       
        seen = dict()

        for col, row in known:     
            seen[(col,row)] = grid[row][col]
        
        for col, row in robots: 
            self.robots.append(Robot(col, row, grid, seen))
