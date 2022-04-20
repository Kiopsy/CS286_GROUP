from constants import c
from robot import Robot
import numpy as np
import copy 

class Grid:
    def __init__(self):
        self.robots = []
        self.grid = [[]]
        self.dynamic_grid = [[]]
        
    def update_grid(self): 
        rob = self.robots[0]

        rob.sense()
        rob.get_frontier_path()

        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                pt = (x, y)
                if pt == rob.pos:
                    print("R", end="")
                    self.dynamic_grid[y][x] = "R"
                elif rob.frontier == pt:
                    print("F", end="")
                    self.dynamic_grid[y][x] = "F"
                elif pt in rob.path:
                    print(".", end = "")
                    self.dynamic_grid[y][x] = "."
                elif pt in rob.prev_path:
                    print("*", end = "")
                    self.dynamic_grid[y][x] = "*"
                elif pt in rob.seen:
                    if rob.seen[pt] == c.FREE:
                        print(" ", end="")
                        self.dynamic_grid[y][x] = " "
                    elif rob.seen[pt] == c.WALL:
                        print("W", end="")
                        self.dynamic_grid[y][x] = "W"
                else:
                    print("-", end='')
                    self.dynamic_grid[y][x] = "-"
            print()

        return rob.explore()
        

    def read_Grid(self, filename):
        grid = []

        robot_pos = []

        '''
        with open(filename) as f:
            int_grid = np.genfromtxt(filename, dtype=int)
            str_grid = np.full_like(int_grid, " ", dtype=str)
            str_grid[int_grid >= c.WALL_THRESH] = c.WALL
            str_grid[int_grid == c.ROBOT_INT] = c.ROBOT

            robot_pos = np.where(str_grid == c.ROBOT)
            robot_pos = list(zip(robot_pos[0], robot_pos[1]))

            self.grid = str_grid.tolist()
            self.dynamic_grid = copy.deepcopy(str_grid.tolist())

            # Create robot list
            for x, y in robot_pos:
                self.robots.append(Robot(int(x), int(y), grid))
        '''

        # Read file
        #with open(filename) as f:
            #lines = f.readlines()

        int_grid = np.genfromtxt(filename, dtype=int).tolist()

        for y, row in enumerate(int_grid):
            temp = []
            
            #row = row.replace(' ', '')
            #row = row.replace('\n', '')
            for x, col in enumerate(row):
                if col == c.ROBOT_INT:
                    temp.append(c.ROBOT)
                    r = (x, y)
                    robot_pos.append(r)
                elif col >= c.WALL_THRESH:
                    temp.append(c.WALL)
                else:
                    temp.append(c.FREE)
                    
            grid.append(temp)
        
        # Create grid
        self.grid = grid
        self.dynamic_grid = copy.deepcopy(grid)

        # Create robot list
        for x, y in robot_pos:
            self.robots.append(Robot(x, y, grid))

    def plot_grid(self): 
        for row in self.grid:
            for col in row:
                print(col, end='')
            print()