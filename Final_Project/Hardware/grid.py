from constants import c
from robot import Robot
import numpy as np
import copy 

class Grid:
    def __init__(self):
        self.robots = []
        self.grid = np.array([[]])
        self.dynamic_grid = np.array([[]])
        
    def update_grid(self): 
        rob = self.robots[0]

        rob.sense()
        rob.get_frontier_path()

        for y in range(len(self.grid)):
            for x in range(len(self.grid[y])):
                pt = (x, y)
                if pt == rob.pos:
                    print("R", end="")
                    self.dynamic_grid[y][x] = c.ROBOT
                elif pt == rob.frontier:
                    print("F", end="")
                    self.dynamic_grid[y][x] = c.FREE
                elif pt in rob.path:
                    print(".", end = "")
                    self.dynamic_grid[y][x] = c.NEXT_PATH
                elif pt in rob.prev_path:
                    print("*", end = "")
                    self.dynamic_grid[y][x] = c.PREV_PATH
                elif pt in rob.seen:
                    if rob.seen[pt] == c.FREE:
                        print(" ", end="")
                        self.dynamic_grid[y][x] = c.FREE
                    elif rob.seen[pt] == c.WALL:
                        print("W", end="")
                        self.dynamic_grid[y][x] = c.WALL
                else:
                    print("-", end='')
                    self.dynamic_grid[y][x] = c.UNEXPLORED
            print()
        
        '''
        # Attempted vectorization below (does not print)

        grid = self.grid
        dynamic_grid = self.dynamic_grid

        dynamic_grid[grid == rob.pos] = c.ROBOT
        # Something weird with this
        dynamic_grid[grid == rob.frontier | (grid in rob.seen & rob.seen[grid] == c.FREE)] = c.FREE
        dynamic_grid[grid in rob.path] = c.NEXT_PATH
        dynamic_grid[grid in rob.prev_path] = c.PREV_PATH
        #dynamic_grid[grid in rob.seen & rob.seen[grid] == c.FREE] = c.FREE
        dynamic_grid[grid in rob.seen & rob.seen[grid] == c.WALL] = c.WALL
        dynamic_grid[not(grid == rob.pos 
                        | grid == rob.frontier | (grid in rob.seen & rob.seen[grid] == c.FREE) 
                        | grid in rob.path
                        | grid in rob.prev_path
                        | grid in rob.seen & rob.seen[grid] == c.WALL)] = c.UNEXPLORED
        '''



        return rob.explore()
        

    def read_Grid(self, filename):
        grid = []

        robot_pos = []


        with open(filename) as f:
            int_grid = np.genfromtxt(filename, dtype=int)
            print(int_grid)
            int_grid[int_grid >= c.WALL_THRESH] = c.WALL
            int_grid[int_grid == c.ROBOT_INT] = c.ROBOT
            print("NEW")
            print(int_grid)

            robot_pos = np.where(int_grid == c.ROBOT)
            robot_pos = list(zip(robot_pos[1], robot_pos[0]))

            self.grid = int_grid
            print(self.grid)
            self.dynamic_grid = copy.deepcopy(int_grid)

            # Create robot list
            for x, y in robot_pos:
                print("ROBOT X, Y", x, y)
                self.robots.append(Robot(int(x), int(y), int_grid))

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
        '''

    def plot_grid(self): 
        for row in self.grid:
            for col in row:
                print(col, end='')
            print()