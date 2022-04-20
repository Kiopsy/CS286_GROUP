from constants import c
from robot import Robot
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

        self.print_grid(rob)

        return rob.explore()

    def print_grid(self, rob):
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

    # Creates a grid from the real life test bed
    def testbed_Grid(self):
        grid = []

    # Reads the grid from a text file
    def read_Grid(self, filename):
        grid = []

        robot_pos = []

        # Read file
        with open(filename) as f:
            rows = f.readlines()

        for y, row in enumerate(rows):
            temp = []
            
            row = row.replace(' ', '')
            row = row.replace('\n', '')
            for x, col in enumerate(row): 
                temp.append(col)
                if(col == 'R'):
                    r = (x, y)
                    robot_pos.append(r)
                    
            grid.append(temp)
        
        # Create grid
        self.grid = grid
        self.dynamic_grid = copy.deepcopy(grid)

        # Create robot list
        for x, y in robot_pos:
            self.robots.append(Robot(x, y, grid))

