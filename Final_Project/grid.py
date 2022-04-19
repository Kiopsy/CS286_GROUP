from constants import c
from robot import Robot

class Grid:
    def __init__(self):
        self.robots = []
        self.grid = [[]]
        
    def update_grid(self): 

        rob = self.robots[0]
        rob.sense()

        for y in range(len(self.grid)):
            for x in range(len(self.grid[0])):
                pt = (x, y)
                if pt == rob.pos:
                    print("R", end="")
                elif pt in rob.path:
                    print(".", end = "")
                elif pt in rob.seen:
                    if rob.seen[pt] == c.FREE:
                        print(" ", end="")
                    elif rob.seen[pt] == c.WALL:
                        print("W", end="")
                else:
                    print("-", end='')
            print()

        return rob.explore()
        

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

        # Create robot list
        for x, y in robot_pos:
            self.robots.append(Robot(x, y, grid))

    def plot_grid(self): 
        for row in self.grid:
            for col in row:
                print(col, end='')
            print()