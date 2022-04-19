# Final Project Simulation

# from matplotlib.pyplot import grid


WALL = "X"
FREE =  "0"
ROBOT = "R"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def pprint(color, text, end="\n"):
    print(color + text + bcolors.ENDC, end = end)

class Robot: 
    def __init__(self, x, y, grid):
        self.pos = (x, y)
        self.radius = 8
        self.grid = grid
        self.seen = dict()
        self.frontiers = set()
    
    def get_frontier(self):
        Q = [self.pos]
        V = set()

        while len(Q) != 0:
            n = Q.pop(0)

            if n in V: continue
            else: V.add(n)

            x, y = n

            try:
                space = self.grid[y][x] 
            except:
                continue
            
            if space != WALL:

                if n not in self.seen:
                    return n

                Q.append((x - 1, y))
                Q.append((x + 1, y))
                Q.append((x, y - 1))
                Q.append((x, y + 1))
            else:
                pass

        return None
    
    def explore(self):
        frontier = self.get_frontier()

        if frontier: 
            self.pos = frontier
            return frontier
        else:
            print("Map finished")
            return None
    
    def sense(self):
        Q = [(self.pos, 0)]
        V = set()


        while len(Q) != 0:
            n, dist = Q.pop(0)

            if n in V: continue
            else: V.add(n)

            if dist > self.radius//2: break

            x, y = n

            try:
                space = self.grid[y][x]
            except:
                continue

            if space != WALL:

                self.seen[n] = FREE

                Q.append(((x - 1, y), dist + 1))
                Q.append(((x + 1, y), dist + 1))
                Q.append(((x, y - 1), dist + 1))
                Q.append(((x, y + 1), dist + 1))

                # Q.append(((x - 1, y - 1), dist + 1))
                # Q.append(((x + 1, y - 1), dist + 1))
                # Q.append(((x - 1, y + 1), dist + 1))
                # Q.append(((x + 1, y + 1), dist + 1))
            else:
                self.seen[n] = WALL 




class Grid:
    def __init__(self):
        self.robots = []
        self.grid = [[]]
        self.dynamic_grid = [[]]
        
    def update_grid(self): 

        rob = self.robots[0]
        rob.sense()

        for y in range(len(self.grid)):
            for x in range(len(self.grid[0])):
                pt = (x, y)
                if pt == rob.pos:
                    print("P", end="")
                elif pt in rob.seen:
                    if rob.seen[pt] == FREE:
                        print(" ", end="")
                    elif rob.seen[pt] == WALL:
                        print("W", end="")
                else:
                    print("-", end='')
            print()

        return rob.explore()
        

    def read_Grid(self, filename):
        grid = []
        dynamic_grid = []

        robot_pos = []

        # Read file
        with open(filename) as f:
            rows = f.readlines()

        for y, row in enumerate(rows):
            temp = []
            dynamic_temp = []
            
            row = row.replace(' ', '')
            row = row.replace('\n', '')
            for x, col in enumerate(row): 
                temp.append(col)
                dynamic_temp.append('-')
                if(col == 'R'):
                    r = (x, y)
                    robot_pos.append(r)
                    
            grid.append(temp)
            dynamic_grid.append(dynamic_temp)
        
        # Create grid
        self.grid = grid
        self.dynamic_grid = dynamic_grid

        # Create robot list
        for x, y in robot_pos:
            self.robots.append(Robot(x, y, grid))

    def plot_grid(self): 
        for row in self.grid:
            for col in row:
                print(col, end='')
            print()
                
def main():
    g = Grid()
    g.read_Grid('layout1.txt')
    # g.plot_grid()
    while True:
        x = g.update_grid()
        print()

        if x == None:
            break

if __name__ == "__main__":
    main()