import math
from bresenham import bresenham
from astar import astar
from constants import c

SENSING_RADIUS = 4
FRONTIER_SIZE = 40

class Robot: 
    def __init__(self, x, y, grid, seen):
        self.pos = (x, y)
        self.radius = SENSING_RADIUS
        self.side = 2 * self.radius + 1
        self.grid = grid
        self.seen = seen
        self.path = set()
        self.prev_path = set()
        self.frontier = None
    
    def is_big(self, pt):

        x, y = pt

        size = FRONTIER_SIZE

        half = size // 2

        new_points = total = 0.0

        for i in range(x - half, x + half):
            for j in range(y - half, y + half):
                try:
                    space = self.grid[j][i]
                except:
                    continue
            
                if (i, j) not in self.seen:
                    new_points += 1.0
                total += 1.0
        
        return new_points >= total*0.9



        
    def get_frontier(self):
        Q = [self.pos]
        V = set()

        # for i in range(self.grid.shape[0]):
        #     for j in range(self.grid.shape[1]):
        #         if self.is_big((j,i)):s
        #             print("big: ", (j,i))
        #             return (j,i)
        # print("couldn't find big")

        while len(Q) != 0:
            n = Q.pop(0)

            if n in V: continue
            else: V.add(n)

            x, y = n

            try:
                space = self.grid[y][x] 
            except:
                continue
            
            if space != c.WALL:

                if n not in self.seen and self.is_big(n):
                    print("big: ", n)
                    return n

                # NOTE: May need to handle case where edges are not walls
                if x > 0:
                    Q.append((x - 1, y))
                Q.append((x + 1, y))
                if y > 0:
                    Q.append((x, y - 1))
                Q.append((x, y + 1))

                # Q.append((x - 1, y - 1))
                # Q.append((x + 1, y - 1))
                # Q.append((x - 1, y + 1))
                # Q.append((x + 1, y + 1))
            else:
                pass
        
        return None

    def get_frontier_path(self):
        self.frontier = self.get_frontier()
        self.prev_path = self.path
        self.path = self.create_path(self.frontier)

    # Return euclidean distance
    def distance(self, point1, point2):
        return math.sqrt(math.pow(point2[0] - point1[0], 2) + math.pow(point2[1] - point1[1], 2))


    def sort_path(self, points, start):
        # Create new list for sorted path
        sorted_path = [start]
        points.remove(start)
        # Sort points based on shortest distance between one point and another, appending as needed
        while points:
            nearest_point = min(points, key=lambda x: self.distance(sorted_path[-1], x))
            sorted_path.append(nearest_point)
            points.remove(nearest_point)
        # Return list for sorted path
        return sorted_path

    def create_path(self, goal):
        if goal:
            goal = goal[::-1]
            start = self.pos[::-1]
            path = astar(self.grid, start, goal)

            for i, pt in enumerate(path):
                path[i] = pt[::-1]
            
            path = set(path)
        else:
            path = set()

        return path

    def explore(self):
        if self.frontier:
            #self.pos = self.frontier # teleport robot
            sorted_path = self.sort_path(self.path, self.pos)
            # self.pos = sorted_path[-1]
            # return self.frontier
            try:
                cell = sorted_path[-10]
            except:
                cell = sorted_path[-1]
            return cell
        else:
            print("Map finished")
            return None
    
    def raycast(self):
        x, y = self.pos

        points = []

        dist = self.radius

        num_points = self.side * 4 - 4

        base_theta = 2 * math.pi / num_points

        for i in range(num_points):
            
            theta = i * base_theta

            dx = round(dist * math.cos(theta))
            dy = round(dist * math.sin(theta))

            pt = (x + dx, y + dy)

            points.append(pt)
        
        return points
    
    def bresenham(self, p2):
        points = []

        x1, y1 = self.pos
        x2, y2 = p2

        intermediates = list(bresenham(x1, y1, x2, y2))
        for p in intermediates:
            x, y = p
            try:
                space = self.grid[y][x]
            except:
                break

            if space != c.WALL:
                self.seen[(x,y)] = c.FREE
            else:
                self.seen[(x,y)] = c.WALL
                break


            points.append((x, y))
        return points

    def sense(self):
        points = self.raycast()

        for point in points:
            self.bresenham(point)
