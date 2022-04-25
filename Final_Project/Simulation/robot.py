
import math
from bresenham import bresenham
from astar import astar
from constants import c
import random
import numpy as np
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt

SENSING_RADIUS = 4
FRONTIER_SIZE = 10
MIN_FRONTIER_DISTANCE = 5
MAX_FRONTIER_LEN = 20
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

    def is_frontier(self, p):
        found_free = False
        found_unknown = False
        found_obstacle = False
        col, row = p
        for i in range(3):
            for j in range(3):
                try:
                    if self.grid[row-1 + i][col-1+j] == c.FREE:
                        found_free = True
                    if (col-1+j, row-1 + i) in self.seen:
                        found_unknown = True
                    if self.grid[row-1 + i][col-1+j] == c.WALL:
                        found_obstacle = True
                except:
                    continue
        return found_free and found_unknown and not found_obstacle

    def get_neighbors(self, p):
        col, row = p
        neighbors = []
        for i in range(3):
            for j in range(3):
                if i == 1 and j == 1:
                    continue
                try:
                    if row - 1+ i < 0 or col -1 + j < 0:
                        continue
                    if self.grid[row-1+i][col -1 +j] != c.FREE:
                        continue
                    self.grid[row-1+i][col-1+j] # means is in grid
                    neighbors.append((col-1+j, row-1+i))
                except:
                    continue
        return neighbors
        
    def get_frontier(self):
        Q_map = [self.pos]
        self.grid[self.pos[1]][self.pos[0]] = c.FREE

        visited = np.zeros_like(np.array(self.grid))

        all_frontiers = []
        num_iter = 0
        grid_size = visited.shape[0] * visited.shape[1]
        while len(Q_map) > 0:
            p = Q_map.pop(0)
            # print("{}/{}".format(num_iter, grid_size))
            # if num_iter % 5000 == 0:
            #     print(len(np.where(visited > 0.5, visited, visited)))
            num_iter += 1
            pcol, prow = p
            if visited[prow][pcol] > 0.5:
                continue
            visited[prow][pcol] = 1.0
            if self.is_frontier(p):
                # run inner BFS
                Q_frontier = [p]
                new_frontier = []
                # mark not visited so that it actually visits
                visited[prow][pcol] = 0.0

                while len(Q_frontier) > 0:
                    q = Q_frontier.pop(0)
                    qcol, qrow = q
                    if visited[qrow][qcol] > 0.5:
                        continue
                    visited[qrow][qcol] = 1.0
                    
                    new_frontier.append(q)
                    neighbors = self.get_neighbors(q)
                    for n in neighbors:
                        ncol, nrow = n
                        if self.is_frontier(n) and visited[nrow][ncol] < 0.5:
                            Q_frontier.append(n)
                # finished inner bfs
                if len(new_frontier) > 0:
                    # print("frontier: {}".format(new_frontier))
                    all_frontiers.append(new_frontier)

            for n in self.get_neighbors(p):
                ncol, nrow = n
                if visited[nrow][ncol] < 0.5:
                    Q_map.append(n)

        # print(all_frontiers)
        # cmap = mpl.colors.ListedColormap(['blue', 'white','black', 'red'])
        # bounds = [-1, 0, 50, 101, 10000]
        # norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
    
        
        def get_centroid(frontier):
            n = float(len(frontier))
            x_acc, y_acc = 0.0, 0.0
            for p in frontier:
                x_acc += float(p[0])
                y_acc += float(p[1])
            return (int(round(x_acc/n)), int(round(y_acc/n)))
            # return (x_acc/n, y_acc/n)
        all_frontiers = sorted(all_frontiers, key=lambda f: len(f), reverse=True)

        return list(map(get_centroid, all_frontiers))

    # def get_frontier(self):
    #     Q = [self.pos]
    #     V = set()

    #     found_frontiers = []



    #     while len(Q) != 0 and len(found_frontiers) < MAX_FRONTIER_LEN:
            
    #         n = Q.pop(0)
            
    #         if n in V: continue
    #         else: V.add(n)

    #         x, y = n

    #         try:
    #             space = self.grid[y][x] 
    #         except:
    #             continue
            
    #         if space <= c.FREE:

    #             if n not in self.seen and self.is_big(n):
    #                 min_dist = 1e9
                    
    #                 for f in found_frontiers:
    #                     if self.distance(f, n) < min_dist:
    #                         min_dist = self.distance(f,n)

    #                 if min_dist > MIN_FRONTIER_DISTANCE:
    #                     found_frontiers.append(n)
    #                 # return n

    #             # NOTE: May need to handle case where edges are not walls
    #             if x > 0:
    #                 Q.append((x - 1, y))
    #             Q.append((x + 1, y))
                
    #             if y > 0:
    #                 Q.append((x, y - 1))
    #             Q.append((x, y + 1))

    #             # Q.append((x - 1, y - 1))
    #             # Q.append((x + 1, y - 1))
    #             # Q.append((x - 1, y + 1))
    #             # Q.append((x + 1, y + 1))
    #         else:
    #             continue
    #     print("found frontiers", found_frontiers)
    #     if len(found_frontiers) > 0:
    #         return found_frontiers
    #         # return random.choice(found_frontiers)
    #     return None

    def get_frontier_path(self):
        self.frontier = self.get_frontier()[0]
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
            self.pos = sorted_path[1]
            # return self.frontier
            try:
                cell = sorted_path[-1]
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