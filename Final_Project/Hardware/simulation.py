# Final Project Simulation
from grid import Grid
from graphics import GraphicsWindow

import pandas as pd
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt

def main():

    grid = np.genfromtxt('map_reader_python\map.csv', delimiter=',')  

    grid = grid[:,1:]
    grid = np.delete(grid, (0), axis = 0)

    grid[200][50] = 101
    

    cmap = mpl.colors.ListedColormap(['blue', 'white','black', 'red'])
    bounds = [-1, 0, 50, 101, 10000]
    norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
    
    # tell imshow about color map so that only set colors are used
    img = plt.imshow(grid, interpolation='nearest', cmap = cmap,norm=norm)

    # make a color bar
    
    plt.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])
    
    plt.show()

    g = Grid()
    robot_positions = [(50,200)]
    g.define_grid(grid, robot_positions)
    frontiers = []
    for robot in g.robots:
        frontier = g.get_frontier_pos(robot)
        # Ensure that each frontier is unique
        while frontier in frontiers:
            frontier = g.get_frontier_pos(robot)
        frontiers.append(frontier)
        print(frontier)

    # timestep = 0
    # while True:
    #     frontier = g.get_frontier_pos()

    #     graphics = GraphicsWindow(g.dynamic_grid)
    #     graphics.show_env(timestep)

    #     if frontier == None:
    #         break

    #     timestep += 1
    
if __name__ == "__main__":
    main()