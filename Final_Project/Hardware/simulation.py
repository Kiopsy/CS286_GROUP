# Final Project Simulation
from grid import Grid
from graphics import GraphicsWindow

                
def main():
    g = Grid()
    g.define_grid(grid, robots)
    frontier = g.get_frontier_pos()

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