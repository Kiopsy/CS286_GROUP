# Final Project Simulation
from grid import Grid
from graphics import GraphicsWindow

                
def main():
    
    g = Grid()
    g.read_Grid('Layouts/layout1_int.txt')
    # g.plot_grid()
    timestep = 0
    while True:
        frontier = g.update_grid()

        graphics = GraphicsWindow(g.dynamic_grid)
        graphics.show_env(timestep)

        if frontier == None:
            break

        timestep += 1
    
if __name__ == "__main__":
    main()