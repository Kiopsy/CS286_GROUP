# Final Project Simulation
from grid import Grid
from graphics import GraphicsWindow
                
def main():
    g = Grid()
    g.read_Grid('layout1.txt')
    # g.plot_grid()
    while True:
        x = g.update_grid()

        graphics = GraphicsWindow(g.dynamic_grid)
        graphics.show_env()
        # print()

        if x == None:
            break

if __name__ == "__main__":
    main()