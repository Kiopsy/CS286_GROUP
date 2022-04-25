# Final Project Simulation
from grid import Grid
from graphics import GraphicsWindow

class Simulation:
    def __init__(self) -> None:
        pass
                
def main():
    
    g = Grid()
    g.read_Grid('Layouts/layout1_int.txt')
    # g.print_grid()
    timestep = 0
    while True:
        frontier = g.update_grid()
        # print(g.dynamic_grid)
        # graphics = GraphicsWindow(g.dynamic_grid)
        # graphics.show_env(timestep)

        if frontier == None:
            break

        timestep += 1
    
if __name__ == "__main__":
    main()