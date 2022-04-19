# Final Project Simulation
from grid import Grid
                
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