# Final Project Simulation
from fileinput import filename
from environment import Env
from constants import c
from graphics import GraphicsWindow

# Variables to set up environment:

# Choosing which from: random walk, greedy, or wayfront_detection
frontier_algo = c.WAYFRONT
# Show graphics: 1 or 0
show_graphics = 1
# layout filename
filename = 'Layouts/layout1_int.txt'

def main():
    # instantiating the environment & running simulations
    env = Env(filename, frontier_algo, show_graphics)
    env.run_simulation()
    
if __name__ == "__main__":
    main()