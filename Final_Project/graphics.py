import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Define RGB triples for colors
RED = [255, 0, 0]
WHITE = [255, 255, 255]
PINK = [255, 192, 203]
GRAY = [128, 128, 128]
BLACK = [0, 0, 0]
GREEN = [0, 255, 0]

class GraphicsWindow:
    def __init__(self, dynamic_grid):
        self.env = dynamic_grid

    def replace_values(self, val):
        # Define character values and corresponding color values
        colors = {"R": RED,
                  " ": WHITE,
                  ".": PINK,
                  "W": GRAY,
                  "-": BLACK,
                  "F": GREEN               
                }
        # Return color value
        return colors.get(val, val)


    def convert(self):
        env = self.env
        # Replace characters with colors
        for y in range(len(env)):
            for x in range(len(env[y])):
                env[y][x] = self.replace_values(env[y][x])
        
    def show_env(self): 
        self.convert()
        print(self.env)
        plt.imshow(self.env)
        plt.show()
        
        
        
        

        






