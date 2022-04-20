import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as mpatch
import matplotlib.colors as mcol
import matplotlib.animation as manim

# Define RGB triples for colors
RED = [255, 0, 0]
LIGHTRED = [255, 192, 203]
GRAY = [128, 128, 128]
BLACK = [0, 0, 0]
GREEN = [0, 255, 0]
LIGHTGREEN = [152, 251, 152]
WHITE = [255, 255, 255]

# Define character values and corresponding colors and labels
COLORMAP = {"R": [RED, "Robot"],
            "*": [LIGHTRED, "Previous Path"],
            " ": [WHITE, "Free Space"],
            "F": [GREEN, "Frontier Cell"],
            ".": [LIGHTGREEN, "Next Path"],
            "W": [GRAY, "Wall"],
            "-": [BLACK, "Unexplored Space"],            
            }

class GraphicsWindow:
    def __init__(self, dynamic_grid):
        self.env = dynamic_grid

    def replace_values(self, val):
        # Return color value
        color, _ = COLORMAP[val]
        return color


    def convert(self):
        env = self.env
        # Replace characters with colors
        for x in range(len(env)):
            for y in range(len(env[x])):
                env[x][y] = self.replace_values(env[x][y])
        
    def show_env(self, timestep): 
        self.convert()
        env = self.env
        print(self.env)

        im = plt.imshow(self.env, interpolation='none')
        color_vals = np.unique(np.array(env).ravel())
        # Get unique colors within map
        pixel_info = [val for val in COLORMAP.values()]
        # Create color patches for legend
        color_patches = []
        for i in range(len(pixel_info)):
            color_patches.append(mpatch.Patch(color=mcol.to_rgba(im.norm(pixel_info[i][0])), label=pixel_info[i][1]))
        # Use patches as handles for legend
        plt.legend(handles=color_patches, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

        plt.title('Robot Map\nTimestep = ' + str(timestep))
        #plt.show()
        plt.savefig(os.getcwd() + '/Frames/Frame_' + str(timestep) + '.png', bbox_inches='tight')


        
        
        
        

        






