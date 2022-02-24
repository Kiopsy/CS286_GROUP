import math
import shapely
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.stats import multivariate_normal

class Robot(object):

    def __init__(self, state, k=0.1):
        self._state = state     # 2-vec
        self._stoch_state = self._state + np.random.randn(2)
        self.input = [0,0]      # movement vector later

        self.k = k * 0.001

    def update(self):     # update the robot state
        # Below was given
        #self._state += self.k * self.input
        #self._stoch_state = self._state + np.random.randn(2)

        self._state = np.add(self._state, np.multiply(self.k, np.array(self.input))).tolist()
        self._stoch_state = np.add(np.array(self._state), np.random.randn(2)).tolist()
        self.input = [0, 0]

    @property
    def state(self):
        return np.array(self._state)

    def stoch_state(self):
        return np.array(self._stoch_state)



class Environment(object):

    def __init__(self, width, height, res, robots, alpha = -10, sigma = 0, cov = 5, target = []):       # width and height are in pixels, so actual dimensions are width * res in meters
        self.width = width
        self.height = height
        self.res = res

        # bottom left corner is 0, 0 both in pixels and in meters
        self.robots = robots        # initialized w/ array of Robot objects
        self.meas_func = np.zeros((len(robots)))
        self.dist = np.zeros((2, len(robots)))

        # define the points you're iterating over
        self.pointsx = np.arange(0, width, res)
        self.pointsy = np.arange(0, height, res)

        self.alpha = alpha
        self.sigma = sigma
        self.cov = cov

        self.target = target

    def sensing_func(self, state, point):
        # Rename variables
        botx = state[0]
        boty = state[1]
        x = point[0]
        y = point[1]
        # Return Euclidean distance
        return math.sqrt((botx - x)**2 + (boty - y)**2)

    # calc the mixing function for the function aka g_alpha, also record f(p, q) and dist, point is np.array([x,y])
    # value is the value of the importance function
    def mix_func(self, point, value=1):  
        # Define initial g_alpha value 
        g_alpha = 0
        # Create list for f values so they don't need to be recalculated
        f_values = []
        # For each robot in environment 
        for bot in self.robots:
            # Get sensing function value
            f = self.sensing_func(bot.state, point)
            # Append output of sensing function to f_vals before modification
            f_values.append(f)
            print("F before: " + str(f))
            if f != 0:
                # Raise f to the power of alpha
                f = f**self.alpha
            print("F after: " + str(f))
            # Add modified sensing function output to g_alpha
            g_alpha += f
            print("G alpha: " + str(g_alpha))
        # Raise g_alpha to the power of 1 / alpha
        g_alpha = g_alpha**(1 / self.alpha)
        print("G alpha final: " + str(g_alpha))

        # Define initial p_dot value 
        p_dot = 0
        for index, robot in enumerate(self.robots):
            quot = (f_values[index] / g_alpha)
            if quot != 0:
                quot = quot**(self.alpha - 1)
            prod = quot * value
            state_arr = np.array(robot.state)
            point_arr = np.array(point)
            prod = np.multiply(prod, np.subtract(state_arr, point_arr))
            p_dot += prod

        # Change control input
        self.input = p_dot.tolist()
        self.moves()

    def update_gradient(self, iter = 0):
        # rv = None
        # if(type(self.target) is np.ndarray):
        #     rv =  multivariate_normal(mean = self.target[:, iter], cov = self.cov)
        # else:
        #     rv =  multivariate_normal(mean = self.target, cov = self.cov)

        for x in self.pointsx:
            for y in self.pointsy:
                value = 1
                # value = rv.pdf((x,y))

                self.mix_func(np.array([x, y]), value)

    def moves(self):
        for bot in self.robots:
            bot.update()



# function to run the simulation
def run_grid(env, iter):
    x = []
    y = []


    # initialize state
    for i, bot in enumerate(env.robots):

        x.append([bot.state[0]])
        y.append([bot.state[1]])

    # run environment for iterations
    for k in range(iter):
        env.update_gradient(k)
        env.moves()

        for i, bot in enumerate(env.robots):

            x[i].append(bot.state[0])
            y[i].append(bot.state[1])

        if (k % 50 == 0):
            print(k)

    # set up the plot
    fig, ax = plt.subplots()
    points = []

    # plt the robot points
    plt.axes(ax)
    for i in range(len(env.robots)):
        plt.scatter(x[i], y[i], alpha=(i+1)/len(env.robots))
        points.append([x[i][-1], y[i][-1]])
    

    # if there is a target setup plot it
    if(type(env.target) is np.ndarray):
        for i in range(env.target.shape[1]):
            plt.scatter(env.target[0, i], env.target[1, i], alpha=(i+1)/env.target.shape[1])

    # set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    plt.plot(b_x, b_y)        

    # set Voronoi
    vor = Voronoi(np.array(points))
    voronoi_plot_2d(vor, ax=ax)
    
    ax.set_xlim((-1, 11))
    ax.set_ylim((-1, 11))

    plt.show()
    
# generate target points
def target(iter):

    raise NotImplementedError

if __name__ == "__main__":

    rob1 = Robot([4, 1], 0.5)
    rob2 = Robot([2, 2], 0.5)
    rob3 = Robot([5, 6], 0.5)
    rob4 = Robot([3, 4], 0.5)
    robots = [rob1, rob2, rob3, rob4]

    env = Environment(10, 10, 0.1, robots)
    #env = Environment(10, 10, 0.1, robots, target=(5,5))


    run_grid(env, 10)