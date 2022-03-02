from logging.config import valid_ident
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

    @property
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

    # Define sensing function, f
    def sensing_func(self, state, point):
        # Rename variables
        botx = state[0]
        boty = state[1]
        x = point[0]
        y = point[1]
        # Return Euclidean distance
        dist = math.sqrt((botx - x)**2 + (boty - y)**2)**2
        return dist

    # Create function to handle cases in which terms are zero
    def update_if_zero(self, val, tol):
        if (type(val) is np.ndarray):
            print("MODIFYING2")
            updated_val = []
            comparison_arr = np.isclose(val, np.zeros_like(val), rtol=tol)
            for idx, is_zero in enumerate(list(comparison_arr)):
                modified_val = val[idx]
                if is_zero:
                    modified_val = tol if val[idx] > 0 else -tol
                updated_val.append(modified_val)
        elif math.isclose(val, 0, rel_tol=tol):
            print("MODIFYING2")
            updated_val = 1 if val > 0 else -1
        else:
            updated_val = val
        return updated_val

    # calc the mixing function for the function aka g_alpha, also record f(p, q) and dist, point is np.array([x,y])
    def mix_func(self, point, stoch, value=1):  
        # Define initial g_alpha value 
        g_alpha = 0
        # Create list for f values so they don't need to be recalculated
        f_values = []
        # For each robot in environment 
        for bot in self.robots:
            # Get sensing function value
            if stoch:
                f = self.sensing_func(bot.stoch_state, point)
            else:
                f = self.sensing_func(bot.state, point)
            # Append output of sensing function to f_vals
            f_values.append(f)
            # Raise f to the power of alpha if f is not 0
            if f != 0:
                f = f**self.alpha
            # Add modified sensing function output to g_alpha
            g_alpha += f
        # Raise g_alpha to the power of 1 / alpha
        g_alpha = g_alpha**(1 / self.alpha)

        # Define initial p_dot value 
        for index, robot in enumerate(self.robots):
            # Calculate quotient
            quot = (f_values[index] / g_alpha)
            # Raise quotient to power of (alpha - 1) if it is not 0
            if quot != 0:
                quot = quot**(self.alpha - 1)
            # Covert point and robot state to numpy arrays for vector ops
            point_arr = np.array(point)
            if stoch:
                state_arr = np.array(robot.stoch_state)
            else:
                state_arr = np.array(robot.state)
            # Find direction of motion for next iteration
            prod = np.multiply(quot, np.subtract(point_arr, state_arr))
            # Multiply by importance value and dq
            prod = np.multiply(prod, value)
            prod = np.multiply(prod, self.res)
            # Updat robot input
            robot.input = np.add(np.array(robot.input), prod).tolist()


    def update_gradient(self, part, stoch, iter=0):
        # Below code was given
        # Create distributions for importance functions
        if part != 'A':
            rv = None
            if (type(self.target) is np.ndarray):
                rv =  multivariate_normal(mean = self.target[:, iter], cov = self.cov)
            else:
                rv =  multivariate_normal(mean = self.target, cov = self.cov)
        # Call mixing function for each point in Q
        # Effectively, integrate over Q, since robot input is being changed in each iteration
        for x in self.pointsx:
            for y in self.pointsy:
                value = 1
                if part != 'A': 
                    value = rv.pdf((x,y))
                
                self.mix_func(np.array([x, y]), stoch, value)


    def moves(self):
        for bot in self.robots:
            bot.update()



# function to run the simulation
def run_grid(env, iter, part, stoch):
    x = []
    y = []


    # initialize state
    for i, bot in enumerate(env.robots):
        x.append([bot.state[0]])
        y.append([bot.state[1]])

    # run environment for iterations
    for k in range(iter):
        env.update_gradient(part, stoch, k)
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
        plt.scatter(x[i], y[i], alpha=max((i+1)/len(x[i]), 0.35), label='Robot '+str(i))
        points.append([x[i][-1], y[i][-1]])
    
    if len(env.target) != 0:
        plt.scatter(env.target[0], env.target[1], label='Target')

    '''
    # if there is a target setup plot it
    if(type(env.target) is np.ndarray):
        for i in range(env.target.shape[1]):
            # Changed the alpha such that more recent points are darker for all robots
            plt.scatter(env.target[0], env.target[1], label='Target')
    # Nothing to plot in case with no target
    elif len(env.target) == 0:
        pass
    # Plot single point when target is not numpy array
    else:
        plt.scatter(env.target[0], env.target[1], label='Target')
    '''


    # set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    plt.plot(b_x, b_y)        

    # set Voronoi
    vor = Voronoi(np.array(points))
    voronoi_plot_2d(vor, ax=ax)
    
    ax.set_xlim((-1, 11))
    ax.set_ylim((-1, 11))
    plt.title("Robot (and Target) Positions")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend(loc='upper left')
    plt.show()
    
# generate target points
def target(iter):
    pi = np.pi
    # Create points for range of circle function
    theta = np.linspace(0, (2 * pi) * (iter / 800), iter)
    
    # Define circle radius
    radius = 3

    # Calculate coordinates
    x = radius * np.cos(theta) + 5
    y = radius * np.sin(theta) + 5

    return np.array([x, y])


if __name__ == "__main__":
    iter = 200
    rob1 = Robot([4, 1], 0.5)
    rob2 = Robot([2, 2], 0.5)
    rob3 = Robot([5, 6], 0.5)
    rob4 = Robot([3, 4], 0.5)
    robots = [rob1, rob2, rob3, rob4]

    # Create variable for which part of question #1 is being considered
    part = 'A'
    # Create variable indicating whether the system is stochastic
    stoch = False

    if part == 'A':
        env = Environment(10, 10, 0.1, robots)
    elif part == 'B':
        env = Environment(10, 10, 0.1, robots, target=(5,5))
    else:
        # Define dynamic, quarter-circle target with period 800
        dyn_target = target(iter)

        env = Environment(10, 10, 0.1, robots, target=dyn_target)

    # Note: The "part" and stoch arguments were added for easier use and eval
    run_grid(env, iter, part, stoch)