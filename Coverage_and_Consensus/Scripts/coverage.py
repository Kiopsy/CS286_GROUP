from logging.config import valid_ident
import math
import shapely
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.stats import multivariate_normal

class Robot(object):

    def __init__(self, state, k=0.1):
        self._state = state     # 2-vec
        self._stoch_state = self._state + np.random.randn(2)
        self.input = [0,0]      # Movement vector

        self.k = k 

    def update(self):
        # Update state (state += k * input)
        self._state = np.add(self._state, np.multiply(self.k, np.array(self.input))).tolist()
        # Update stochastic state (stoch_state += state * + randn(2))
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

        # Bottom left corner is 0, 0 both in pixels and in meters
        self.robots = robots        # Initialized w/ array of Robot objects
        self.meas_func = np.zeros((len(robots)))
        self.dist = np.zeros((2, len(robots)))

        # Define the points to iterate over
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
        # Calculate and return Euclidean distance
        dist = math.sqrt((botx - x)**2 + (boty - y)**2)
        return dist

    # Calc the mixing function for the function aka g_alpha, also record f(p, q) and dist, point is np.array([x,y])
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
            # Covert point and robot state to numpy arrays for vector operations
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
            # Update robot input
            robot.input = np.add(np.array(robot.input), prod).tolist()


    def update_gradient(self, part, stoch, iter=0):
        # Below code was given
        # Create distributions for importance functions
        if part != 'A' and part != 'E':
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
                if part != 'A' and part != 'E': 
                    value = rv.pdf((x,y))
                
                self.mix_func(np.array([x, y]), stoch, value)
        print("Updating! " + str(iter))


    def moves(self):
        for bot in self.robots:
            bot.update()



# Function to run the simulation
def run_grid(env, iter, part, stoch):
    x = []
    y = []
    step_size = None


    # Initialize state
    for i, bot in enumerate(env.robots):
        x.append([bot.state[0]])
        y.append([bot.state[1]])
        if i > 0 and bot.k != step_size:
            raise ValueError("For the purposes of this assignment, please choose the same k for every robot.")
        step_size = bot.k

    # run environment for iterations
    for k in range(iter):
        env.update_gradient(part, stoch, k)
        env.moves()

        for i, bot in enumerate(env.robots):
            x[i].append(bot.state[0])
            y[i].append(bot.state[1])

        if (k % 50 == 0):
            print(k)

    # Set up the plot
    fig, ax = plt.subplots()
    points = []

    # Plot robot points
    plt.sca(ax)
    for i in range(len(env.robots)):
        plt.scatter(x[i], y[i], alpha=max((i+1)/len(x[i]), 0.35), label='Robot '+str(i))
        points.append([x[i][-1], y[i][-1]])
    
    # Plot target, if there is one
    if len(env.target) != 0:
        plt.scatter(env.target[0], env.target[1], label='Target')


    # Set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    plt.plot(b_x, b_y)        

    # Set Voronoi
    vor = Voronoi(np.array(points))
    voronoi_plot_2d(vor, ax=ax)
    
    ax.set_xlim((-1, 11))
    ax.set_ylim((-1, 11))
    plt.title("Robot and Target Positions\nf = ||q - p_i||, alpha = " + str(env.alpha) + ", k = " + str(step_size))
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend(loc='upper left')
    plt.show()
    # Save files
<<<<<<< HEAD
    alpha_name = str(alpha) if isinstance(alpha, int) else str(alpha).replace(".", "-")
    s = '../Images/Problem_1/1'  + part + '/Prob_1' + part + '_' + alpha_name + '.png'
    print(s)
    # plt.savefig(s)
    plt.show(block=False)
=======
    #alpha_name = str(alpha) if isinstance(alpha, int) else str(alpha).replace(".", "-")
    #fig.savefig('/home/dominicgarrity/CS286/CS286_GROUP/Coverage_and_Consensus/Images/Problem_1/1' 
                #+ part + '/Prob_1' + part + '_' + alpha_name + '.png')
>>>>>>> 0f1b2098424ed5c0faf00c2cf2b14107b8b1237f

# Generate target points
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

    ## Create vars for easier eval--just change these according to below comment!
    # Create variable for which part of question #1 is being considered
    part = 'A'
    # Create variable indicating whether the system is stochastic
    stoch = True

    '''
    Configuration for each part:
    #1(a): part = 'A', stoch = False
    #1(b): part = 'B', stoch = False
    #1(c): part = 'C', stoch = False
    #1(E): part = 'E', stoch = True
    '''
    # Create environments and run grid for different values of alpha
    alpha_vals = [-0.5, -1, -5, -10]
    if part == 'A':
        for alpha in alpha_vals:
            rob1 = Robot([4, 1], 0.0005)
            rob2 = Robot([2, 2], 0.0005)
            rob3 = Robot([5, 6], 0.0005)
            rob4 = Robot([3, 4], 0.0005)
            robots = [rob1, rob2, rob3, rob4]
            env = Environment(10, 10, 0.1, robots, alpha)
            run_grid(env, iter, part, stoch)
    elif part == 'B':
        for alpha in alpha_vals:
            rob1 = Robot([4, 1], 0.15)
            rob2 = Robot([2, 2], 0.15)
            rob3 = Robot([5, 6], 0.15)
            rob4 = Robot([3, 4], 0.15)
            robots = [rob1, rob2, rob3, rob4]
            env = Environment(10, 10, 0.1, robots, alpha, target=(5,5))
            run_grid(env, iter, part, stoch)
    elif part == 'C':
        # Define dynamic, quarter-circle target with period 800
        dyn_target = target(iter)
        for alpha in alpha_vals:
            rob1 = Robot([4, 1], 0.15)
            rob2 = Robot([2, 2], 0.15)
            rob3 = Robot([5, 6], 0.15)
            rob4 = Robot([3, 4], 0.15)
            robots = [rob1, rob2, rob3, rob4]
            env = Environment(10, 10, 0.1, robots, alpha, target=dyn_target)
            run_grid(env, iter, part, stoch)
    elif part == 'E':
        for alpha in alpha_vals:
            rob1 = Robot([4, 1], 0.15)
            rob2 = Robot([2, 2], 0.15)
            rob3 = Robot([5, 6], 0.15)
            rob4 = Robot([3, 4], 0.15)
            robots = [rob1, rob2, rob3, rob4]
            env = Environment(10, 10, 0.1, robots, alpha)
            run_grid(env, iter, part, stoch)
    else:
        raise ValueError("The 'part' variable can only take the value A, B, C, or E.")
    
    plt.show()