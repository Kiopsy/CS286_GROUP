import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.stats import multivariate_normal

class Server(object):

    def __init__(self, state, k=0.1):
        self._state = state     # 2-vec
        self.input = [0,0]      # movement vector late
        self.k = k * 0.001

    def update(self):     # update the robot state
        self._state += self.k * self.input
        self.input = [0, 0]

    @property
    def state(self):
        return np.array(self._state)

class Client(object):

    def __init__(self, state, spoofer = False):
        self.state = state      # 2-vec
        self.spoofer = spoofer  # if true, then client is a spoofer; else, client is legitimate.
        self.alpha = None       # confidence metric (Gil et al. Section 3); to be assigned by via the environment
        self.rho = None         # individual contribution to importance function (Gil et al.)

class Environment(object):

    def __init__(self, width, height, res, servers, clients, spoof_mean = 0.1, legit_mean = 0.8, alpha = -10):       
        # Width and height are in pixels, so actual dimensions are width * res in meters
        # Bottom left corner is 0, 0 both in pixels and in meters
        self.width = width
        self.height = height
        self.res = res

        self.servers = servers        # initialized w/ array of Server objects
        self.clients = clients        # initialized w/ array of Client objects

        self.spoof_mean = spoof_mean  # mean of distribution from which legitimate clients sample
        self.legit_mean = legit_mean  # mean of distribution from which spoofed clients sample
        
        self.meas_func = np.zeros((len(servers)))
        self.dist = np.zeros((2, len(servers)))

        # Define the points you're iterating over
        self.pointsx = np.arange(0, width, res)
        self.pointsy = np.arange(0, height, res)

        self.alpha = alpha # "free parameter" from Schwager et al., Section 2.2
        self.importance = np.zeros((len(self.pointsx), len(self.pointsy)))

    def define_rho(self):
        # Create matrix for collective importance map
        imp_vals = np.zeros((len(self.pointsx), len(self.pointsy)))
        for client in self.clients:
            # Create matrix for client-specific importance map
            client_imp_vals = np.zeros_like(imp_vals)
            for i, x in enumerate(self.pointsx):
                for j, y in enumerate(self.pointsy):
                    # Compute importance for each point in environment
                    imp = 0
                    point = np.array([x, y])
                    client_pos = np.array(client.state)
                    client_pos = np.array(client_pos)
                    # Compute first diff vector
                    diff = np.subtract(point, client_pos)
                    # Transpose diff vector
                    diff_T = np.transpose(diff)
                    # Compute error
                    mat_prod = np.dot(diff_T, diff)
                    # Complete calc with exponentation
                    imp += math.exp((-0.5) * mat_prod)
                    # Add to client-specific importance map
                    client_imp_vals[i, j] += imp
            # Set client rho value to its importance map
            client.rho = client_imp_vals 
            if client.alpha is not None:
                client.rho *= client.alpha
            # Add to collective importance map
            imp_vals = np.add(imp_vals, client_imp_vals)
        # Store collective importance map in environment
        self.importance = imp_vals


    def sample_alphas(self, spoof_mean, legit_mean):
        # Create variable for standard deviation of normal dist
        std = 0.10
        for client in self.clients:
            # Create variable to indicate which clients are spoofers
            is_spoofer = client.spoofer
            if (is_spoofer):
                # Define random variable from normal dist with corresponding mean
                rv =  np.random.normal(loc=spoof_mean, scale=std)
            else:
                # Define random variable from normal dist with corresponding mean
                rv =  np.random.normal(loc=legit_mean, scale=std)
            # Set client alpha value
            client.alpha = rv

    def mix_func(self, point, value = 1):     
        # Calc the mixing function for the function aka g_alpha, also record f(p, q) and dist, point is np.array([x,y])
        for i, bot in enumerate(self.servers):
            self.dist[:, i] = point - bot.state
            self.meas_func[i] = 0.5 * np.linalg.norm(self.dist[:, i])**2

        mixing = np.sum(self.meas_func[self.meas_func > 0]**self.alpha)**(1/self.alpha)

        for i, bot in enumerate(self.servers):
            if(self.meas_func[i] > 0):
                bot.input += (self.meas_func[i] / mixing)**(self.alpha - 1) * self.dist[:, i] * value

    def update_gradient(self):
        # Redefine rho in case alpha values have been sampled
        env.define_rho()
        for i, x in enumerate(self.pointsx):
            for j, y in enumerate(self.pointsy):
                # Compute mixing function, using env's importance map
                self.mix_func(np.array([x, y]), self.importance[i, j])

    def moves(self):
        for bot in self.servers:
            bot.update()



# Function to run the simulation
def run_grid(env, iter, part):
    x = []
    y = []

    # Set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    #plt.plot(b_x, b_y)        

    # Initialize state
    env.define_rho()

    for i, bot in enumerate(env.servers):
        x.append([bot.state[0]])
        y.append([bot.state[1]])

    # Run environment for iterations
    for k in range(iter):
        # Only use alpha-modified importance function if part is not "B"
        if part != "B":
            env.sample_alphas(env.spoof_mean, env.legit_mean)
        env.update_gradient()
        env.moves()

        for i, bot in enumerate(env.servers):
            x[i].append(bot.state[0])
            y[i].append(bot.state[1])
            
        if (k % 50 == 0):
            print(k)

    # Set up the plot
    fig1, ax1 = plt.subplots()
    points = []
    # Define list for transparency vals in legend
    legend_alphas = []

    # Plot the server points in first figure
    plt.axes(ax1)
    for i in range(len(env.servers)):
        plt.scatter(x[i], y[i], alpha=max((i+1)/len(x[i]), 0.25), label='Server '+str(i))
        points.append([x[i][-1], y[i][-1]])
        legend_alphas.append(1)

    # Plot the client points in first figure
    for j, c in enumerate(env.clients):
        if c.spoofer:
            plt.scatter(c.state[0], c.state[1], alpha=0.1, label="Client "+str(j))
            legend_alphas.append(0.1)
        else:
            plt.scatter(c.state[0], c.state[1], alpha=0.9, label="Client "+str(j))
            legend_alphas.append(0.9)

    # Set Voronoi
    # vor = Voronoi(np.array(points))
    # voronoi_plot_2d(vor, show_vertices = False, line_colors='blue', ax=ax)
    
    ax1.set_xlim((-1, 11))
    ax1.set_ylim((-1, 11))

    # Beautify first figure
    if part == "B":
        plt.title("Server and Client States\nIters = " + str(iter) + ", Not Alpha-Modified")
    else:
        plt.title("Server and Client States\nIters = " + str(iter) + ", Spoofer Mean = " + str(env.spoof_mean) + ", Legit Mean = " + str(env.legit_mean))
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    leg = ax1.legend(loc='upper left')
    for idx, lh in enumerate(leg.legendHandles): 
        lh.set_alpha(legend_alphas[idx])
    # Save first figure
    fig1.savefig('/home/dominicgarrity/CS286/CS286_GROUP/PSET3/Images/Prob_1/1' 
                + part + '/Prob_1' + part + '_States.png')

    # Plot importance function in second figure
    fig2, ax2 = plt.subplots()
    ax2 = plt.axes(projection='3d')
    x = env.pointsx
    y = env.pointsy
    Y, X = np.meshgrid(x, y)
    ax2.plot_surface(X, Y, env.importance, rstride=1, cstride=1,
            cmap='viridis', edgecolor='none')
    # Beautify second figure
    if part == "B":
        plt.title("Importance Function\nIters = " + str(iter) + ", Not Alpha-Modified")
    else:
        plt.title("Importance Function\nIters = " + str(iter) + ", Spoofer Mean = " + str(env.spoof_mean) + ", Legit Mean = " + str(env.legit_mean))
    ax2.set_xlabel("X Position")
    ax2.set_ylabel("Y Position")
    ax2.set_zlabel("Importance")
    # Save second figure
    fig2.savefig('/home/dominicgarrity/CS286/CS286_GROUP/PSET3/Images/Prob_1/1' 
                + part + '/Prob_1' + part + '_Importance.png')

    plt.show()

if __name__ == "__main__":

    serv1 = Server([4, 1])
    serv2 = Server([2, 2])
    serv3 = Server([5, 6])
    serv4 = Server([3, 4])
    servers = [serv1, serv2, serv3, serv4]

    client1 = Client([3,3], True)
    client2 = Client([1,1])
    client3 = Client([9,9])
    client4 = Client([8,3], True)
    clients = [client1, client2, client3, client4]

    # Define variables for easier eval--just change these according to below comment!
    part = "B" # part of question #1 to be considered
    num_iter = 200 # number of iterations

    '''
    Configuration for each part:
    #1(B): part = 'B', num_iter = 200
    #1(D): part = 'D', num_iter = 200
    #1(Ei, test 1): part = 'Ei1', num_iter = 200
    #1(Ei, test 2): part = 'Ei2', num_iter = 200
    #1(Eii): part = 'Eii', num_iter = 200
    '''

    # Define environment
    if part == "B":
        env = Environment(10, 10, 0.1, servers, clients)
    elif part == "D":
        env = Environment(10, 10, 0.1, servers, clients, 0.2, 0.8)
    elif part == "Ei1":
        env = Environment(10, 10, 0.1, servers, clients, 0.01, 0.99)
    elif part == "Ei2":
        env = Environment(10, 10, 0.1, servers, clients, 0.99, 0.01)
    elif part == "Eii":
        env = Environment(10, 10, 0.1, servers, clients, 0.5, 0.5)
    else:
        raise ValueError("The variable 'part' can only equal 'B', 'D', 'Ei1', 'Ei2', or 'Eii'")

    run_grid(env, num_iter, part)