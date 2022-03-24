import math
from re import I
from tempfile import SpooledTemporaryFile
import numpy as np
import matplotlib.pyplot as plt

# Run experiments for different questions:

# Question options
Qs = ["2a", "2b", "2c", "2d", "2ei", "2eii"]

# NOTE: Change this string to any of the question options above
question = "2a"

# Function for switching between which question to answer (not relevant to answers)
run_question = dict()
def answer_question(question):
    for key in Qs:
        run_question[key] = False
    run_question[question] = True


# this returns a bunch of weights - sets a mean alpha level, then randomly generates alphas around the means
class AlphaWeights(object):

    def __init__(self, mean_alpha, std = 0.1):

        self.std = std
        self.iter = 1

        # assuming fully connected - this is an adjancency matrix
        # assume the vector is [legitimate, spoofed], assume it's diagraph
        self.means = mean_alpha
        self.betas = self.means + np.random.randn(self.means.shape[0], self.means.shape[1]) * self.std

    # Calculate the updates for the betas used in updating W
    # be sure to cap alpha at -0.5 and 0.5 at all times
    # this function should generate a random set of alphas based on the means and then update beta accordingly
    def update_betas(self):

        alpha = self.means + np.random.randn(self.means.shape[0], self.means.shape[1]) * self.std

        # cap alphas at 0.5 because thats the way it is
        thresh = alpha > 0.5
        alpha[thresh] = 0.5
        thresh = alpha < -0.5
        alpha[thresh] = -0.5

        # update beta because it's a running sum of alphas
        self.betas += alpha
        
        self.iter += 1


# define the simulation environment
class Environment(object):

    def __init__(self, leg, spoof, Theta, Omega):

        self.leg = leg         # legitimate nodes list n_l
        self.spoof = spoof     # spoofed nodes list n_s

        self.leg_len = leg.shape[0]
        self.full_len = leg.shape[0] + spoof.shape[0]

        self.Theta = Theta              # transition for spoofed based on leg - n_s x n_l   (row, col)
        self.Omega = Omega              # transition for spoofed based on spoofed - n_s x n_s

        # transition for legitimate based on spoof and leg - n_l x (n_l + n_s)
        # first n_l columns are the legit part W_L
        self.W = np.zeros((self.leg_len, self.full_len))

        self.iter = 0

    # updates according to the system dynamics given
    def update(self, sin_positions=None):

        self.leg = np.matmul(self.W, np.concatenate((self.leg, self.spoof), axis=0))
        self.spoof = np.matmul(self.Theta, self.leg) + np.matmul(self.Omega, self.spoof)

        # Use the sin_positions at our current iteration
        if run_question["2ei"]:
            self.spoof.fill(sin_positions[self.iter])
        
        self.iter += 1

    # set the transitions of W
    # call alphaweights to get an updated beta, then use that to update W.
    # the code will call update to progress the simulation
    def transition_W(self, alphaweights):
        betas = alphaweights.betas

        for i in range(self.W.shape[0]):
            for j in range(self.W.shape[1]):
                if betas[i][j] >= 0:
                    self.W[i][j] = (1/self.leg_len)*(1 - math.exp(-betas[i][j] / 2))
                else:
                    self.W[i][j] = (1/(2*self.leg_len))*(math.exp(betas[i][j]))

        # Changing when i == j
        for i in range(self.W.shape[0]):
            self.W[i][i] += 1 - (np.sum(self.W[i]))


    # check if we reached consensus function
    def reached_consensus(self):

        t = .005

        # Potential consensus value
        mean = np.sum(self.leg) / self.leg_len

        at_consensus = True
        for node in self.leg:
            if (node > mean + t) or (node < mean - t):
                at_consensus = False
        
        return at_consensus, mean


# it plots the states - basically the same function from HW 2
def plot_states(node_states, spoof_node_states, consensus=None, title = None):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()

    # Plot state of legitimate nodes
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i], 'c')
    line.set_label(f'Legit States')

    # Plot state of spoof nodes
    for i in range(spoof_node_states.shape[1]):
        line, = ax.plot(steps, spoof_node_states[:, i], ':r')
    line.set_label(f'Spoofed States')

    # Plot the average state
    line, = ax.plot(steps, np.ones(len(steps)) * np.average(node_states[0, :]), '--')
    line.set_label("Average of Legit States")

    if consensus:
        x, y = consensus
        line = ax.scatter(x, y, s=80, color="purple", marker=(5, 1), zorder=2)
        line.set_label(f"Consensus = {y[0]:0.2f} ")

    # print average states
    print(f"Legit nodes' starting average state: {np.average(node_states[0, :])}")
    print(f"Legit nodes' ending average state: {np.average(node_states[-1, :])}")

    if title:
        ax.set_title(title)

    plt.legend()
    plt.show()

def run(leg, spoof, **kwargs):

    print(kwargs)
    
    # Setting up a new matrix of inputs for the dynamics
    alphas = np.ones((leg.shape[0] + spoof.shape[0], leg.shape[0] + spoof.shape[0]))
    alphas = 0.4 * alphas
    alphas[:, -spoof.shape[0]:] = -1 * alphas[:, -spoof.shape[0]:] 

    alphas = AlphaWeights(alphas)

    theta = np.zeros((spoof.shape[0], leg.shape[0]))

    # Sets the spoof transition function to account for the exponential case in 2e ii
    omega = np.eye(spoof.shape[0]) * 1.01 if run_question["2eii"] else np.eye(spoof.shape[0]) 

    # Define the environment
    env = Environment(leg, spoof, theta, omega)

    # Changed from 200 iterations for better graphs
    iter = 50 if run_question["2eii"] or run_question["2ei"] else 25

    # Sin values for 2e i
    step = 2 * np.pi / 50
    inputs = np.arange(0, iter*step, step)
    sin_values = np.sin(inputs) + 4

    # Run the simulation and plot
    leg_states = []
    spoof_states = []

    leg_states.append(env.leg)
    spoof_states.append(np.array(env.spoof.tolist()))

    # capture consensus
    never_reached_consensus = True
    consensus = None

    # update and store states over the 'iter' iterations
    for i in range(iter):
        alphas.update_betas()
        env.transition_W(alphas)        # update W at every iteration
        env.update(sin_values)

        # check if we reached consensus
        reached, val = env.reached_consensus()
        if never_reached_consensus and reached:
            print(f"Reached consensus of {val} at timestep {i}")
            never_reached_consensus = False
            consensus = ([i], [val])
            
        spoof_states.append(np.array(env.spoof.tolist()))
        leg_states.append(env.leg)

    title = ""
    for key, val in kwargs.items():
        title += f"{key}: {val}, ".capitalize()
    plot_states(np.array(leg_states), np.array(spoof_states), consensus = consensus, title = title[:-2])

if __name__ == "__main__":

    # picks which question to answer
    answer_question(question)

    # assume everything is in 1-D and fully connected
    spoof = np.array([4, 4, 4, 4])

    # Question 2a: use the 4 spoof nodes and the given legitimate nodes
    if run_question["2a"]:
        leg_2a = np.array([1, 2, 1.1, 1.9, 1.4, 2.3, 0.7, 2, 1, 2, 1, 2, 1, 0.5, 0.8, 1.5, 1, 2, 1, 2])
        run(leg_2a, spoof, Question="2a")

    # legitimate nodes with mean of 1.5 and st.d of 1
    leg = 1.5 + 1 * np.random.randn(20)

    # Question 2b: generate plots with legitimate node states generated from randomness four times; each with a different standard deviation
    if run_question["2b"]:
        standard_deviations = [0.1, 1, 3, 5]
        for std in standard_deviations:
            # Create varoius legitmate node lists with mean = 1.5 and varying st. d
            leg_2b = 1.5 + std * np.random.randn(20)
            run(leg_2b, spoof, Question="2b", st_dev=std)

    # Question 2c: Create spoofs with inital states of 0, 2, 4, and 6
    if run_question["2c"]:
        for val in range(0, 8, 2):
            # Create spoofs with different spoof values
            spoof_2c = np.array([val] * 4)
            run(leg, spoof_2c, Question="2c", spoof_val=val)

    # Question 2d: Testing the following # of spoofers: 6, 8, 10, 14
    if run_question["2d"]:
        for size in [6, 8, 10, 14]:
            spoof_2d = np.array([4] * size)
            run(leg, spoof_2d, Question="2d", spoof_count=size)
    
    # Question 2e
    if run_question["2ei"] or run_question["2eii"]:
        run(leg, spoof, Question=question)