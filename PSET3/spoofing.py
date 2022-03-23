import math
from tempfile import SpooledTemporaryFile
import numpy as np
import matplotlib.pyplot as plt

# For part 2e.
sine = False
exponential = True

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
    def update(self, sin_positions):

        self.leg = np.matmul(self.W, np.concatenate((self.leg, self.spoof), axis=0))
        self.spoof = np.matmul(self.Theta, self.leg) + np.matmul(self.Omega, self.spoof)

        if sine:
            self.spoof.fill(sin_positions[self.iter])
            self.iter += 1
        # elif exponential:
            


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


# it plots the states - basically the same function from HW 2
def plot_states(node_states):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)

    # plot the average
    line, = ax.plot(steps, np.ones(len(steps)) * np.average(node_states[0, :]))
    line.set_label("average")

    
    plt.legend()
    plt.show()


if __name__ == "__main__":

    # assume everything is in 1-D and fully connected
    leg = np.array([1, 2, 1.1, 1.9, 1.4, 2.3, 0.7, 2, 1, 2, 1, 2, 1, 0.5, 0.8, 1.5, 1, 2, 1, 2])
    spoof = np.array([4, 4, 4, 4])

    # Create the different plots here:
    # TODO

    # Setting up a new matrix of inputs for the dynamics
    alphas = np.ones((leg.shape[0] + spoof.shape[0], leg.shape[0] + spoof.shape[0]))
    alphas = 0.4 * alphas
    alphas[:, -spoof.shape[0]:] = -1 * alphas[:, -spoof.shape[0]:] 

    alphas = AlphaWeights(alphas)

    theta = np.zeros((spoof.shape[0], leg.shape[0]))

    # Sets the spoof transition function to account for the exponential case in 2e
    omega = np.eye(spoof.shape[0]) * 1.01 if exponential else np.eye(spoof.shape[0]) 

    # Define the environment
    env = Environment(leg, spoof, theta, omega)

    iter = 200

    # Sine values
    inputs = np.arange(0, iter)
    theta = 2 * np.pi
    period = 50 
    sin_positions = np.sin((theta / period) * inputs) + 4

    # Run the simulation and plot
    leg_states = []
    spoof_states = []

    leg_states.append(env.leg)
    for _ in range(iter):
        alphas.update_betas()
        env.transition_W(alphas)        # update W at every iteration
        env.update(sin_positions)

        # spoof_states.append(env.spoof)
        spoof_states.append(np.array(env.spoof.tolist()))
        leg_states.append(env.leg)

    plot_states(np.array(leg_states))

    print("out")