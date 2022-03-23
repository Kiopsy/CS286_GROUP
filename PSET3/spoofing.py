import math
from tempfile import SpooledTemporaryFile
import numpy as np
import matplotlib.pyplot as plt

import math

# Run experiments for different questions:
question = "2d"

run_question = {
    "2a" : False,
    "2b" : False,
    "2c" : False,
    "2d" : False,
    "2ei" : False,
    "2eii" : False
}

def answer_question(question):
    for key in run_question.keys():
        run_question[key] = False
    run_question[question] = True

# Question 2e
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
    def update(self, sin_positions=None):

        self.leg = np.matmul(self.W, np.concatenate((self.leg, self.spoof), axis=0))
        self.spoof = np.matmul(self.Theta, self.leg) + np.matmul(self.Omega, self.spoof)

        def sin_func(x):
            B = np.pi * 2 / 50
            C = B / 4
            D = 4
            return  math.sin(B*(x + C)) + D

        if run_question["2ei"]:
            self.spoof.fill(sin_func(self.iter))
        
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


# it plots the states - basically the same function from HW 2
def plot_states(node_states, title = None):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)

    # plot the average
    line, = ax.plot(steps, np.ones(len(steps)) * np.average(node_states[0, :]))
    line.set_label("average")

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


    # Sets the spoof transition function to account for the exponential case in 2e
    omega = np.eye(spoof.shape[0]) * 1.01 if run_question["2eii"] else np.eye(spoof.shape[0]) 


    # Define the environment
    env = Environment(leg, spoof, theta, omega)


    iter = 200

    # Sine values
    # y = sin(2pi/50 (x + 2pi/200)) + 4
    # def sin_func(x):
    #     B = np.pi * 2 / 50
    #     C = B / 4
    #     D = 4
    #     return  np.sin(B(x + C)) + D

    # inputs = np.arange(0, iter)
    # theta = 2 * np.pi
    # period = 50 
    # sin_positions = []
    # if run_question["2ei"]:
    #     for i in range(iter):
    #         sin_positions.append(sin_func(i))


    # Run the simulation and plot
    leg_states = []
    spoof_states = []


    leg_states.append(env.leg)


    for i in range(iter):
        alphas.update_betas()
        env.transition_W(alphas)        # update W at every iteration
        env.update()

        # spoof_states.append(env.spoof)
        spoof_states.append(np.array(env.spoof.tolist()))
        leg_states.append(env.leg)

    title = ""
    for key, val in kwargs.items():
        title += f"{key} = {val}; "
    plot_states(np.array(leg_states), title = title)


    print("out")


if __name__ == "__main__":

    answer_question(question)

    # assume everything is in 1-D and fully connected
    spoof = np.array([4, 4, 4, 4])

    if run_question["2a"]:
        leg_2a = np.array([1, 2, 1.1, 1.9, 1.4, 2.3, 0.7, 2, 1, 2, 1, 2, 1, 0.5, 0.8, 1.5, 1, 2, 1, 2])
        run(leg_2a, spoof, question="2a")

    # legitimate nodes with mean of 1.5 and st.d of 1
    leg = 1.5 + 1 * np.random.randn(20)

    # Question 2b
    if run_question["2b"]:
        standard_deviations = [0.1, 1, 3, 5]
        for std in standard_deviations:
            # Create varoius legitmate node lists with mean = 1.5 and varying st. d
            leg_2b = 1.5 + std * np.random.randn(20)
            run(leg_2b, spoof, question="2b", std=std)

    # Question 2c
    if run_question["2c"]:
        for val in range(0, 8, 2):
            # Create spoofs with different spoof values
            spoof_2c = np.array([val] * 4)
            run(leg, spoof_2c, question="2c", val=val)

    # Question 2d
    #       Testing the following # of spoofers: 6, 8, 10, 14
    if run_question["2d"]:
        for size in [6, 8, 10, 14]:
            spoof_2d = np.array([4] * size)
            run(leg, spoof_2d, question="2d", size=size)
    
    if run_question["2ei"] or run_question["2eii"]:
        run(leg, spoof, question="2e")