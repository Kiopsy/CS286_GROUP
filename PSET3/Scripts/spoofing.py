import math
from tempfile import SpooledTemporaryFile
import numpy as np
import matplotlib.pyplot as plt

# Run experiments for different questions by changing this variable
# The variable must equal one of the keys of run_question
question = "2a"

# Define dictionary for evaluations
run_question = {
    "2a" : False,
    "2b" : False,
    "2c" : False,
    "2d" : False,
    "2ei" : False,
    "2eii" : False
}

# Set corresponding question value to True
def answer_question(question):
    for key in run_question:
        run_question[key] = False
    run_question[question] = True

# Create variables for question 2e
sine = False
exponential = True

# This returns a bunch of weights, sets a mean alpha level, then randomly generates alphas around the means
class AlphaWeights(object):

    def __init__(self, mean_alpha, std = 0.1):

        self.std = std
        self.iter = 1

        # Assuming fully connected - this is an adjancency matrix
        # Assume the vector is [legitimate, spoofed], assume it's diagraph
        self.means = mean_alpha
        self.betas = self.means + np.random.randn(self.means.shape[0], self.means.shape[1]) * self.std

    # Calculate the updates for the betas used in updating W
    # Be sure to cap alpha at -0.5 and 0.5 at all times
    # This function should generate a random set of alphas based on the means and then update beta accordingly
    def update_betas(self):

        alpha = self.means + np.random.randn(self.means.shape[0], self.means.shape[1]) * self.std

        # Cap alphas at 0.5 as specified in the paper
        thresh = alpha > 0.5
        alpha[thresh] = 0.5
        thresh = alpha < -0.5
        alpha[thresh] = -0.5

        # Update beta because it's a running sum of alphas
        self.betas += alpha
        
        self.iter += 1


# Define the simulation environment
class Environment(object):

    def __init__(self, leg, spoof, Theta, Omega):

        self.leg = leg         # legitimate nodes list n_l
        self.spoof = spoof     # spoofed nodes list n_s

        self.leg_len = leg.shape[0]
        self.full_len = leg.shape[0] + spoof.shape[0]

        self.Theta = Theta              # transition for spoofed based on leg - n_s x n_l   (row, col)
        self.Omega = Omega              # transition for spoofed based on spoofed - n_s x n_s

        # Transition for legitimate based on spoof and leg - n_l x (n_l + n_s)
        # First n_l columns are the legit part W_L
        self.W = np.zeros((self.leg_len, self.full_len))

        self.iter = 0

    # Updates according to the system dynamics given
    def update(self, sin_positions=None):

        self.leg = np.matmul(self.W, np.concatenate((self.leg, self.spoof), axis=0))
        self.spoof = np.matmul(self.Theta, self.leg) + np.matmul(self.Omega, self.spoof)

        # Modify spoofing states such that they follow sine pattern
        if run_question["2ei"]:
            self.spoof.fill(sin_positions[self.iter])
        
        self.iter += 1

    # Set the transitions of W
    # Call alphaweights to get an updated beta, then use that to update W.
    # The code will call update to progress the simulation
    def transition_W(self, alphaweights):
        betas = alphaweights.betas
        
        # Change each element of weighting matrix W according to algorithm
        for i in range(self.W.shape[0]):
            for j in range(self.W.shape[1]):
                if betas[i][j] >= 0:
                    self.W[i][j] = (1/self.leg_len)*(1 - math.exp(-betas[i][j] / 2))
                else:
                    self.W[i][j] = (1/(2*self.leg_len))*(math.exp(betas[i][j]))

        # Modify W when i == j
        for i in range(self.W.shape[0]):
            self.W[i][i] += 1 - (np.sum(self.W[i]))


# Plot states like in HW 2
def plot_states(node_states, q, file_mod, title = None):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label("Node " + str(i))

    # Plot the average state
    line, = ax.plot(steps, np.ones(len(steps)) * np.average(node_states[0, :]))
    line.set_label("Average")

    # Add title to plot
    if title:
        ax.set_title(title)
    
    # Add labels to plot axes
    plt.xlabel("Iteration")
    plt.ylabel("State")

    # Add legend to plot
    plt.legend(loc='upper right', bbox_to_anchor=(1, 1), ncol=2)

    # Construct path and file names based on user input and problem parameters
    path_name = "/home/dominicgarrity/CS286/CS286_GROUP/PSET3/Images/Prob_2/" + q
    if file_mod != "":
        file_name = "/Prob_" + q + "_" + file_mod + "_Plot.png"
    else:
        file_name = "/Prob_" + q + "_Plot.png"
    path_name += file_name

    # Save plot
    plt.savefig(path_name, bbox_inches='tight')

    plt.show()

def run(leg, spoof, **kwargs):
    # Print problem parameters
    print(kwargs)
    
    # Set up a new matrix of inputs for the dynamics
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

    # Define sine values
    step = 2 * np.pi / 50
    inputs = np.arange(0, iter*step, step)
    sin_values = np.sin(inputs) + 4

    # Run the simulation and plot
    leg_states = []
    spoof_states = []


    leg_states.append(env.leg)


    for i in range(iter):
        alphas.update_betas()
        env.transition_W(alphas)    # update W at every iteration
        env.update(sin_values)

        # spoof_states.append(env.spoof)
        spoof_states.append(np.array(env.spoof.tolist()))
        leg_states.append(env.leg)

    # Modify plot title based on user input and problem parameters
    title = "Node States Over Time"
    kwargs_len = len(kwargs)
    idx = 0
    q = "" # question
    file_mod = "" # modifier for file name
    if kwargs_len > 0:
        # Set title up for the addition of problem parameters
        if kwargs_len > 1:
            title += "\n"
        for key, val in kwargs.items():
            if key == "question":
                q = val
                continue # do not add question to title
            
            # Add problem parameters to title
            if idx != kwargs_len - 2:
                title += f"{key} = {val}, "
                file_mod += f"{key}{val}"
            else:
                title += f"{key} = {val}"
                file_mod += f"{key}{val}"
            idx += 1
    
    plot_states(np.array(leg_states), q, file_mod, title = title)


if __name__ == "__main__":

    # Track user input
    answer_question(question)

    # Assume everything is in 1-D and fully connected
    spoof = np.array([4, 4, 4, 4])

    if run_question["2a"]:
        # Define legitimate states
        leg_2a = np.array([1, 2, 1.1, 1.9, 1.4, 2.3, 0.7, 2, 1, 2, 1, 2, 1, 0.5, 0.8, 1.5, 1, 2, 1, 2])
        run(leg_2a, spoof, question="2A")

    # Make legitimate nodes with mean of 1.5 and std of 1
    leg = 1.5 + 1 * np.random.randn(20)

    # Question 2b
    if run_question["2b"]:
        standard_deviations = [0.1, 1, 3, 5]
        for std in standard_deviations:
            # Create varoius legitmate node lists with mean = 1.5 and varying std
            leg_2b = 1.5 + std * np.random.randn(20)
            run(leg_2b, spoof, question="2B", std=std)

    # Question 2c
    if run_question["2c"]:
        for val in range(0, 8, 2):
            # Create spoofs with different spoof values
            spoof_2c = np.array([val] * 4)
            run(leg, spoof_2c, question="2C", x_s=val)

    # Question 2d
    # Test with the following numbers of spoofers: 6, 8, 10, 14
    if run_question["2d"]:
        for size in [6, 8, 10, 14]:
            spoof_2d = np.array([4] * size)
            run(leg, spoof_2d, question="2D", n_s=size)
    
    if run_question["2ei"]:
        # Test with spoofers following sine pattern
        run(leg, spoof, question="2Ei")
    
    if run_question["2eii"]:
        # Test with spoofers following exponential pattern
        run(leg, spoof, question="2Eii")