from mimetypes import init
from tkinter import E
import numpy as np
import matplotlib.pyplot as plt
import random

# "laplacian" is a function in the scipy library that finds the laplacian matrix of an adjacency matrix
# It finds the laplacian matrix by calculating the degree matrix and subtracting the adjacency matrix
from scipy.sparse.csgraph import laplacian
# We use this library to create deepcopies of lists so we don't permenantly alter the values of the list objects
import copy

# Globals dealing with noise & adversary nodes
add_noise = 1
add_adversary = 0

# Nodes for storing information
class Node(object):

    def __init__(self, init_state):

        # Adds the gaussian noise w st. d = 0.1
        if add_noise:
            init_state += np.random.normal(scale = 0.1)
            
        self._prev_state = np.float64(init_state)
        self._next_state = np.float64(init_state)

    # Store the state update
    def update(self, update):
        # Adds the gaussian noise w st. d = 0.1
        if add_noise:
            update += np.float64(np.random.normal(scale = 0.1))

        self._next_state += update

    # Push the state update
    def step(self):
        self._prev_state = self._next_state

    @property
    def state(self):
        return self._prev_state


# Adversarial node
class AdversaryNode(object):

    def __init__(self, init_state, target, epsilon_adv = 0.01):

        self._prev_state = np.float64(init_state)
        self._next_state = np.float64(init_state)
        
        self._target = target
        self._epsilon = epsilon_adv

    # Store the state update
    def update(self, update):

        # Set update regardless, move update towards the target with epsilon change
        if self._next_state < self._target:
            update = self._epsilon
        elif self._next_state > self._target:
            update = -self._epsilon
        else:
            update = 0

        self._next_state += np.float64(update)

    # Push the state update
    def step(self):
        self._prev_state = self._next_state

    @property
    def state(self):
        return self._prev_state


# Graph for connecting nodes
class Graph(object):

    def __init__(self, node_list, adj_matrix, epsilon = 0.2, threshold = 0, sigma = 0):

        self.node_list = node_list
        self.adj_matrix = adj_matrix
        self._epsilon = epsilon
        self._finished = False      # Bool determining when we've reached a threshold
        self._threshold = threshold
        self._sigma = sigma

    # Update the graph
    def update_graph(self):

        adj = self.adj_matrix
        
        # Representing Oflati-Saber's Equation 15

        # Update each node's next state in the graph
        for i in range(len(adj)):
            curr_node = self.node_list[i]
            sum = 0

            for j in range(len(adj[i])):
                curr_neighbor = self.node_list[j]

                sum += adj[i][j] * (curr_neighbor.state - curr_node.state)
            
            curr_node.update(self._epsilon * sum)

        # Step each node in the graph once updated
        for node in self.node_list:
            node.step()

    # Return the state of the nodes currently - you can disable print here
    def node_states(self):
        string = ""
        out = []
        for node in self.node_list:
            string = string + node.state.astype('str') + "\t"
            out.append(node.state)

        return out

    # Check if the graph has reached consensus somehow, even if there are adversaries
    def is_finished(self):

        # Consensus threshold --> should have a greater threshold when dealing with noise
        t = 0.1 if add_noise else 0.02

        # Potential consensus val
        mean = sum(n.state for n in self.node_list) / len(self.node_list)
        
        at_consensus = True
        for node in self.node_list:
            # Check that each node state is within a threshold
            if (node.state > mean + t) or (node.state < mean - t):
                at_consensus = False
        
        return at_consensus
        
    @property
    def finished(self):
        self._finished = self.is_finished()
        return self._finished


# Return a random adjacency matrix
def rand_adj(dim, p):

    # Initialize a dim x dim matrix with all zeroes
    adj_matrix = np.zeros((dim, dim))

    # Loop through all i,j in the matrix but skip all j,i
    for i in range(dim):
        for j in range(i, dim):

            # Set matrix[i][j] and matrix [j][i] to 1 a random number gen falls under probability p
            adj_matrix[i][j] = adj_matrix[j][i] = int(random.uniform(0, 1) < p)

    return adj_matrix


# Return the Fiedler value to show strong connection of the array
def fiedler(adj_mat):

    # Find the laplacian matrix using the scipy funciton
    lap_matrix = laplacian(adj_mat)

    # Get the eigenvalues of the laplacian matrix
    e_values, _ = np.linalg.eig(lap_matrix)

    # Return the fiedler value (the second smallest eigenvalue) of the lapacian matrix
    e_values.sort()
    return e_values[1]


# Plots the development of node values in the consensus problem over time
def plot_states(node_states, title):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)

    plt.title(title)
    plt.legend()
    plt.show()


if __name__ == "__main__":

    node_list = [Node(4.0), Node(2.0), Node(-1.0), Node(3.0), Node(0.0)]

    # If true, replace the last node with an adversary node
    if add_adversary:
        init_state, target, epsilon = 3.6, 0.0, 0.01
        node_list[-1] = AdversaryNode(init_state, target, epsilon)

    # Linear formation
    linear = np.array([[0, 1, 0, 0, 0],
                        [1, 0, 1, 0, 0],
                        [0, 1, 0, 1, 0],
                        [0, 0, 1, 0, 1],
                        [0, 0, 0, 1, 0]])

    # Circular formation
    circular = np.array([[0, 1, 0, 0, 1],
                        [1, 0, 1, 0, 0],
                        [0, 1, 0, 1, 0],
                        [0, 0, 1, 0, 1],
                        [1, 0, 0, 1, 0]])

    # Fully connected formation
    fully_conected = np.array([[0, 1, 1, 1, 1],
                                [1, 0, 1, 1, 1],
                                [1, 1, 0, 1, 1],
                                [1, 1, 1, 0, 1],
                                [1, 1, 1, 1, 0]])
    
    # Doubly connected formation as described in question 2d
    q2d = np.array([[0, 2, 1, 1, 2],
                    [2, 0, 2, 1, 1],
                    [1, 2, 0, 2, 1],
                    [1, 1, 2, 0, 2],
                    [2, 1, 1, 2, 0]])

    matrix_list = [linear, circular, fully_conected, q2d]
    title_names = ["Linear formation graph", 
                   "Circular formation graph", 
                   "Fully connected formation graph", 
                   "Doubly connected formation graph",
                   "Randomly connected graph: p = "]

    # If we added noise, run many trials to see how long it takes to reach consensus
    if add_noise:

        for i in range(3):
            graph = Graph(copy.deepcopy(node_list), matrix_list[i])

            # Track average consensus time
            avg_consensus_time = 1

            # Run 100 trials
            for j in range(100):

                # Stop if we can't reach consensus in 100 steps
                for k in range(100):
                    # Update the graph at each time step
                    graph.update_graph()

                    # Track the time we reach consensus in
                    if graph.finished:
                        avg_consensus_time += k
                        break
                else:
                    avg_consensus_time += 100
            
            # Average it
            avg_consensus_time /= 100

            print(f"{title_names[i]} average consensus time with noise: {avg_consensus_time}")


    test1 = np.array([[1, 0, 0, 1, 1],
            [0, 1, 1, 1, 0],
            [0, 1, 1, 1, 1],
            [1, 1, 1, 1, 0],
            [1, 0, 1, 0, 0]])


    test2 = np.array([[0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 1, 0, 0],
            [1, 0, 0, 0, 0]])


    test3 = np.array([[1, 0, 1, 0, 1],
            [0, 0, 1, 0, 0],
            [1, 1, 1, 0, 1],
            [0, 0, 0, 0, 1],
            [1, 0, 1, 1, 0]])


    fiedler_matrices = [linear, circular, fully_conected, test1, test2, test3]

    # calculate fiedler values for 6 examples
    for i, m in enumerate(fiedler_matrices):
        f = fiedler(m)
        print(f"{i}) Fielder Value = {f}")
        

    # Plotting graphs: linear, circular, fully and doubly connected
    for i in range(len(matrix_list)):
        graph = Graph(copy.deepcopy(node_list), matrix_list[i])

        # track the node states for the graph
        node_states = [graph.node_states()]

        for j in range(100): 

            # Update the graph at each time step
            graph.update_graph()
            node_states.append(graph.node_states())

            # Stop updating if at consensus
            if graph.finished:
                print(f"Consensus for {title_names[i]} reached at t = {j}")
                break

        # Make plot
        node_states = np.array(node_states)
        plot_states(node_states, title_names[i])
    
    

    # 3 testing probabilities
    ps = [1/10, 1/3, 2/3]

    # Plotting the randomly connected graphs
    for p in ps:

        nodes = copy.deepcopy(node_list)

        # Track the fieldler values of the graph to make an average
        fiedlers = []

        # Track the node states of the graph to plot
        node_states = []
        for j in range(101): 
            # Make a random adj matrix with probability p
            adj_matrix = rand_adj(len(nodes), p)

            # Make a graph with the adj matrix
            graph = Graph(nodes, adj_matrix)
            fiedlers.append(fiedler(adj_matrix))
            node_states.append(graph.node_states())

            # Update the graph at each time step
            graph.update_graph()

            nodes = copy.deepcopy(graph.node_list)

            # Stop if consensus is reached
            if graph.finished:
                print(f"Consensus for {title_names[-1]}{p:.2f} reached at t = {j}")
                break
        
        # Make plot and calculate average fiedler value
        node_states = np.array(node_states)
        plot_states(node_states, "{}{:.2f}".format(title_names[-1], p))
        print(f"Avg fiedler: {sum(fiedlers)/len(fiedlers)}")
 