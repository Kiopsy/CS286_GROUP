from mimetypes import init
from tkinter import E
import numpy as np
import matplotlib.pyplot as plt
import random

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
        c = self.node_list[0].state
        
        at_consensus = True
        for node in self.node_list:
            # Check that each node state is within a threshold
            if (node.state > c + t) or (node.state < c - t):
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
    # SOURCES
    # Read pg 7 : https://www.math.umd.edu/~immortal/MATH401/book/ch_graph_theory.pdf
    # Eigenvalues : https://lpsa.swarthmore.edu/MtrxVibe/EigMat/MatrixEigen.html
    # Laplacian Matrix = Degree matrix - Adjacency Matrix
    # https://datascience.stackexchange.com/questions/54414/how-do-i-generate-a-laplacian-matrix-for-a-graph-dataset

    if False:
        # METHOD 1 : FIND THE LAPLACIAN MATRIX FROM FINDING THE DEGREE MATRIX AND SUBTRACTING THEM
        deg_matrix = []
        for i in range(len(adj_mat)):
            new_row = [0] * len(adj_mat[0])
            new_row[i] = np.count_nonzero(adj_mat[0])
            deg_matrix.append(new_row)
        
        lap_matrix = np.subtract(deg_matrix, adj_mat)
    else:
        # METHOD 2 : FIND THE LAPLACIAN MATRIX USING A SCIPY FUNCTION
        from scipy.sparse.csgraph import laplacian
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
    q2d = np.array([[0, 2, 1, 1, 1],
                    [2, 0, 2, 1, 1],
                    [1, 2, 0, 2, 1],
                    [1, 1, 2, 0, 2],
                    [1, 1, 1, 2, 0]])

    matrix_list = [linear, circular, fully_conected, q2d]
    title_names = ["Linear formation graph", 
                   "Circular formation graph", 
                   "Fully connected formation graph", 
                   "Doubly connected formation graph",
                   "Randomly connected graph: p = "]

    # Plotting graphs: linear, circular, fully and doubly connected
    for i in range(len(matrix_list)):
        graph = Graph(copy.deepcopy(node_list), matrix_list[i])
        node_states = [graph.node_states()]
        for j in range(100): 
            graph.update_graph()
            node_states.append(graph.node_states())
            if graph.finished:
                print(f"Consensus for {title_names[i]} reached at t = {j}")
                break

        node_states = np.array(node_states)
        plot_states(node_states, title_names[i])
    
    # Plotting the randomly connected graphs
    ps = [1/10, 1/3, 2/3]
    for p in ps:
        nodes = copy.deepcopy(node_list)
        node_states = []
        for j in range(101): 
            graph = Graph(nodes, rand_adj(len(nodes), p))
            node_states.append(graph.node_states())
            graph.update_graph()
            nodes = copy.deepcopy(graph.node_list)
            if graph.finished:
                print(f"Consensus for {title_names[-1]}{p:.2f} reached at t = {j}")
                break
            
        node_states = np.array(node_states)
        plot_states(node_states, "{}{:.2f}".format(title_names[-1], p))
 