from mimetypes import init
from tkinter import E
import numpy as np
import matplotlib.pyplot as plt
import random

# Globals dealing with noise & adversary nodes
add_noise = 1
add_adversary = 0

# nodes for storing information
class Node(object):

    def __init__(self, init_state):

        # adds the gaussian noise w st. d = 0.1
        if add_noise:
            init_state = np.float64(np.random.normal(loc = init_state, scale = 0.1))
            
        self._prev_state = init_state
        self._next_state = init_state

    # store the state update
    def update(self, update):
        # adds the gaussian noise w st. d = 0.1
        if add_noise:
            update = np.float64(np.random.normal(loc = update, scale = 0.1))
        self._next_state += update

    # push the state update
    def step(self):
        self._prev_state = self._next_state

    @property
    def state(self):
        return self._prev_state


# adversarial node
class AdversaryNode(object):

    def __init__(self, init_state, target, epsilon_adv = 0.01):

        self._prev_state = init_state
        self._next_state = init_state
        
        self._target = target
        self._epsilon = epsilon_adv

    # store the state update
    def update(self, update = None):

        if not update:
            if self._next_state < self._target:
                update = self._epsilon
            elif self._next_state > self._target:
                update = -self._epsilon
            else:
                update = 0

        self._next_state += update

    # push the state update
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
        self._finished = False      # bool determining when we've reached a threshold
        self._threshold = threshold
        self._sigma = sigma

    # update the graph
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

    # return the state of the nodes currently - you can disable print here
    def node_states(self):
        string = ""
        out = []
        for node in self.node_list:
            string = string + node.state.astype('str') + "\t"
            out.append(node.state)

        return out

    # check if the graph has reached consensus somehow, even if there are adversaries
    def is_finished(self):
        return all(self.node_list[0].state == n.state for n in self.node_list)
        
    @property
    def finished(self):
        self._finished = self.is_finished()
        return self._finished


# return a random adjacency matrix
def rand_adj(dim, p):

    adj_matrix = np.zeros((dim, dim))

    for i in range(dim):
        for j in range(i, dim):
            adj_matrix[i][j] = adj_matrix[j][i] = int(random.uniform(0, 1) < p)

    print(adj_matrix)
    return adj_matrix


# return the Fiedler value to show strong connection of the array
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

    e_values, _ = np.linalg.eig(lap_matrix)

    e_values.sort()

    return e_values[1]


# plots the development of node values in the consensus problem over time
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
        init_state, target, epsilon = 0, 0, 0
        node_list.append(AdversaryNode(init_state, target, epsilon))

    # linear formation
    linear = np.array([[0, 1, 0, 0, 0],
                            [1, 0, 1, 0, 0],
                            [0, 1, 0, 1, 0],
                            [0, 0, 1, 0, 1],
                            [0, 0, 0, 1, 0]])

    # circular formation
    circular = np.array([[0, 1, 0, 0, 1],
                            [1, 0, 1, 0, 0],
                            [0, 1, 0, 1, 0],
                            [0, 0, 1, 0, 1],
                            [1, 0, 0, 1, 0]])

    # fully connected formation
    fully_conected = np.array([[0, 1, 1, 1, 1],
                            [1, 0, 1, 1, 1],
                            [1, 1, 0, 1, 1],
                            [1, 1, 1, 0, 1],
                            [1, 1, 1, 1, 0]])

    # fully connected formation as described in question 2 d
    q2d = np.array([[0, 2, 1, 1, 1],
                    [2, 0, 2, 1, 1],
                    [1, 2, 0, 2, 1],
                    [1, 1, 2, 0, 2],
                    [1, 1, 1, 2, 0]])

    matrix_list = [linear, circular, fully_conected, q2d]
    title_names = ["Linear formation graph", 
                   "Circular formation graph", 
                   "Fully connected formation graph", 
                   "Doubly connected formation graph"]

    for i in range(len(matrix_list)):
        graph = Graph(node_list, matrix_list[i])
        node_states = []
        for _ in range(100): 
            graph.update_graph()
            node_states.append(graph.node_states())

        node_states = np.array(node_states)
        plot_states(node_states, title_names[i])
    


