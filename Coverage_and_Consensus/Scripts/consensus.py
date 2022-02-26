from mimetypes import init
import numpy as np
import matplotlib.pyplot as plt

add_noise = 1

# Graph theory concepts!
# degree == number of neighbors
# path, connectivity
# adjacency list

# A graph is connected if all verticies are connected 


# nodes for storing information
class Node(object):

    def __init__(self, init_state):

        # adds the gaussian noise w st. d = 0.1
        if add_noise:
            init_state += np.random.normal(scale = 0.1)
            
        self._prev_state = init_state
        self._next_state = init_state

    # store the state update
    def update(self, update):
        if add_noise:
            update += np.random.normal(scale = 0.1)
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
    def update(self, update):
    
        raise NotImplementedError

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

        # Returns a list of nodes that are neighbors to the input node
        def neighbors(node):
            neighbor_list = []
            for i in range(len(self.node_list)):
                if self.adj_matrix[node.id][i]:
                    neighbor_list.append(self.node_list[i])
            return neighbor_list


        adj = self.adj_matrix
        
        for i in range(len(adj)):
            curr_node = self.node_list[i]
            sum = 0

            for j in range(len(adj[i])):
                curr_neighbor = self.node_list[j]

                sum += adj[i][j] * (curr_neighbor.state - curr_node.state)
            
            curr_node.update(self._epsilon * sum)

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
        consensus_val = self.node_list[0].state
        for node in self.node_list:
            if node.state != consensus_val:
                return False
        return True
        
    @property
    def finished(self):
        self._finished = self.is_finished()

        return self._finished


# return a random adjacency matrix
def rand_adj(dim, p):

    raise NotImplementedError

# return the Fiedler value to show strong connection of the array
def fiedler(adj_mat):

    raise NotImplementedError


# plots the development of node values in the consensus problem over time
def plot_states(node_states):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)
    
    plt.legend()
    plt.show()


if __name__ == "__main__":

    node_list = [Node(4.0), Node(2.0), Node(-1.0), Node(3.0), Node(0.0)]

    # stoch_altered_node_list = [Node(np.random.normal(scale = 0.1) + node.state()) for node in node_list]

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

    matrix_list = [linear, circular, fully_conected]

    for adj_matrix in matrix_list:
        graph = Graph(node_list, adj_matrix)
        node_states = []
        for _ in range(100):
        # while not graph.finished:
            graph.update_graph()
            node_states.append(graph.node_states())

        node_states = np.array(node_states)
        plot_states(node_states)
    


