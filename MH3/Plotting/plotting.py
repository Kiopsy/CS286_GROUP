import pandas as pd
import matplotlib.pyplot as plt

def create_plot(part):
    # Create a list of to-be-dropped columns
    irrelevant_cols = [0, 1, 2, 4, 5, 6, 8, 9, 10, 11]
    # Convert txt file to csv
    graph_data = pd.read_csv(r'/home/dominicgarrity/CS286/CS286_GROUP/MH3/Plotting/task' + part + '_map_graph.txt', sep=' ', header=None)
    #graph_data.to_csv(r'/home/dominicgarrity/CS286/CS286_GROUP/MH3/Plotting/task2a_map_graph.csv', index=None)
    # Drop irrelevant cols, leaving only x and y
    graph_data = graph_data.drop(irrelevant_cols, axis=1)
    graph_x = graph_data.iloc[:, 0]
    graph_y = graph_data.iloc[:, 1]
    # Plot x and y
    plt.plot(graph_x, graph_y, label="Map Trajectory")

    # Convert txt file to csv
    odom_data = pd.read_csv(r'/home/dominicgarrity/CS286/CS286_GROUP/MH3/Plotting/task' + part + '_odometry.txt', sep=' ', header=None)
    # Drop irrelevant cols, leaving only x and y
    odom_data = odom_data.drop(irrelevant_cols, axis=1)
    odom_x = odom_data.iloc[:, 0]
    odom_y = odom_data.iloc[:, 1]
    # Plot x and y
    plt.plot(odom_x, odom_y, label="Odom Trajectory")

    # Beautify, show, and save plot
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Comparison of Robot Trajectories\nPart " + part)
    plt.legend()
    plt.savefig('/home/dominicgarrity/CS286/CS286_GROUP/MH3/Task_' + part + '/Task_' + part + '_Trajectory_Plot.png')
    plt.show()

# Create variable for easier evaluation
# PART can either be "2a" or "3"
PART = "2"
if PART != "2" and PART != "3":
    raise ValueError("The PART variable can only be '2' or '3'")
create_plot(PART)