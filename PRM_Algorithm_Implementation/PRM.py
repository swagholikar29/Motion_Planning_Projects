# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx

import math
from scipy import spatial


cov_gauss = [[400, 0],
            [0, 400]]
cov_bridge = [[625, 0],
            [0, 625]]
k_neighbors = 40
sample_radius = 15
step_uniform = 10


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        steps = 70                  #we can increase number of steps as per the requirement

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        #checking collision at regular steps between the two points
        #also can be done using equation of line and checking if collision is there on line
        x_step = dx / steps
        y_step = dy / steps

        x_pt = p1[0]
        y_pt = p1[1]

        for i in range(steps):
            if (self.map_array[int(x_pt)][int(y_pt)] == 0):              # check if obstacle is present
                return True
            x_pt = x_pt + x_step
            y_pt = y_pt + y_step
        return False
        


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        #Ref from: 'https://www.w3schools.com/python/ref_math_dist.asp'

        ### YOUR CODE HERE ###
        return math.dist(point1, point2)


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        #according to n_pts number, step_uniform needs change
        for row in range(0, self.size_row, step_uniform):
            for col in range(0, self.size_col, step_uniform):
                self.samples.append((row,col))
                
                if(self.map_array[row][col] == 0):                                   #remove point if colliding with obstacles
                    self.samples.remove((row,col))
                

    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        for i in range(n_pts):
            
            x = np.random.randint(0, self.size_row)
            y = np.random.randint(0, self.size_col)
            
            #sampled_point = [x, y]

            self.samples.append((x, y))

            if (self.map_array[x][y] == 0):          #remove if sampled point is in collision with the obstacle
                self.samples.remove((x, y))


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        for i in range(n_pts):

            x = np.random.randint(0, self.size_row)
            y = np.random.randint(0, self.size_col)
            q1 = [x, y]

            #Ref from: 'https://numpy.org/doc/stable/reference/random/generated/numpy.random.multivariate_normal.html'
            q2 = np.random.multivariate_normal(q1, cov_gauss, size=1)[0] 
            q2 = [int(q) for q in q2]

            #if q2 is out of grid, discard
            m = self.size_row - 1
            n = self.size_col - 1
            if ((q2[0] > m) or (q2[1] > n)):
                continue

            #both in collision or collision-free, discard 
            if (self.map_array[q1[0]][q1[1]] == 1 and self.map_array[q2[0]][q2[1]] == 1):
                continue
            if (self.map_array[q1[0]][q1[1]] == 0 and self.map_array[q2[0]][q2[1]] == 0):
                continue
            
            #when one point in collision and other is collision-free, sample the collision-free
            if (self.map_array[q1[0]][q1[1]] == 1):
                self.samples.append((q1[0], q1[1]))
            else:
                self.samples.append((q2[0], q2[1])) 


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        for i in range(n_pts):

            x = np.random.randint(0, self.size_row)
            y = np.random.randint(0, self.size_col)
            q1 = [x, y]

            #sample q2 iff q1 is in collision, else continue
            if(self.map_array[q1[0]][q1[1]] == 1):
                continue
            
            #Ref from: 'https://numpy.org/doc/stable/reference/random/generated/numpy.random.multivariate_normal.html'
            q2 = np.random.multivariate_normal(q1, cov_bridge, size=1)[0] 
            q2 = [int(q) for q in q2]

            #if q2 is out of grid, discard
            if((q2[0] > self.size_row-1) or (q2[1] > self.size_col-1)):
                continue

            #proceed only if q2 is in collision
            if(self.map_array[q2[0]][q2[1]] == 0):
                mid = [round((a+b)/2) for (a, b) in zip(q1, q2)]
                if(self.map_array[mid[0]][mid[1]] == 1):
                    self.samples.append((mid[0], mid[1]))


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        pairs = []

        #Ref from: 'https://stackoverflow.com/questions/13796782/networkx-random-geometric-graph-implementation-using-k-d-trees'
        kdtree = spatial.KDTree(self.samples)
        p_ids = kdtree.query_pairs(sample_radius)
        

        for p_id in p_ids:
            point_1 = p_id[0]
            point_2 = p_id[1]
            
            #remove the pairs in collision
            if self.check_collision(list(self.samples[point_1]), list(self.samples[point_2])) == True: 
                continue

            weight = self.dis(self.samples[point_1], self.samples[point_2])

            pairs.append((point_1, point_2, weight))

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []

        #Ref from: 'https://stackoverflow.com/questions/13796782/networkx-random-geometric-graph-implementation-using-k-d-trees'
        kdtree = spatial.KDTree(self.samples)
        _, p_ids = list(kdtree.query([start, goal], k_neighbors))       #making list of k-nearing neighbours from start and goal

        for i in range(k_neighbors):
            p_start = p_ids[0][i]
            p_goal = p_ids[1][i]

            #remove the pairs in collision
            if self.check_collision(list(self.samples[p_start]), list(start)) == True: 
                continue

            weight = self.dis(self.samples[p_start], start)
            start_pairs.append(('start', p_start, weight))

            #remove the pairs in collision
            if self.check_collision(list(self.samples[p_goal]), list(goal)) == True: 
                continue

            weight = self.dis(self.samples[p_goal], goal)
            goal_pairs.append(('goal', p_goal, weight))


        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        