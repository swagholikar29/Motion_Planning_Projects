# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np

import math


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###

        #Ref from: 'https://www.w3schools.com/python/ref_math_dist.asp'
        return math.sqrt(math.pow((node1.row - node2.row), 2) + math.pow((node1.col - node2.col), 2))
    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        steps = 70                  #we can increase number of steps as per the requirement

        dx = node2.row - node1.row
        dy = node2.col - node1.col
        
        #checking collision at regular steps between the two points
        #also can be done using equation of line and checking if collision is there on line
        x_step = dx / steps
        y_step = dy / steps

        x_pt = node1.row
        y_pt = node1.col

        for i in range(steps):
            if (self.map_array[int(x_pt)][int(y_pt)] == 0):              # check if obstacle is present
                return False
            x_pt = x_pt + x_step
            y_pt = y_pt + y_step
        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###

        # generate a random node in the limits of grid
        node_rand = Node(np.random.randint(0, self.size_row), np.random.randint(0, self.size_col)) 

        # get point considering the goal_bias
        point = np.random.choice([self.goal, node_rand], p=[goal_bias, 1 - goal_bias])
        return point

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###

        #initializing minimum distance as infinity (a very large number)
        min_dis = 1000000000

        #check for all the appended vertices, which is nearest
        for vertex in self.vertices:
            if(min_dis > self.dis(vertex, point)):
                min_dis = self.dis(vertex, point)
                node_nearest = vertex                   #update the nearest node every time whenever you get the less distance than the shortest
        return node_nearest


    #####################################################
    # Get Neighbors Function to be explicitly use in RRT*
    #####################################################

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###

        #create empty list of neighbors
        neighbors = []

        #check and append the nearby vertices within specific distance
        for vertex in self.vertices:
            if(self.dis(vertex, new_node) < neighbor_size):
                neighbors.append(vertex)
        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###

        #for a given node, check for all its neighbors
        for neighbor in neighbors:
            #check cost when connected to nearest neighbor
            new_neighbor_cost = new_node.cost + self.dis(neighbor, new_node)

            #continue if there is collision and we are not able to connect new_node and neighbor
            if(self.check_collision(new_node, neighbor) == False):
                continue

            #check for the neighbor cost and update
            if(new_neighbor_cost < neighbor.cost):
                neighbor.cost = new_neighbor_cost
                neighbor.parent = new_node

    def expand(self, node1, node2):

        dist_extend = 12   # maximum distance to extend the nodes
        
        # Avoid goal to be some other node's parent and if node is nearby less than dist_extend, just return node 2
        if(self.dis(node1, node2) <= dist_extend) and \
            (node2.row != self.goal.row) and (node2.col != self.goal.col):
            return node2
        
        else:
            dx = node2.row - node1.row
            dy = node2.col - node1.col
            mod = self.dis(node1, node2)
            stepx = dx * dist_extend / mod          
            stepy = dy * dist_extend / mod

            # extending node in the direction of random point by step magnitude
            x = node1.row + stepx               
            y = node1.col + stepy

            # checking for the boundaries of grid
            if(x < 0): 
                x = 0
            elif(x > self.size_row): 
                x = self.size_row - 1
            if(y < 0): 
                y = 0
            elif(y > self.size_col): 
                y = self.size_col - 1

            # updating node attributes
            new_node = Node(x, y)
            new_node.parent = node1
            new_node.cost = node1.cost + self.dis(new_node, node1)
            return new_node

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        for point in range(n_pts):
            # In each step,
            # get a new point, 
            goal_bias = 0.025                           #can be updated as per requirement
            new_point = self.get_new_point(goal_bias)
                
            # get its nearest node,
            node_nearest = self.get_nearest_node(new_point)

            # extend the node and check collision to decide whether to add or drop,
            temp_node = self.expand(node_nearest, new_point)                            

            # check collision to decide whether to keep the node or drop
            if(self.check_collision(node_nearest, temp_node) == True):
                    temp_node.parent = node_nearest
                    temp_node.cost = node_nearest.cost + self.dis(temp_node, node_nearest)
                    self.vertices.append(temp_node)                                     # append the virtices for valid nodes

            # if added, check if reach the neighbor region of the goal.
            # check if can connect this node to goal or it collides
            if(self.check_collision(temp_node, self.goal) == False):
                continue

            # check if we have reached the goal
            const_goal_dist = 15
            if(self.dis(temp_node, self.goal) < const_goal_dist):
                self.found = True
                self.goal.parent = temp_node
                self.goal.cost = temp_node.cost + self.dis(temp_node, self.goal)
                break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        for point in range(n_pts):
            # In each step,
            # get a new point, 
            goal_bias = 0.025                           #can be updated as per requirement
            new_point = self.get_new_point(goal_bias)

            # get its nearest node, 
            node_nearest = self.get_nearest_node(new_point)

            # extend the node and check collision to decide whether to add or drop,
            temp_node = self.expand(node_nearest, new_point)                            #write the extend function

            # addition to RRT: checking the neighbors of new point before attaching parent
            if(self.check_collision(node_nearest, temp_node) == True):
                neighbors = self.get_neighbors(temp_node, neighbor_size)
                min_node = node_nearest
                min_cost = node_nearest.cost + self.dis(node_nearest, temp_node)
                
                # check between the neighbors to get minimum cost
                for neighbor in neighbors:
                    if(self.check_collision(neighbor, temp_node) == True):
                        if((neighbor.cost + self.dis(neighbor, temp_node)) < min_cost):
                            min_node = neighbor
                            min_cost = neighbor.cost + self.dis(neighbor, temp_node)

                temp_node.parent = min_node
                temp_node.cost = min_cost
                self.vertices.append(temp_node)

                # if we get any other neighboring node with min cost, rewire
                self.rewire(temp_node, neighbors)

            # if added, rewire the node and its neighbors,
            # and check if reach the neighbor region of the goal if the path is not found.
            goal_neighbors = self.get_neighbors(self.goal, neighbor_size)
            for neighbor in goal_neighbors:
                if(self.check_collision(neighbor, self.goal) == True):
                    if((neighbor.cost + self.dis(neighbor, self.goal)) < self.goal.cost):
                        self.goal.parent = neighbor
                        self.goal.cost = neighbor.cost + self.dis(neighbor, self.goal)
                        self.found = True

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
