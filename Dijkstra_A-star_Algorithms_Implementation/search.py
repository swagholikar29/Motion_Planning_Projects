# Basic searching algorithms
from cmath import inf

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h=0, parent=None, g=inf, cost=inf):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = g            # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = cost      # total cost (depend on the algorithm)
        self.parent = parent  # previous node

    #########################
    #Functions added in class
    #########################
    
    def neighbours(self, grid):

        #shifting to neighbours as required
        shift = [[0,1],[1,0],[0,-1],[-1,0]]

        neighbours = []
        
        for i in range(len(shift)):
            row, col = self.row+shift[i][0], self.col+shift[i][1]
            neighbours.append([row, col])

            #exclude ones outside the grid and with present obstacles
            if (row<0 or col<0 or row>=len(grid) or col>=len(grid[0]) or grid[row][col]==1):
                neighbours.remove([row, col])
        return neighbours

    def generate_path(self, found):

        #making a path list
        path = []

        if found == False:
            return path

        #going reverse from end to start and then reversing the list
        current = self

        while current.parent != None:
            path.append([current.row, current.col])
            current = current.parent
        path.append([current.row, current.col])
        path.reverse()
        return path

#########################################################
#Initializer Function - To be used in both the algorithms
#########################################################

def initializer(grid, start, goal):

    #Initializing start and end nodes
    start_node=Node(start[0], start[1], grid[start[0]][start[1]])
    goal_node=Node(goal[0], goal[1], grid[goal[0]][goal[1]])

    #Checking if start or end positions are obstacles
    obs=0
    if (start_node.is_obs == 1 or goal_node.is_obs == 1):
        obs=1
    
    #Creating a blank visited matrix
    visited=[]

    row=len(grid)
    col=len(grid[0])
    for i in range(row):
        temp=[]
        for j in range(col):
            temp.append(0)
        visited.append(temp)
    
    return start_node, goal_node, visited, obs

###########################################################
#Graph Traversal Function To be used in both the algorithms
###########################################################

def graph_traversal(start_node, goal_node, grid, visited, steps, heuristics):

    found = False
    
    #kept all costs as inf by default. changing for start node
    start_node.g = 0
    if heuristics == True:
        start_node.h = abs(goal_node.row - start_node.row) + abs(goal_node.col - start_node.col)        #only for A-star Algorithm
    start_node.cost = start_node.g + start_node.h
    
    #creating open queue for implementation purposes
    openQ = []
    
    openQ.append(start_node)

    while(len(openQ)>0):
        openQ.sort(key=lambda x: x.cost)
        u = openQ.pop(0)


        #execute iff u is in closed visited list
        if (visited[u.row][u.col] == 0):
            #to add in closed list
            visited[u.row][u.col] = 1
            steps += 1

            if ([u.row, u.col] == [goal_node.row, goal_node.col]):
                found = True
                goal_node.parent = u.parent
                goal_node.g = u.g
                goal_node.cost = u.cost
                break

            #explore the neighbours of u in a function
            for x in u.neighbours(grid):
                if (visited[x[0]][x[1]] == 0):
                    
                    v = Node(x[0], x[1], grid[x[0]][x[1]], parent=u)
                    v.g = min(v.g, u.g + 1)
                    if heuristics == True:
                        v.h = abs(goal_node.row - v.row) + abs(goal_node.col - v.col)                   #only for A-star Algorithm
                    v.cost = v.g + v.h                                                                 
                    openQ.append(v)


    return goal_node, steps, found


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    
    #initiate start, goal node
    start_node, goal_node, visited, obs = initializer(grid, start, goal)
    
    #if start or goal node is obstacle, return message 'No path found'
    if obs!=0:
        print("No path found")
        return path, steps
    
    goal_node, steps, found = graph_traversal(start_node, goal_node, grid, visited, steps, heuristics=False)

    path = goal_node.generate_path(found)
    
    
    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    #initiate start, goal node
    start_node, goal_node, visited, obs = initializer(grid, start, goal)
    
    #if start or goal node is obstacle, return message 'No path found'
    if obs!=0:
        print("No path found")
        return path, steps

    goal_node, steps, found = graph_traversal(start_node, goal_node, grid, visited, steps, heuristics=True)

    path = goal_node.generate_path(found)
    
    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
