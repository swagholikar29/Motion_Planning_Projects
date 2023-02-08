# Basic searching algorithms

# Class for each node in the grid
class Node:
    
    def __init__(self, row, col, is_obs, h=0, parent=None):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.h = h            # h
        self.cost = None      # total cost (depend on the algorithm)
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

    def generate_path(self):

        #making a path list
        path = []

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

##########################################################
# DFS-Step Function - Recursive Function to be used in DFS
##########################################################

def dfs_step(grid, u, goal_node, visited, steps, found):
    visited[u.row][u.col] = 1
    steps += 1
    if ([u.row, u.col] == [goal_node.row, goal_node.col]):
        found = True
        goal_node.parent = u.parent
        return found, steps
    for n in u.neighbours(grid):
        # skip already visited neighbors
        if (visited[n[0]][n[1]] == 1):               
            continue
        v = Node(n[0], n[1], 0, parent=u)
        found, steps = dfs_step(grid, v, goal_node, visited, steps, found)
        if (found == 1):
            return found, steps
    return found, steps


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
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
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
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

    visited[start_node.row][start_node.col] = 1

    #appending start node to queue (using Queue data structure)
    Q=[]
    Q.append(start_node)
    
    while len(Q)>0:
        u=Q.pop(0)             #FIFO operation of Queue data structure
        steps += 1
        
        if ([u.row, u.col] == [goal_node.row, goal_node.col]):
            found = True
            goal_node.parent = u.parent
            break
        
        #explore the neighbours of u in a function
        for x in u.neighbours(grid):
            v = Node(x[0], x[1], grid[x[0]][x[1]], parent=u)
            Q.append(v)
            #remove already visited neighbors from Q
            if (visited[x[0]][x[1]] == 1):     
                Q.remove(v)
            visited[v.row][v.col] = 1

    #generating a path from start to goal
    #path = goal_node.generate_path(start_node)
    path = goal_node.generate_path()

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps

def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
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
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
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

    found, steps = dfs_step(grid, start_node, goal_node, visited, steps, found)

    #generating a path from start to goal
    #path = goal_node.generate_path(start_node)
    path = goal_node.generate_path()

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
