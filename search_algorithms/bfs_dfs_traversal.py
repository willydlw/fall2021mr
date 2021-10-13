''' BFS and DFS Traversal Example
    Code reference: https://www.geeksforgeeks.org/breadth-first-traversal-bfs-on-a-2d-array/
    Minor modifications to code linked above

    Note: Algorithm is the same for BFS and DFS.
    The data structures are different.
    BFS uses a FIFO queue
    DFS uses a LIFO stack
'''
# deque is a doubly ended queue 
from collections import deque as queue 

# direction vectors for neighboring cells
# Above (row-1,col+0), right (row+0,col+1)
# below (row+1,col+0), left  (row+0,col-1)
dRow = [-1, 0, 1, 0]
dCol = [ 0, 1, 0, -1]

# Matrix Constants
ROWS = 4
COLS = 4

def isValid(vis, row, col):
    # cell out of bounds?
    if(row < 0 or col < 0 or row >= ROWS or col >= COLS):
        return False 
    # cell already visited
    if(vis[row][col]):
        return False
    # valid location, not visited
    return True 

def bfs(grid, vis, row, col):
    '''
    Performs breadth-first search traversal
    Iterative implementation
    grid - 2D array to be traversed
    vis - visited array
    row, col - starting location 
    '''
    # store indices of cells
    q = queue()

    # mark starting cell as visited
    # add it to the queue
    q.append((row,col))
    vis[row][col] = True 

    # continue looping while queue is not empty
    while(len(q) > 0):
        cell = q.popleft()
        r = cell[0]
        c = cell[1] 
        print(grid[r][c], end = " ")

        # children are in adjacent cells
        for i in range(4):
            adjr = r + dRow[i]
            adjc = c + dCol[i]
            if(isValid(vis, adjr, adjc)):
                q.append((adjr, adjc))
                vis[adjr][adjc] = True

# function is exactly the same as bfs with
# the expception of using pop versus popleft
# queue versus stack
def dfs(grid, vis, row, col):
    '''
    Performs depth-first search traversal
    Iterative implementation
    grid - 2D array to be traversed
    vis - visited array
    row, col - starting location 
    '''
    # store indices of cells
    q = queue()

    # mark starting cell as visited
    # add it to the end of the queue (stack)
    q.append((row,col))
    vis[row][col] = True 

    # continue looping while queue is not empty
    while(len(q) > 0):
        # pop element from stack in LIFO order
        cell = q.pop()
        r = cell[0]
        c = cell[1] 
        print(grid[r][c], end = " ")

        # children are in adjacent cells
        for i in range(4):
            adjr = r + dRow[i]
            adjc = c + dCol[i]
            if(isValid(vis, adjr, adjc)):
                q.append((adjr, adjc))
                vis[adjr][adjc] = True


def printGrid(grid):
    for r in range(ROWS):
        for c in range(COLS):
            print("{:3d}".format(grid[r][c]), end = " ") 
        print("")


if __name__ == '__main__':

    # creates 2D array filled with 0's
    grid = [[ 0 for i in range(COLS)] for j in range(ROWS)]
    
    # populate array with unique integers
    for r in range(ROWS):
        for c in range(COLS):
            grid[r][c] =  r*COLS + c

    print("Grid")
    printGrid(grid)
    # Declare and innitialize visited array 
    vis = [[False for i in range(ROWS)] for i in range(COLS)]

    print("\nBFS Traversal order")
    bfs(grid, vis, 0, 0)

    vis = [[False for i in range(ROWS)] for i in range(COLS)]
    print("\n\nDFS Traversal Order")
    dfs(grid, vis, 0, 0)