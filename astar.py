import heapq
import numpy as np

distance_matrix = np.load('./env/distance_matrix.npy')

WALL_TOKEN = '@'

class Heapq():
    '''Defines a heap queue with priority on the key and an index as a tiebreaker'''
    def __init__(self, node, key=lambda x: x):
        self.key = key
        self.index = 0
        self.heapq = [(self.key(node), 0, node)]

    def push(self, node):
        self.index += 1
        heapq.heappush(self.heapq, (self.key(node), self.index, node))

    def pop(self):
        return heapq.heappop(self.heapq)[2]

class Node():
    '''AStar Node'''
    def __init__(self, loc, parent=None):
        self.loc = loc
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0

    def get_children(self, grid, constraints=[]):
        '''Get all possible children from node according to valid moves'''
        children = []
        for move in [(0, 0), (0, -1), (0, 1), (-1, 0), (1, 0)]:
            loc = (self.loc[0] + move[0], self.loc[1] + move[1])
            if 0 <= loc[0] < len(grid) and 0 <= loc[1] < len(grid[0]):
                if (grid[loc[0]][loc[1]] != WALL_TOKEN
                    and [self.g + 1, loc] not in constraints):
                    children.append(Node(loc, parent=self))
        return children

    def get_path(self):
        '''Return path from goal to current node'''
        if self.parent is not None:
            return self.parent.get_path() + [self.loc]
        return [self.loc]

def astar(grid, start, goal, constraints=[], t_shift=0, tlim=0):
    '''Astar search algorithm which uses a precomuted distance matrix with the real distance form all nodes to all nodes'''

    start_node = Node(start)
    goal_node = Node(goal)

    start_node.g = t_shift

    open_set = set([(start_node.loc, start_node.g)])
    closed_set = set()
    open_heapq = Heapq(start_node, key=lambda n: n.f)

    while open_set:
        node = open_heapq.pop()
        
        if node.loc == goal_node.loc and node.g > tlim:
            return node.get_path()

        open_set.remove((node.loc, node.g))
        closed_set.add((node.loc, node.g))

        for child in node.get_children(grid, constraints):
            child.g = node.g + 1
            if (child.loc, child.g) in closed_set:
                continue

            child.h = distance_matrix[child.loc[0]][child.loc[1]][goal_node.loc[0]][goal_node.loc[1]]
            #child.f = child.g + child.h
            child.f = child.h # If H is the true distance (precomputed in distance matrix), we don't need G

            if (child.loc, child.g) not in open_set:
                open_set.add((child.loc, child.g))
                open_heapq.push(child)

    # If solution not found, for debugging purposes
    print("Astar couldn't find a solution for:")
    print("\t -Start: {}".format(start))
    print("\t -Goal: {}".format(goal))
    print("\t -Constraints: {}".format(constraints))
    print("\t -Path: {}".format(node.get_path()))

def multi_astar(grid, visits, constraints=[]):
    '''Astar with multiple visits with an already specified order'''
    tlim = max([c[0] for c in constraints], default=0)

    if len(visits) == 1:
        trip = visits
    else:
        trip = astar(grid, visits[0], visits[1], constraints)
        for i in range(2, len(visits)):
            t = astar(grid, visits[i-1], visits[i], constraints, len(trip) - 1)
            trip.extend(t[1:])

    if len(trip) - 1 < tlim: # If constraints in the future keep planing to avoid them
        t = astar(grid, visits[-1], visits[-1], constraints, len(trip) - 1, tlim=tlim)
        trip.extend(t[1:])

    return trip
