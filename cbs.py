import heapq
from collections import defaultdict
from copy import deepcopy
from astar import multi_astar
import time

class Heapq():
   def __init__(self, node):
        self.index = 0
        self.heapq = [(node.cost, len(node.get_all_conflicts()), 0, node)]

   def push(self, node):
        self.index += 1
        heapq.heappush(self.heapq, (node.cost, len(node.get_all_conflicts()), self.index, node))

   def pop(self):
        return heapq.heappop(self.heapq)[3]

class Node():
    def __init__(self, grid, trips):
        self.constraints = [[] for _ in trips]

        self.paths = [multi_astar(grid, t, c) for t, c in zip(trips, self.constraints)]
        
        self.lengths = [len(s) for s in self.paths]
        self.cost = sum(self.lengths)
        self.makespan = max(self.lengths)
        self.solution = [[p[t] if t < l else p[-1]
            for l, p in zip(self.lengths, self.paths)] for t in range(self.cost)]

    def add_constraint(self, grid, trips, agent, constraint):
        self.constraints[agent].append(constraint)

        self.paths[agent] = multi_astar(grid, trips[agent], self.constraints[agent])
        self.lengths[agent] = len(self.paths[agent])
        self.cost = sum(self.lengths)
        self.makespan = max(self.lengths)
        self.solution = [[p[t] if t < l else p[-1]
            for l, p in zip(self.lengths, self.paths)] for t in range(self.cost)]

    def get_conflict(self):
        for t in range(1, self.cost):
            tally = defaultdict(list)
            for i, s in enumerate(self.solution[t]):
                tally[s].append(i)
            
            for loc, agents in tally.items():
                if len(agents) > 1:
                    return [[a, [t, loc]] for a in agents]
            
            tally = defaultdict(int)
            for i, (s0, s1) in enumerate(zip(self.solution[t-1], self.solution[t])):
                if s1 + s0 in tally:
                    return [[i, [t, s1]], [tally[str(s1 + s0)], [t, s0]]]
                tally[s0 + s1] = i

    def get_all_conflicts(self):
        conflicts = []
        for t in range(1, self.cost):
            tally = defaultdict(list)
            for i, s in enumerate(self.solution[t]):
                tally[s].append(i)
            
            conflicts.extend([[[a, [t, loc]] for a in agents] for loc, agents in tally.items() if len(agents) > 1])
            
            tally = defaultdict(int)
            for i, (s0, s1) in enumerate(zip(self.solution[t-1], self.solution[t])):
                if s1 + s0 in tally:
                    conflicts.extend([[[i, [t, s1]], [tally[str(s1 + s0)], [t, s0]]]])
                tally[s0 + s1] = i

        return conflicts

def cbs(grid, trips, maxt=600):
    open_heapq = Heapq(Node(grid, trips))

    done = 0
    mint = time.time()
    while time.time() - mint < maxt:
        node = open_heapq.pop()

        if not done:
            nconflicts = len(node.get_all_conflicts())
            done += 1

        if not (conflict := node.get_conflict()):
            return node.solution, node.makespan, nconflicts

        for a, c in conflict:
            if c not in node.constraints[a]:
                child = deepcopy(node)
                child.add_constraint(grid, trips, a, c)
                open_heapq.push(child)

    return None, None, nconflicts
