import random

# PARAMETERS
N_TASKS = 40
N_AGENTS = 20

N_INSTANCES = 50

# LOAD GRID
with open('./env/grid.txt', 'r') as f:
    grid = [l.strip() for l in f.readlines()]

grid = [['e' if item == 'G' else item for item in row] for row in grid]

n_endpoints = sum(1 if item == 'e' else 0 for row in grid for item in row) - N_AGENTS

# GENERATE FILES
for k in range(N_INSTANCES):
    typecell = {'.': [], 'e': [], '@': []}
    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            typecell[cell].append((i, j))
    random.shuffle(typecell['e'])

    agents = [typecell['e'].pop() for _ in range(N_AGENTS)]
    tasks = [[typecell['e'].pop(), typecell['e'].pop()] for _ in range(N_TASKS)]

    for a in agents:
        grid[a[0]][a[1]] = 'r'

    with open('./instances/' + str(N_AGENTS) + '_' + str(N_TASKS) + '/maps/' + str(k) + '.map', 'w') as f:
        f.write(','.join([str(len(grid)), str(len(grid[0]))]))
        f.write('\n' + str(n_endpoints))
        f.write('\n' + str(N_AGENTS))
        f.write('\n' + '5000')
        
        for row in grid:
            f.write('\n' + ''.join(row))

    for a in agents:
        grid[a[0]][a[1]] = 'e'

    with open('./instances/' + str(N_AGENTS) + '_' + str(N_TASKS) + '/tasks/' + str(k) + '.task', 'w') as f:
        f.write(str(N_TASKS))
        
        for t in tasks:
            f.write('\n' + '\t'.join(['0',
            str(t[0][0] * len(grid[0]) + t[0][1]), str(t[1][0] * len(grid[0]) + t[1][1]),
            '0', '0']))
