# Capacitated MAPD with OR-Tools and CBS

This code integrates [OR-Tools](https://developers.google.com/optimization/routing/vrp) to perform the task assignment for Multi-Agent Pickup and Delivery problems. Conflict Based Search (CBS) resolves the conflicts that might appear in the routes ititally proposed by OR-Tools.

## How to use it?

### Generating instances

1. Decide which agent and task size you want to use. (max 10 agents and 20 tasks)

2. Create directory to place the instances

```
mkdir instances/(agent size here)_(task size here)/agents
mkdir instances/(agent size here)_(task size here)/tasks
```

3. Generate the instances

Remember to assign the sizes you want to use in the file generate_instances.py, then:

```
python3 generate_instances.py
```

### Evaluation

Remember to assign the sizes you are using to use in the file evaluation_ortools.py, then:

```
python3 evaluation_ortools.py
```
