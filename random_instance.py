import random

from cbs import CBSSolver_Astar, CBSSolver_SIPP

"""
author(s):  github.com/nicofretti
            repo: nicofretti/MAPF
"""

def correct_random_map(height, width, agents, obstacles_percentage):
    # Creates a random map with the given parameters
    map = [];starts = [];goals = []
    positions = [(x,y) for x in range(height) for y in range(width)]
    random.shuffle(positions)

    for _ in range(agents):
        starts.append(positions.pop())
        goals.append(positions.pop())

    for i in range(height):
        map.append([])
        for j in range(width):
            if (i,j) in starts or (i,j) in goals or random.random() > obstacles_percentage:
                # mark as obstacle
                map[i].append(False)
            else:
                # free space
                map[i].append(True)
    # Check if the map is valid
    try:
        solver = CBSSolver_Astar(map, starts, goals, 30) # exists a solution in 30 seconds
        solver.find_solution(disjoint=True)
    except BaseException as e:
        # Map is not valid
        print(f"No solution (CBS) in 30 seconds, new map {height}x{width} {agents} agents {obstacles_percentage} obs%")
        # try again
        return correct_random_map(height, width, agents, obstacles_percentage)
    # Make sure SIPP can also solve the same map (otherwise it's not useful for performance benchmark)
    try: 
        solver2 = CBSSolver_SIPP(map, starts, goals, 30) # exists a solution in 30 seconds
        solver.find_solution()
    except BaseException as e:
        # Map is not valid
        print(f"No solution (SIPP) in 30 seconds, new map {height}x{width} {agents} agents {obstacles_percentage} obs%")
        # try again
        return correct_random_map(height, width, agents, obstacles_percentage)
    return map, starts, goals

def random_map(height, width, agents, obstacles_percentage):
    # Creates a random map with the given parameters
    map = [];starts = [];goals = []
    positions = [(x,y) for x in range(height) for y in range(width)]
    random.shuffle(positions)

    for _ in range(agents):
        starts.append(positions.pop())
        goals.append(positions.pop())

    for i in range(height):
        map.append([])
        for j in range(width):
            if (i,j) in starts or (i,j) in goals or random.random() > obstacles_percentage:
                # mark as obstacle
                map[i].append(False)
            else:
                # free space
                map[i].append(True)
    return map, starts, goals

def add_random_agent(map, starts, goals):
    # function that return a possible start and goal of a new agent
    height = len(map)
    width = len(map[0])
    goal = (random.randint(0, height-1), random.randint(0, width-1))
    while goal in starts or goal in goals or not map[goal[0]][goal[1]]:
        goal = (random.randint(0, height-1), random.randint(0, width-1))
    start = (random.randint(0, height - 1), random.randint(0, width - 1))
    while start in starts or start in goals or not map[start[0]][start[1]] and goal!=start:
        start = (random.randint(0, height-1), random.randint(0, width-1))
    starts.append(start)
    goals.append(goal)
    try:
        solver = CBSSolver_Astar(map, starts, goals, 60*5) # exists a solution in 5 minutes
        solver.find_solution()
    except BaseException as e:
        # Map is not valid
        print("No solution, too many agents")
    return start,goal

def save_map(map, starts, goals, filename):
    # Saves the map to a file
    with open(filename, 'w') as f:
        f.write('{} {}\n'.format(len(map),len(map[0])))
        for row in map:
            f.write('{}\n'.format(''.join(['@ ' if cell else '. ' for cell in row])[:-1]))
        f.write('{}\n'.format(len(starts)))
        for agent in range(len(starts)):
            f.write('{} {} {} {}\n'.format(starts[agent][0], starts[agent][1], goals[agent][0], goals[agent][1]))

if __name__== '__main__':
    # Generates a random map and saves it to a file
    map, starts, goals = random_map(10, 10, 5, 0.5)
    save_map(map, starts, goals, 'img/output_map.txt')
