#!/usr/bin/python
import argparse
import glob
from copy import deepcopy
from pathlib import Path
from cbs import CBSSolver_Astar, CBSSolver_SIPP
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from random_instance import random_map, save_map, correct_random_map
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import os
import time as timer
import random

LOWLEVEL = "SIPP"
SOLVER = "CBS"

"""
Driver code for benchmarking/running test instances with different solvers

author(s):  github.com/nicofretti,      William Horvath (github.com/bolded)
            repo: nicofretti/MAPF       repo: lla105/417Project
"""

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals
import json

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--random', action='store_true', default=False,
                        help='Use a random map with auto-genereted agents (see function random_map)')
    parser.add_argument('--benchmark', type=str, default=None,
                        help='Runs on benchmark mode (random, success)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=True,
                        help='Use the disjoint splitting')
    parser.add_argument('--lowlevel', type=str, default=LOWLEVEL,
                        help='The low-level search to use (one of: {SIPP, Astar}), defaults to ' + str(LOWLEVEL))
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()

    if args.benchmark:
        # Benchmark mode
        if args.benchmark == "random":
            map_size = 10;obstacles_dist = .05;max_agents=16
            experiment = 0;max_time = 40
            result = {}
            samples = 25
            start_agents = 4
            for agents in range(start_agents, max_agents+1,2):
                result[agents] = {
                    'cbs_astar': {'cpu_time':[-1]*samples, 'expanded':[-1]*samples},
                    'cbs_sipp': {'cpu_time':[-1]*samples, 'expanded':[-1]*samples},
                }
                for _ in range(samples):
                    print("Samples {} with {} agents".format(_, agents))
                    # only benchmark on maps that can be solved
                    my_map, starts, goals = correct_random_map(map_size, map_size, agents, obstacles_dist)
                    filename = "benchmark/max_agents_{}/test_{}.txt".format(agents, _)
                    os.makedirs(os.path.dirname(filename), exist_ok=True)
                    save_map(my_map, starts, goals, filename)
                    for alg in ['cbs_astar','cbs_sipp']:
                        if alg == 'cbs_astar':
                            solver = CBSSolver_Astar(my_map,starts,goals,max_time)
                        else:
                            solver = CBSSolver_SIPP(my_map,starts,goals,max_time)
                        try:
                            solver.find_solution(disjoint=True)
                            result[agents][alg]['cpu_time'][_] = round(timer.time() - solver.start_time,2)
                        except BaseException as e:
                            # Timeout
                            pass
                        result[agents][alg]['expanded'][_] = solver.num_of_expanded
            with open('benchmark/result.json', 'w') as outfile:
                json.dump(result, outfile)
        if args.benchmark == "success":
            map_size = 10;obstacles_dist = .05;max_agents=16
            max_time = 60
            samples = 25
            start_agents = 4
            result = {}
            #we dont care about generating a correct random map if we're testing success rate
            map, starts, goals = random_map(map_size, map_size, max_agents, obstacles_dist)
            save_map(map, starts, goals, "benchmark/{}_agents_success.txt".format(max_agents))
            for agents in range(start_agents,max_agents + 1,2):
                result[agents] = {
                    'cbs_astar': {'cpu_time': [-1] * samples, 'expanded': [-1] * samples},
                    'cbs_sipp': {'cpu_time': [-1] * samples, 'expanded': [-1] * samples},
                }
                for i in range(samples):
                    # take first i agents
                    random.shuffle(starts);sub_goals = goals[0:agents]
                    random.shuffle(goals);sub_starts = starts[0:agents]
                    print("sample {} with {} agents".format(i,agents))
                    for alg in ['cbs_astar','cbs_sipp']:
                        if alg == 'cbs_astar':
                            solver = CBSSolver_Astar(map,starts,goals,max_time)
                        else:
                            solver = CBSSolver_SIPP(map,starts,goals,max_time)
                        try:
                            solver.find_solution(disjoint=True)
                            result[agents][alg]['cpu_time'][i] = round(timer.time() - solver.start_time, 2)
                        except BaseException as e:
                            # Timeout
                            pass
                        result[agents][alg]['expanded'][i] = solver.num_of_expanded
            print(result)
            with open('benchmark/result_success.json', 'w') as outfile:
                json.dump(result, outfile)

    else:
        result_file = open("results.csv", "w", buffering=1)
        # Otherwise, run the algorithm
        files = ["random.generated"] if args.random else glob.glob(args.instance)
        for file in files:
            print("***Import an instance***")
            my_map, starts, goals = random_map(8, 8, 6, .1) if args.random else import_mapf_instance(file)
            print_mapf_instance(my_map, starts, goals)
            save_map(my_map, starts, goals, 'img/output_map.txt')
            if args.lowlevel == "Astar":
                planner = "Astar"
            elif args.lowlevel == "SIPP":
                planner = "SIPP"
            else:
                raise RuntimeError("Unknown low-level search!")
            
            if args.solver == "CBS":
                if planner == "Astar":
                    cbs = CBSSolver_Astar(my_map, starts, goals)
                    print(f"***Run CBS with Astar***")
                elif planner == "SIPP":
                    cbs = CBSSolver_SIPP(my_map, starts, goals)
                    print(f"***Run CBS with SIPP***")
                paths = cbs.find_solution(args.disjoint)
            elif args.solver == "Independent":
                print(f"***Run Independent with {planner}***")
                solver = IndependentSolver(my_map, starts, goals, planner)
                paths = solver.find_solution()
            elif args.solver == "Prioritized":
                print(f"***Run Prioritized with {planner}***")
                solver = PrioritizedPlanningSolver(my_map, starts, goals, planner)
                paths = solver.find_solution()
            else:
                raise RuntimeError("Unknown solver!")
                

            cost = get_sum_of_cost(paths)
            result_file.write("{},{}\n".format(file, cost))
            result_file.close()
            if not args.batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)
                #animation.save("output.mp4", 1.0)
                animation.show()
    print("***Done***")
    
