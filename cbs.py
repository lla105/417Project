import random
import time as timer
import heapq
from single_agent_planner import compute_heuristics, get_location, get_sum_of_cost, SafeIntervalPlanner, AStarPlanner
import argparse
from math import fabs
from itertools import combinations
from copy import deepcopy
import time as timer
from graph_generation import SIPPGraph, State
import random
import time as timer
DEBUG = False

"""
A planner that plans for each robot independently.

author(s):  github.com/nicofretti,      William Horvath (github.com/bolded)
            repo: nicofretti/MAPF       repo: lla105/417Project
"""

def normalize_paths(pathA, pathB):
    """
    given path1 and path2, finds the shortest path and pads it with the last location
    """
    path1 = pathA.copy()
    path2 = pathB.copy()
    shortest, pad = (path1, len(path2) - len(path1)) if len(path1) < len(path2) else (path2, len(path1) - len(path2))
    for _ in range(pad):
        shortest.append(shortest[-1])
    return path1, path2


def detect_collision(pathA, pathB):
    ##############################
    # Return the first collision that occurs between two robot paths (or None if there is no collision)
    # There are two types of collisions: vertex collision and edge collision.
    # A vertex collision occurs if both robots occupy the same location at the same timestep
    # An edge collision occurs if the robots swap their location at the same timestep.
    # You should use "get_location(path, t)" to get the location of a robot at time t.
    # this function detects if an agent collides with another even after one of the two reached the goal
    path1, path2 = normalize_paths(pathA, pathB)
    length = len(path1)
    for t in range(length):
        # check for vertex collision
        pos1 = get_location(path1, t)
        pos2 = get_location(path2, t)
        if pos1 == pos2:
            # we return the vertex and the timestep causing the collision
            return [pos1], t, 'vertex'
        # check for edge collision (not if we are in the last timestep)
        if t < length - 1:
            next_pos1 = get_location(path1, t + 1)
            next_pos2 = get_location(path2, t + 1)
            if pos1 == next_pos2 and pos2 == next_pos1:
                # we return the edge and timestep causing the collision
                return [pos1, next_pos1], t + 1, 'edge'
    return None


def detect_collisions(paths):
    ##############################
    # Return a list of first collisions between all robot pairs.
    # A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    # causing the collision, and the timestep at which the collision occurred.
    # You should use your detect_collision function to find a collision between two robots.
    collisions = []
    # i and j are agents
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            coll_data = detect_collision(paths[i], paths[j])
            # if coll_data is not None (collision detected)
            if coll_data:
                collisions.append({
                    'a1': i,
                    'a2': j,
                    'loc': coll_data[0],  # vertex or edge
                    'timestep': coll_data[1],  # timestep
                    'type': coll_data[2]
                })
    return collisions


def standard_splitting(collision):
    ##############################
    # Return a list of (two) constraints to resolve the given collision
    # Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                   specified timestep, and the second constraint prevents the second agent to be at the
    #                   specified location at the specified timestep.
    # Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                 specified timestep, and the second constraint prevents the second agent to traverse the
    #                 specified edge at the specified timestep
    #                 in this case, we can ignore final as all the paths are normalized
    constraints = []
    if collision['type'] == 'vertex':
        constraints.append({
            'agent': collision['a1'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
        constraints.append({
            'agent': collision['a2'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
    elif collision['type'] == 'edge':
        constraints.append({
            'agent': collision['a1'],
            'loc': collision['loc'],
            'timestep': collision['timestep'],
            'final': False
        })
        constraints.append({
            'agent': collision['a2'],
            # revesred returns an iterator. In python list == iterator returns false, not an error: nasty bug
            'loc': list(reversed(collision['loc'])),
            'timestep': collision['timestep'],
            'final': False
        })
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Return a list of (two) constraints to resolve the given collision
    # Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                  specified timestep, and the second constraint prevents the same agent to be at the
    #                  same location at the timestep.
    # Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                specified timestep, and the second constraint prevents the same agent to traverse the
    #                specified edge at the specified timestep
    # Choose the agent randomly
    choice = random.randint(0, 1)
    agents = [collision['a1'], collision['a2']]
    agent = agents[choice]
    loc = collision['loc'] if choice == 0 else list(reversed(collision['loc']))
    return [
        {
            'agent': agent,
            'loc': loc,
            'timestep': collision['timestep'],
            'positive': True,
            'final': False
        },
        {
            'agent': agent,
            'loc': loc,
            'timestep': collision['timestep'],
            'positive': False,
            'final': False
        }
    ]


def paths_violate_constraint(constraint, paths):
    ##############################
    # compute the list of agents that violates the positive constraints
    # constraint:{'agent': 0, 'loc': [(2, 4)], 'timestep': 3, 'final': False, 'positive': False}
    # paths:[[(2, 1), ... (3, 4), (3, 5)], [(1, 2), ..., (4, 4)]]

    agents_violate = []
    if len(constraint['loc']) == 1:
        return vertex_check(constraint, paths)
    else:
        return edge_check(constraint, paths)

def vertex_check(constraint, paths):
    agents_violate = []
    for agent in range(len(paths)):
        if constraint['loc'][0] == get_location(paths[agent], constraint['timestep']):
            agents_violate.append(agent)
    return agents_violate

def edge_check(constraint, paths):
    agents_violate = []
    for agent in range(len(paths)):
        loc = [get_location(paths[agent], constraint['timestep'] - 1), get_location(paths[agent], constraint['timestep'])]
        if loc == constraint['loc'] or constraint['loc'][0] == loc[0] or constraint['loc'][1] == loc[1]:
            agents_violate.append(agent)
    return agents_violate

class CBSSolver_Astar(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals, max_time=None):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.start_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.max_time =  max_time if max_time else float('inf')

        self.open_list = []
        self.cont = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        if DEBUG:
            print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        if DEBUG:
            print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=False):
        
        """ Finds paths for all agents from their start locations to their goal locations
        disjoint    - use disjoint splitting or not
        """
        self.start_time = timer.time()
        planner = AStarPlanner(self.my_map)
        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {
            'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': []
        }
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = planner.get_path(self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        # Task 3.1: Testing
        if DEBUG:
            print(root['collisions'])

        # Task 3.2: Testing
        if DEBUG:
            for collision in root['collisions']:
                print(standard_splitting(collision))

        ##############################
        # High-Level Search
        # Repeat the following as long as the open list is not empty:
        #   1. Get the next node from the open list (you can use self.pop_node())
        #   2. If this node has no collision, return solution
        #   3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #      standard_splitting function). Add a new child node to your open list for each constraint
        # Ensure to create a copy of any objects that your child nodes might inherit

        while self.open_list and timer.time() - self.start_time < self.max_time:
            p = self.pop_node()
            # if there are no collisions, we found a solution
            if not p['collisions']:
                self.print_results(p)
                return p['paths']
            # we choose a collision and turn it into constraints
            collision = random.choice(p['collisions'])
            # Adjusting the High-Level Search
            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)
            # HERE
            for c in constraints:
                skip_node = False
                q = {'cost': 0,
                     'constraints': [*p['constraints'], c],  # all constraints in p plus c
                     'paths': p['paths'].copy(),
                     'collisions': []
                     }
                agent = c['agent']
                path = planner.get_path(self.starts[agent], self.goals[agent], self.heuristics[agent], agent, q['constraints'])
                if path:
                    q['paths'][agent] = path
                    if c['positive']:
                        rebuild_agents = paths_violate_constraint(c, q['paths'])
                        for r_agent in rebuild_agents:
                            c_new = c.copy()
                            c_new['agent'] = r_agent
                            c_new['positive'] = False
                            q['constraints'].append(c_new)
                            r_path = planner.get_path(self.starts[r_agent], self.goals[r_agent], self.heuristics[r_agent], r_agent,q['constraints'])
                            if r_path is None:
                                skip_node = True
                                break # at least one agents has none solution
                            else:
                                q['paths'][r_agent] = r_path
                    if(not skip_node):
                        q['collisions'] = detect_collisions(q['paths'])
                        q['cost'] = get_sum_of_cost(q['paths'])
                        self.push_node(q)
                else:
                    raise BaseException('No solutions')
        raise BaseException('Time limit exceeded')

    def print_results(self, node):
        pass
        #if DEBUG:
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

"""
SIPP+CBS implementation  

author(s):  Ashwin Bose (github.com/atb033),            github.com/WheelChan,                       
            repo: atb033/multi_agent_path_planning      repo: WheelChan/Multi-Agent-Path-Finding    

            William Horvath (github.com/bolded)
            repo: lla105/417Project

SIPP article: https://doi.org/10.1109/ICRA.2011.5980306
"""

class HighLevelNode(object):
    def __init__(self):
        self.solution = []
        self.constraint_dict = []
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

class CBSSolver_SIPP(object):
    def __init__(self, my_map, starts, goals, max_time=None):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.start_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.max_time =  max_time if max_time else float('inf')

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(self.my_map, goal))

        self.open_set = set()
        self.closed_set = set()
    
     # finds actual collision and returns path
    def detect_collision(self,path1, path2):    
        max_t = max(len(path1), len(path2))
        #print("max t")
        #print(max_t)

        if max_t == len(path1):
            longer = 1
        else:
            longer = 2
        
        # list of obstacles
        for t in range(max_t):
            if t >= len(path1):
                loc_1 =(path1[len(path1)-1]['x'], path1[len(path1)-1]['y'])
            else:    
                loc_1 =(path1[t]['x'], path1[t]['y'])

            if t >= len(path2):
                loc_2 =(path2[len(path2)-1]['x'], path2[len(path2)-1]['y'])
            else:    
                loc_2 =(path2[t]['x'], path2[t]['y'])

            #for vertex collisions
            if loc_1 == loc_2:
                if longer == 1:
                    return {'typeEdge': False, 'path':path1[t]}
                else:
                    return {'typeEdge': False, 'path':path2[t]}

            #for edge collisions
            if t == 0:
                prev_loc1 = loc_1
                prev_loc2 = loc_2
            else:
                #print(prev_loc1, prev_loc2, loc_1, loc_2)
                if loc_2 == prev_loc1 and loc_1 == prev_loc2:
                    #randomly choose agent and then build dynamic obstacle based on the path
                    #using disjoint splitting so can randomly choose aggent to build constraint for now instead of later
                    randvalue = random.randint(0,1)
                    if randvalue == 0:
                        return {'typeEdge': True, 'agent': 1, 'path': path1[t]}
                    else:
                        return {'typeEdge': True, 'agent': 2, 'path': path2[t]}
                else:
                    prev_loc1 = loc_1
                    prev_loc2 = loc_2
    
    # iterates through agents to find collisions
    def agent_get_first_collision(self, paths):
        result = []
        index = 0
        # print(paths)
        for i in range(len(paths)):
            for k in range(i+1, len(paths)):
                tmp = self.detect_collision(paths[i], paths[k])
                if tmp is None:
                    continue
                if tmp['typeEdge'] == True:
                    if tmp['agent'] == 1:
                        curr_agent = i
                    else:
                        curr_agent = k 
                    result = {'agent1': curr_agent, 'agent2': curr_agent, 'collision_loc': tmp['path'] }
                else:
                    result = {'agent1': i, 'agent2': k, 'collision_loc': tmp['path'] }
                
                #collision detected
                if result['collision_loc'] != None:
                    return result
        # no collision found
        return None
    
    #x,y,t (path) == dynamic obstacles
    def add_constraint(self, constraint):
        constraint_set = []
        for loc in constraint: # same loc diff ts
            #print("Add_constraint(): loc - ")
            #print(loc)
            constraint_set.append( {'x':loc['x'], 'y':loc['y'], 't':loc['t'] } )
        return constraint_set

    # disjoint splitting
    # used negative value for chosen agent to build positive constraint for specific agent (all other agents cannot be in location)
    # dynamic obstacle built for negative constraint agents if agent != agent * -1 (seen in line 239)
    # all agent values are +1 of original because of agent 0 edge case, 0 * -1 =0 and so if agent 0 is chosen no constraints ever built for other agents
    ## as such, all agent values are +1 of what they actually are and so checks need to be agent - 1 in order to find true agent value
    def create_constraints_from_conflict(self, conflict):
        value = random.randint(0,1)
        if value == 0:
            chosen_agent = conflict['agent1']+1
        else:
            chosen_agent = conflict['agent2']+1

        constraint_dict = []

        temp = []
        for i in range(-1, 2): #two sets of constraint
            if conflict['collision_loc']['t'] == 0:
                continue
            #choosen agent can't be in this loc at ts +/- 1 -> negative constraint
            temp.append({'agent': chosen_agent, 'x':conflict['collision_loc']['x'], 'y': conflict['collision_loc']['y'], 't': conflict['collision_loc']['t']+i}) 
        constraint_dict.append(temp)
        
        temp = []
        for i in range(-1, 2): #two sets of constraint
            if conflict['collision_loc']['t'] == 0:
                continue
            #applies to every other agent can't be in this loc at ts +/- 1 -> positive constraint i.e. dynamical obstacle for other agents
            temp.append({'agent': -1 * chosen_agent, 'x':conflict['collision_loc']['x'], 'y': conflict['collision_loc']['y'], 't': conflict['collision_loc']['t']+i}) 
        constraint_dict.append(temp)
        return constraint_dict

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution])

    def process_result(self, result):
        """" Converts dictionary path to tuple array path for all agents """

        presult = []

        for i in range(len(result)):
            li=[]
            for k in range(len(result[i])):
                li.append((result[i][k]['x'], result[i][k]['y']))
            presult.append(li)            
        return presult    

    def find_solution(self, disjoint=True):
        self.start_time = timer.time()
        start = HighLevelNode()
        e_nodes = 0
        g_nodes = 0
        
        start.constraint_dict = {}
        
        #low level search
        for agent in range(self.num_of_agents):
            start.constraint_dict[agent] = []
            sipp_planner = SafeIntervalPlanner(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], start.constraint_dict[agent])
            if sipp_planner.compute_plan():
                start.solution.append(sipp_planner.get_plan())
            else:
                raise BaseException('Path not found for individual agent')
        #print(start.solution)
        start.cost = self.compute_solution_cost(start.solution)             

        self.open_set |= {start}

        while self.open_set  and timer.time() - self.start_time < self.max_time:
            P = min(self.open_set)
            e_nodes += 1
            self.open_set -= {P}
            self.closed_set |= {P}

            conflict_dict = self.agent_get_first_collision(P.solution)

            #found solution
            if not conflict_dict:
                # print("solution found")
                # print("CBS g_nodes: ", g_nodes)
                # print("CBS e_nodes: ", e_nodes)
                result = self.process_result(self.generate_plan(P.solution))
                
                self.CPU_time = timer.time() - self.start_time
                return result

            tmp_constraint_dict = self.create_constraints_from_conflict(conflict_dict)

            #creating constraints
            for constraint in tmp_constraint_dict:
                #print("Cur Constraint: " )
                #print(constraint)
                emptyPathFound = False
                new_node = deepcopy(P)
                if constraint[0]['agent']-1 >= 0:
                    curr_agent = constraint[0]['agent']-1
                    new_node.constraint_dict[curr_agent].append(self.add_constraint(constraint))

                else:
                    tmp = self.add_constraint(constraint)
                    for agent in range(self.num_of_agents):
                        if agent == constraint[0]['agent']*-1-1:
                            continue
                        
                        curr_agent = agent
                        new_node.constraint_dict[agent].append(tmp)
                        
                for agent in range(self.num_of_agents):
                    sipp_planner = SafeIntervalPlanner(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], new_node.constraint_dict[agent])
                    # print("SIPP PLANNER_cp:")
                    # print(sipp_planner.compute_plan())
                    if sipp_planner.compute_plan():
                        new_node.solution[agent] = sipp_planner.get_plan()
                        #(sipp_planner.get_plan())
                        #print("New node solution")
                        #print(new_node.solution[agent])
                    else:
                        #print("Path failed for: ", agent) 
                        emptyPathFound = True 
                
                if emptyPathFound:
                    continue
                new_node.cost = self.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}
                    g_nodes += 1
                    #print("Added new node")
        #print("Solution NOT found")
        return None

    def generate_plan(self, solution):
        plan = []
        for agent_path in solution:
            path_dict_list=[]
            for loc in agent_path:
                path_dict_list.append( {'x':loc['x'], 'y':loc['y'], 't':loc['t']})
            plan.append(path_dict_list)

        return plan

    def print_results(self, node):
        pass
        #if DEBUG:
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))


# class CBSSolver(object): FAILED IMPLEMENTATION OF GENERIC INTERFACE + SIPP
#     """The high-level search of CBS."""

#     def __init__(self, my_map, starts, goals, planner, max_time=None):
#         """my_map   - list of lists specifying obstacle positions
#         starts      - [(x1, y1), (x2, y2), ...] list of start locations
#         goals       - [(x1, y1), (x2, y2), ...] list of goal locations
#         """

#         self.start_time = 0
#         self.my_map = my_map
#         self.starts = starts
#         self.goals = goals
#         self.num_of_agents = len(goals)

#         self.num_of_generated = 0
#         self.num_of_expanded = 0
#         self.CPU_time = 0
#         self.max_time =  max_time if max_time else float('inf')

#         self.open_list = []
#         self.cont = 0

#         # compute heuristics for the low-level search
#         self.heuristics = []
#         for goal in self.goals:
#             self.heuristics.append(compute_heuristics(my_map, goal))
        
#         # initialize low-level planner class (A* or SIPP)
#         self.path_planner = planner(self.my_map)
        
#     def push_node(self, node):
#         heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
#         if DEBUG:
#             print("Generate node {}".format(self.num_of_generated))
#         self.num_of_generated += 1

#     def pop_node(self):
#         _, _, id, node = heapq.heappop(self.open_list)
#         if DEBUG:
#             print("Expand node {}".format(id))
#         self.num_of_expanded += 1
#         return node

#     def find_solution(self, disjoint=True):
#         """ Finds paths for all agents from their start locations to their goal locations

#         disjoint    - use disjoint splitting or not
#         """
#         self.start_time = timer.time()

#         # Generate the root node
#         # constraints   - list of constraints
#         # paths         - list of paths, one for each agent
#         #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
#         # collisions     - list of collisions in paths
#         root = {
#             'cost': 0,
#             'constraints': [],
#             'paths': [],
#             'collisions': []
#         }
#         for i in range(self.num_of_agents):  # Find initial path for each agent
#             path = self.path_planner.get_path(self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
#             if path is None:
#                 raise BaseException('No solutions')
#             root['paths'].append(path)
        
#         root['cost'] = get_sum_of_cost(root['paths'])
#         root['collisions'] = detect_collisions(root['paths'])
#         self.push_node(root)
#         # Task 3.1: Testing
#         if DEBUG:
#             print(root['collisions'])

#         # Task 3.2: Testing
#         if DEBUG:
#             for collision in root['collisions']:
#                 print(standard_splitting(collision))

#         ##############################
#         # High-Level Search
#         # Repeat the following as long as the open list is not empty:
#         #   1. Get the next node from the open list (you can use self.pop_node())
#         #   2. If this node has no collision, return solution
#         #   3. Otherwise, choose the first collision and convert to a list of constraints (using your
#         #      standard_splitting function). Add a new child node to your open list for each constraint
#         # Ensure to create a copy of any objects that your child nodes might inherit

#         while self.open_list and timer.time() - self.start_time < self.max_time:
#             p = self.pop_node()
#             # if there are no collisions, we found a solution
#             if not p['collisions']:
#                 self.print_results(p)
#                 return p['paths']
#             # we choose a collision and turn it into constraints
#             collision = random.choice(p['collisions'])
#             # 4.2 Adjusting the High-Level Search
#             constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

#             # HERE
#             for c in constraints:
#                 skip_node = False
#                 q = {'cost': 0,
#                      'constraints': [*p['constraints'], c],  # all constraints in p plus c
#                      'paths': p['paths'].copy(),
#                      'collisions': []
#                      }
#                 agent = c['agent']
#                 #print(q['constraints'])
#                 path = self.path_planner.get_path(self.starts[agent], self.goals[agent], self.heuristics[agent], agent, q['constraints'])
#                 if path:
#                     q['paths'][agent] = path
#                     if c['positive']:
#                         rebuild_agents = paths_violate_constraint(c, q['paths'])
#                         for r_agent in rebuild_agents:
#                             c_new = c.copy()
#                             c_new['agent'] = r_agent
#                             c_new['positive'] = False
#                             q['constraints'].append(c_new)
#                             r_path = self.path_planner.get_path(self.starts[r_agent], self.goals[r_agent], self.heuristics[r_agent], r_agent, q['constraints'])
#                             if r_path is None:
#                                 skip_node = True
#                                 break # at least one agents has none solution
#                             else:
#                                 q['paths'][r_agent] = r_path
#                     if(not skip_node):
#                         q['collisions'] = detect_collisions(q['paths'])
#                         q['cost'] = get_sum_of_cost(q['paths'])
#                         self.push_node(q)
#                 else:
#                     raise BaseException('No solutions')
#         raise BaseException('Time limit exceeded')

#     def print_results(self, node):
#         pass
#         #if DEBUG:
#         #print("\n Found a solution! \n")
#         #CPU_time = timer.time() - self.start_time
#         #print("CPU time (s):    {:.2f}".format(CPU_time))
#         #print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
#         #print("Expanded nodes:  {}".format(self.num_of_expanded))
#         #print("Generated nodes: {}".format(self.num_of_generated))