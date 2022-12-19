import heapq
from math import fabs
from graph_generation import SIPPGraph, State

"""
A* single agent planner implementation  

author(s):  github.com/nicofretti,      William Horvath (github.com/bolded)
            repo: nicofretti/MAPF       repo: lla105/417Project
"""

class AStarPlanner(object):
    def __init__(self, my_map):
        """ 
        my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
        """
        
        self.my_map = my_map

        #self.CPU_time = 0

        # compute heuristics for the low-level search
        # self.heuristics = []
        # for goal in self.goals:
        #     self.heuristics.append(compute_heuristics(my_map, goal))


    def build_constraint_table(self, constraints, agent):
        ##############################
        # Return a table that constains the list of constraints of
        # the given agent for each time step. The table can be used
        # for a more efficient constraint violation check in the 
        # is_constrained function.
        c_table = dict()
        for c in constraints:
            # we need to consider only the constraints for the given agent
            # 4.1 Supporting positive constraints
            if (not 'positive' in c.keys()):
                c['positive'] = False
            if c['agent'] == agent:
                timestep = c['timestep']
                if timestep not in c_table:
                    c_table[timestep] = [c]
                else:
                    c_table[timestep].append(c)
        return c_table


    def get_plan(self, goal_node):
        path = []
        curr = goal_node
        while curr is not None:
            path.append(curr['loc'])
            curr = curr['parent']
        path.reverse()
        return path


    def flatten_constraints(self, list_of_constraints_list):
        constraints = []
        for constr_list in list_of_constraints_list:
            for c in constr_list:
                constraints.append(c)
        return constraints


    def is_constrained(self, curr_loc, next_loc, next_time, constraint_table):
        ##############################
        # Check if a move from curr_loc to next_loc at time step next_time violates
        # any given constraint. For efficiency the constraints are indexed in a constraint_table
        # by time step, see build_constraint_table.
        if next_time in constraint_table:
            constraints = constraint_table[next_time]
            for c in constraints:
                if [next_loc] == c['loc'] or [curr_loc, next_loc] == c['loc']:
                    return True
        else:
            constraints = [c for t, c in constraint_table.items() if t < next_time]
            constraints = self.flatten_constraints(constraints)
            for c in constraints:
                if [next_loc] == c['loc'] and c['final']:
                    return True
        return False


    def is_goal_constrained(self, goal_loc, timestep, constraint_table):
        """
        checks if there's a constraint on the goal in the future.
        goal_loc            - goal location
        timestep            - current timestep
        constraint_table    - generated constraint table for current agent
        """
        constraints = [c for t, c in constraint_table.items() if t > timestep]
        constraints = self.flatten_constraints(constraints)
        for c in constraints:
            if [goal_loc] == c['loc']:
                return True
        return False


    def push_node(self, open_list, node):
        heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


    def pop_node(self, open_list):
        _, _, _, curr = heapq.heappop(open_list)
        return curr


    def compare_nodes(self, n1, n2):
        """Return true is n1 is better than n2."""
        return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


    def a_star(self, start_loc, goal_loc, h_values, agent, constraints):
        """ 
        my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
        """

        open_list = []
        closed_list = dict()
        h_value = h_values[start_loc]
        c_table = self.build_constraint_table(constraints, agent)
        root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time': 0}
        self.push_node(open_list, root)
        closed_list[(start_loc, 0)] = root
        max_map_width = max([len(e) for e in self.my_map])
        while len(open_list) > 0:
            curr = self.pop_node(open_list)

            if curr['loc'] == goal_loc and not self.is_goal_constrained(goal_loc, curr['time'], c_table):
                return self.get_plan(curr)
            for direction in range(5):
                # directions 0-3: the agent is moving, direction 4: the agent is still
                if direction < 4:
                    child_loc = move(curr['loc'], direction)
                    # check if the child location is outsite the map or the agent go against an obstacle
                    if child_loc[0] < 0 or child_loc[1] < 0 or \
                            child_loc[0] >= len(self.my_map) or child_loc[1] >= max_map_width or \
                            self.my_map[child_loc[0]][child_loc[1]]:
                        continue
                    child = {'loc': child_loc,
                            'g_val': curr['g_val'] + 1,
                            'h_val': h_values[child_loc],
                            'parent': curr,
                            'time': curr['time'] + 1}
                else:
                    # the agent remains still
                    child = {'loc': curr['loc'],
                            'g_val': curr['g_val'] + 1,  # remaining in the same cell has a cost
                            'h_val': curr['h_val'],
                            'parent': curr,
                            'time': curr['time'] + 1}
                # check if the child violates the constraints
                if self.is_constrained(curr['loc'], child['loc'], child['time'], c_table):
                    continue
                if (child['loc'], child['time']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['time'])]
                    if self.compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['time'])] = child
                        self.push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['time'])] = child
                    self.push_node(open_list, child)

        return None  # Failed to find solutions
    
    def get_path(self, start_loc, goal_loc, h_values, agent, constraints):
        return self.a_star(start_loc, goal_loc, h_values, agent, constraints)


# '''
# Helper functions for the rest of the codebase
# '''

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def move(loc, direction):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[direction][0], loc[1] + directions[direction][1]

def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for direction in range(4):
            child_loc = move(loc, direction)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

"""
SIPP single agent planner implementation  

author(s):  Ashwin Bose (github.com/atb033),            github.com/WheelChan,                       
            repo: atb033/multi_agent_path_planning      repo: WheelChan/Multi-Agent-Path-Finding    

            William Horvath (github.com/bolded)
            repo: lla105/417Project

SIPP article: https://doi.org/10.1109/ICRA.2011.5980306
"""
class SafeIntervalPlanner(SIPPGraph):
    def __init__(self, my_map, starts, goals, h_values, dyn_obstacles):
        SIPPGraph.__init__(self, my_map, dyn_obstacles)
        
        self.start = starts
        self.goal = goals
        self.hvalue = h_values
        
        self.open = []
        self.max_path = 0

    def get_successors(self, state):
        successors = []
        m_time = 1
        neighbour_list = self.get_valid_neighbours(state.position)

        for neighbour in neighbour_list:
            start_t = state.time + m_time
            end_t = state.interval[1] + m_time
            for i in self.sipp_graph[neighbour].interval_list:
                if i[0] > end_t or i[1] < start_t:
                    continue
                time = max(start_t, i[0]) 
                s = State(neighbour, time, i)
                successors.append(s)
        return successors

    def get_heuristic(self, position):
        return fabs(position[0] - self.goal[0]) + fabs(position[1]-self.goal[1])

    def get_first(self, element):
        return element[0]

    #compute_plan + get_plan = A-star function
    #compute_plan() determines if plan exists for specific problem and if yes, get_plan() retrieves said plan
    def compute_plan(self):
        self.open = []
        goal_reached = False
        cost = 1
        e_nodes = 0
        g_nodes = 0

        s_start = State(self.start, 0) 

        self.sipp_graph[self.start].g = 0.
        f_start = self.get_heuristic(self.start)
        self.sipp_graph[self.start].f = f_start

        self.open.append((f_start, s_start))

        #following algorithm described in original paper (fig 4)
        while (not goal_reached):
            # print("OPEN LIST:")
            # print(self.open)
            if self.open == []: 
                # Plan not found
                # print("Expected to this msg for no plans found")
                return 0
            s = self.open.pop(0)[1]
            e_nodes += 1
            successors = self.get_successors(s)

            for successor in successors:
                #print("Successor pos:", successor.position)
                #print("successor g(): ", self.sipp_graph[successor.position].g)
                #print("parent g(): ", self.sipp_graph[s.position].g + cost)
                
                if self.sipp_graph[successor.position].g > self.sipp_graph[s.position].g + cost:

                    self.sipp_graph[successor.position].g = self.sipp_graph[s.position].g + cost
                    self.sipp_graph[successor.position].parent_state = s

                    #print("Successor time: ", successor.time)
                    #print("Max path: ", self.max_path)
                    if successor.position == self.goal and successor.time > self.max_path:
                        # print("Plan successfully calculated!!")
                        goal_reached = True
                        break

                    self.sipp_graph[successor.position].f = self.sipp_graph[successor.position].g + self.get_heuristic(successor.position)
                    self.open.append((self.sipp_graph[successor.position].f, successor))
                    g_nodes += 1
            
            self.open.sort(key = self.get_first)
            

        # Tracking back
        start_reached = False
        self.plan = []
        current = successor
        while not start_reached:
            self.plan.insert(0,current)
            if current.position == self.start:
                start_reached = True
            current = self.sipp_graph[current.position].parent_state
        # print("Expanded Nodes: ", e_nodes)
        # print("Generated Nodes: ", g_nodes)
        return 1
            
    def get_plan(self):
        path_list = []

        # first setpoint
        setpoint = self.plan[0]
        temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
        path_list.append(temp_dict)

        for i in range(len(self.plan)-1):
            for j in range(self.plan[i+1].time - self.plan[i].time-1):
                x = self.plan[i].position[0]
                y = self.plan[i].position[1]
                t = self.plan[i].time
                setpoint = self.plan[i]
                temp_dict = {"x":x, "y":y, "t":t+j+1}
                path_list.append(temp_dict)

            setpoint = self.plan[i+1]
            temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
            path_list.append(temp_dict)

        #print("GET_PLAN")
        #print(path_list)
        return path_list

