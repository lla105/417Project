import time as timer
import heapq
import random
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    path1_length = len(path1)
    path2_length = len(path2)
    longerpath_length = max(path1_length, path2_length)
    # print("HELLOO?????")
    for t in range(longerpath_length):
        # print(t)
        # print("TYPE IS ::::::::::" + str(type(get_location(path2,t-1))))
        # location21 = get_location(path2,t-1)
        # if location21 == get_location(path2,t-1):
        #     print('YESSSS')
        if t > 0 and get_location(path1,t-1) == get_location(path2,t) and get_location(path1,t) == get_location(path2,t-1):
            location21 = get_location(path2,t-1)
            location22 = get_location(path2,t)
            # return {'loc':(location21, location22), 'timestep':t} #TASK3.1
            return {'loc':[(location21, location22)], 'timestep':t} #TASK3.2
            
        elif get_location(path1,t) == get_location(path2,t):
            location11 = get_location(path1,t) 
            # return {'loc':location11, 'timestep':t} #TASK3.1
            return {'loc':[location11], 'timestep':t} #TASK3.2
    return {}

    return 'wot'



def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    # print(paths)
    path_length = len(paths)-1
    for i in range(path_length):
        path2length = len(paths)
        for j in range(i+1, path2length):
            collision = detect_collision(paths[i],paths[j])
            if collision == {}:
                print("no collision!!")
            else:
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    # print(len(collision['loc']))
    # print(collision['loc'])
    temploc = collision['loc']
    print("standard_splitting()")
    print(temploc)
    # print(collision['loc'][1] + collision['loc'][0])
    
    if len(collision['loc']) != 1:
        # print(collision['loc'][0])
        # print(collision['loc'][1])
        # print("end==============")
        # constraint1 = {'agent': collision['a1'],'loc': temploc,'timestep': collision['timestep']} #TASK3.2    
        # constraint2 = {'agent': collision['a2'],'loc': [collision['loc'][1], collision['loc'][0]],'timestep': collision['timestep']}#TASK3.2
        constraint1 = {'agent': collision['a2'],'loc': temploc,'timestep': collision['timestep']}  #TASK3.3     
        constraint2 = {'agent': collision['a1'],'loc': [collision['loc'][1], collision['loc'][0]],'timestep': collision['timestep']} #TASK3.3

    else:
        constraint1 = {'agent': collision['a1'],'loc': temploc,'timestep': collision['timestep']}
        constraint2 = {'agent': collision['a2'],'loc': temploc,'timestep': collision['timestep']}
    return [constraint1,constraint2]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [{'agent': 2,
                'loc': [(3,4)],
                'timestep': 5}],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(f"agent: {agent}")
        # print(f"root['collisions']: {root['collisions'] } ")
        # print(f"root['collisions']: {(root['collisions'])[0]['loc'] } ")

        # tempa1 = int(root['collisions'][0]['a1']) #TASK3.1
        # tempa2 = int(root['collisions'][0]['a2']) #TASK3.1
        # temploc = root['collisions'][0]['loc'] #TASK3.1
        # temptimestep = (root['collisions'][0]['timestep']) #TASK3.1
        # print("[{'a1': "+str(tempa1)+", 'a2': "+str(tempa2)+", 'loc': " #TASK3.1
        #     +str(temploc)+", 'timestep': "+str(temptimestep)+"}]")

        # print(f"root'paths']!!!!!!! {root['paths']} ")
        # print(f"detect_collisions?????" {detect_collisions(root['paths'])} )

        # Task 3.2: Testing
        # for collision in root['collisions']:
        #     # print("!!!!!!!!!!")
        #     print(standard_splitting(collision))


        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # self.print_results(root) #TASK3.2 and before
        # return root['paths'] #TASK3.2 and before

        while len(self.open_list) > 0: #6 while OPEN is not empty do
            # print("++++++++++++++++++++++++++++++")
            curr = self.pop_node()#7 P ← node from OPEN with the smallest cost
            # print(f"!!!!!{curr}")
            # print(f">>>>>>>>>>>>>>>>> {curr['collisions' ]} ")
            temp_collision_list = curr['collisions']#
            # print(temp_collision_list)
            temp2list = curr['collisions']#
            templength = len(temp_collision_list)#
            length2 = len(curr['collisions'])#
            if length2 == len(curr["collisions"]):#
                print("")
                pass
            if templength != 0: # 8 if P.collisions = ∅ then
                pass
            else: #8 if P.collisions = ∅ then
                # print(f"THIS IS curr['paths']: {curr['paths']})
                return curr["paths"]#9 return P.paths // P is a goal node
            nextnode = {}#10 collision ← one collision in P.collisions
            temp3list = curr['collisions']#
            templist3 = curr['paths']#
            if templist3 == curr['paths']:#
                # print("SAMEEEE")
                pass
            real_constraints = standard_splitting(temp3list[0])#11 constraints ← standard_splitting(collision)
            for eachconstraint in real_constraints:#12 for constraint in constraints do
                child={}#13 Q ← new node
                nextnode['paths'] = copy.deepcopy(curr['paths'])#15 Q.paths ← P.paths
                child['constraints']= copy.deepcopy(curr['constraints'])#14 Q.constraints ← P.constraints ∪ {constraint}
                nextnode['paths'].append(eachconstraint)#15 Q.paths ← P.paths
                nextnode['constraints']=copy.deepcopy(curr['constraints'])#14 Q.constraints ← P.constraints ∪ {constraint}
                child['constraints'].append(eachconstraint)#
                child['paths']= copy.deepcopy(curr['paths'])#15 Q.paths ← P.paths
                if nextnode == child:#
                    # print(f"{nextnode} == {child}")
                    pass
                agentnum = eachconstraint['agent']#16 ai ← the agent in constraint
                path = a_star(self.my_map, #17 path ← a_star(ai, Q.constraints)
                                self.starts[agentnum], 
                                self.goals[agentnum], 
                                self.heuristics[agentnum],
                                agentnum, 
                                child['constraints'])
                if path is None: #18 if path is not empty then
                    pass
                else:#18 if path is not empty then
                    child['paths'][agentnum] = path#19 Replace the path of agent ai in Q.paths by path
                    child['collisions'] = detect_collisions(child['paths'])#20 Q.collisions ← detect_collisions(Q.paths)
                    child['cost'] = get_sum_of_cost(child['paths'])#21 Q.cost ← get_sum_of_cost(Q.paths)
                    self.push_node(child)#22 Insert Q into OPEN
#

        raise BaseException('No solutions')#23 return'No solutions'

        

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
