import time as timer
from single_agent_planner import compute_heuristics, AStarPlanner, SafeIntervalPlanner, get_sum_of_cost

"""
A planner that plans for each robot independently.

author(s):  github.com/nicofretti,      Ashwin Bose (github.com/atb033),            William Horvath (github.com/bolded)
            repo: nicofretti/MAPF       repo: atb033/multi_agent_path_planning      repo: lla105/417Project
                                                  
"""

class IndependentSolver(object):
    def __init__(self, my_map, starts, goals, planner):
        """
        my_map      - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        planner     - which low-level path planner to use (A* or SIPP)
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
            
        self.planner = planner

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        
        if self.planner == "Astar":
            self.planner = AStarPlanner(self.my_map)
            for i in range(self.num_of_agents):  # Find path for each agent
                
                
                path = self.planner.get_path(self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
                if path is None:
                    raise BaseException('No solutions')
                result.append(path)
        
        else: #planner == SIPP
            for i in range(self.num_of_agents):  # Find path for each agent
                
                sipp_planner = SafeIntervalPlanner(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], constraints)
                
                if result != []:
                        sipp_planner.max_path = len(result[0])

                if sipp_planner.compute_plan():               
                    plan = sipp_planner.get_plan()
                    #print("!!!! PLAN: ")
                    #print(plan)
                    result.append(plan)
                    #print(result)
                    #TODO add a time constraint for agent to be at goal location at designated time
                    
                #else:
                    #raise BaseException('No solutions')

        ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        return result
