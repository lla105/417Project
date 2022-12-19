"""
Graph generation for SIPP 

author(s):  Ashwin Bose (github.com/atb033),            William Horvath (github.com/bolded)
            repo: atb033/multi_agent_path_planning      repo: lla105/417Project       

SIPP article: https://doi.org/10.1109/ICRA.2011.5980306
"""

from bisect import bisect

class State(object):
    def __init__(self, position=(-1,-1), t=0, interval=(0,float('inf'))):
        self.position = tuple(position)
        self.time = t
        self.interval = interval

class SIPPGrid(object):
    def __init__(self):
        # self.position = ()
        self.interval_list = [(0, float('inf'))]
        self.f = float('inf')
        self.g = float('inf')
        self.parent_state = State()

    def split_interval(self, t, last_t = False):
        """
        Function to generate safe-intervals
        """
        for interval in self.interval_list:
            if last_t:
                if t<=interval[0]:
                    self.interval_list.remove(interval)
                elif t>interval[1]:
                    continue
                else:
                    self.interval_list.remove(interval)
                    self.interval_list.append((interval[0], t-1))
            else:
                if t == interval[0]:
                    self.interval_list.remove(interval)
                    if t+1 <= interval[1]:
                        self.interval_list.append((t+1, interval[1]))
                elif t == interval[1]:
                    self.interval_list.remove(interval)
                    if t-1 <= interval[0]:
                        self.interval_list.append((interval[0],t-1))
                elif bisect(interval,t) == 1:
                    self.interval_list.remove(interval)
                    self.interval_list.append((interval[0], t-1))
                    self.interval_list.append((t+1, interval[1]))
            self.interval_list.sort()

class SIPPGraph(object):
    def __init__(self, map, dynamic_obstacles):
        self.map = map
        # print("map:")
        # print(self.map)
        
        self.dimensions = (len(self.map[0]), len(self.map))
        
        self.obstacles = []
        for x in range(len(self.map)):
            for y in range(len(self.map[0])):
                if self.map[x][y]:
                    self.obstacles.append((x, y))
        
        #need to update
        self.dyn_obstacles = []
        if bool(dynamic_obstacles):
            for obs in dynamic_obstacles:
                self.dyn_obstacles.append(obs)

        self.sipp_graph = {}
        self.init_graph()
        self.init_intervals()

    def init_graph(self):
        for i in range(self.dimensions[0]):
            for j in range(self.dimensions[1]):
                grid_dict = {(i,j):SIPPGrid()}
                self.sipp_graph.update(grid_dict)
    
    def init_intervals(self):
        if not self.dyn_obstacles: return
        for schedule in self.dyn_obstacles:
            for i in range(len(schedule)):
                location = schedule[i]
                last_t = i == len(schedule)-1

                position = (location["x"],location["y"])
                t = location["t"]

                self.sipp_graph[position].split_interval(t, last_t)
                # print(str(position) + str(self.sipp_graph[position].interval_list))     
    
    def is_valid_position(self, position):
        dim_check = position[0] in range(self.dimensions[0]) and  position[1] in range(self.dimensions[1])
        obs_check = position not in self.obstacles
        # print(dim_check)
        return dim_check and obs_check

    #determines positions of possible next nodes for intervals
    def get_valid_neighbours(self, position):
        neighbour_list = []

        up = (position[0], position[1]+1)
        if self.is_valid_position(up): neighbour_list.append(up)

        down = (position[0], position[1]-1)
        if self.is_valid_position(down): neighbour_list.append(down)

        left = (position[0]-1, position[1])
        if self.is_valid_position(left): neighbour_list.append(left)

        right = (position[0]+1, position[1])
        if self.is_valid_position(right): neighbour_list.append(right)
        
        return neighbour_list

