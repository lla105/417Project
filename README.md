# CMPT 417 Project
# Group members: 
> William Horvath, Trung Hieu Le, Leon Lee

## Introduction:
> We propose for our group project to implement and test the Safe Interval Path Planning (SIPP) search algorithm in Python, in the context of the multi-agent path finding (MAPF) algorithms Cooperative A* and Conflict-Based Search (CBS). We aim to combine, refine, and extend our individual projects on MAPF by modularizing the single-agent planner component to work with both A* (original) and SIPP (our new implementation) algorithms. After implementing SIPP, we would like to test/benchmark the resultant program under the four combinations of (A*, SIPP) x (Coop-A*, CBS) on a battery of test instances, which would include instances given in the individual assignment and instances that we have crafted.
> We hope to answer questions about which combination of planner and multi-agent path finder gives the fastest result in different classes of instances (and if there even are significant differences between the classes’ results), and also to compare the amount of nodes expanded and memory used by each one. Some of these instance classes which we can use to categorize results are # of agents, crowded vs sparse, and/or number of obstacles in the map. We would also like to discover whether suboptimal solutions occur with A* and not SIPP, and vice versa. 

## To-Do List:
1. ~~Setup working environment, with basic MAPF working(with space_time A* & CBS set up)~~
2. ~~Get Coop A* to run as Multi agent path finder.~~
3. ~~Get CBS to run as multi agent path finder.~~
4. Get basic A* to run as single agent planner.
5. Turn pseudo SIPP into python SIPP, get it to run as a single agent planner.
6. A* with Cooperative A*
7. A* with CBS
8. SIPP with Cooperative A*
9. SIPP with CBS
10. Yield result. With # of nodes, time taken, memories used, h(n) used, # of agents, obstacles in mind, write report to attempt to explain the differences in these results.


## Findings:
### Prioritized Planning Space Time A* (Left) VS SIPP (Right)
https://user-images.githubusercontent.com/105393685/206966008-a897e7b0-b996-4930-8160-699ef9a8cf12.mov

> Case 1: Given same map, same number of agent, same obstacle, same goal, the path taken is exactly the same.

https://user-images.githubusercontent.com/105393685/206967300-4673e499-7e6b-49e0-8287-7ed31c311029.mov

> Case 2: Given same map, same # of agents, same obstacles, same goal, the path taken are different. Prioritized Planning Space Time A* moves Agent 1 out of the way for Agent 0, while SIPP just lets Agent 1 sit still, resulting in a collision.


