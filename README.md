# Robotics
This assignment focuses on different methods of path finding.  The environment for this assignment was provided.  This includes start_goal.txt, world_obstacles.txt and map generation within the script.

prm.py implements and visualizes a probabilistic roadmap along with the shortest path.  This algorithm generates nodes randomly throughout the environment, then creates connections between nodes, while avoiding obstacles, to find the shortest path.

rrt.py implements and visualizes rapidly exploring random tree along with the sortest path.  Unlike the PRM, this algorithm incrementally generates points close to the current node randomly and connects to the node closest to the direction of the goal state, while avoiding obstacles.

bidirect_rrt.py implements and visualizes a bidirectional rapidly exploring random tree.  Similar implementation to rrt.py except the RRT spans from both the start and goal nodes, and also provides a connect function for when the two trees are close enough to complete the path.
