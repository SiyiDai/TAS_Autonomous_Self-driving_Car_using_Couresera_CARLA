### Global Planner Module

As the waypoints we got from map do not have a graph-structure, but are only discreted points, we used NearestNeighbors from sklearn package of python to create connection among the nearest points. Every waypoint was set as a node in a Networkx graph and for edges we considered the connections between the nearest neighbours. That is how we created node graph from discreted waypoints distribution. 

After that, we applied A* algorithm on the node graph we extracted from the waypoints and got the shortest path from starting node to goal node. For heuristic function we used the distance from the current node to goal node. For edge weight we considered euclidean distance between two neighbours. 

To see how this works you can have a look at `shortest_path_test.py`, where `positions.txt` contains all the waypoints we extracted from the map. 
