#!/usr/bin/env python3



from __future__ import print_function
from __future__ import division


# System level imports
import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from collections import defaultdict
import networkx as nx



def readfile(filename):
    f = open(filename)
    lines = f.readlines()
    data = np.zeros((len(lines),2))
    tempo = []
    #data = lines
    for i in range(len(lines)):
        tempo.append(eval(lines[i]))#.strip('\n')))
        data[i,0] = tempo[i][0]
        data[i,1] = tempo[i][1]
    #print(tempo)
    return data



def created_edges(indices):
    edges1 = []
    for i in range(len(indices)):
        a = str(indices[i,0])
        b = str(indices[i,1])
        c = str(indices[i,2])
        d = str(indices[i,3])
        e = str(indices[i,4])
        edges1.append([a,b])
        edges1.append([a,c])
        edges1.append([a,d])
        edges1.append([a,e])
    return edges1


# Function to build the graph
def build_graph(edges):

    graph = defaultdict(list)

    # Loop to iterate over every
    # edge of the graph

    for edge in edges:
        a, b = edge[0], edge[1]

        # Creating the graph
        # as adjacency list
        graph[a].append(b)
        graph[b].append(a)
    return graph

"""BFS functions"""
# finds shortest path between 2 nodes of a graph using BFS
def bfs_shortest_path(graph, start, goal):
    # keep track of explored nodes
    explored = []
    # keep track of all the paths to be checked
    queue = [[start]]
    # return path if start is goal
    if start == goal:
        return "Easy! Start = goal"
    # keeps looping until all possible paths have been checked
    while queue:
        # pop the first path from the queue
        path = queue.pop(0)
        # get the last node from the path
        node = path[-1]
        if node not in explored:
            neighbours = graph[node]
            # go through all neighbour nodes, construct a new path and
            # push it into the queue
            for neighbour in neighbours:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                # return path if neighbour is goal
                if neighbour == goal:
                    return new_path
            # mark node as explored
            explored.append(node)
    # in case there's no path between the 2 nodes
    return "Sorry, cannot find connecting path :("

# distance between two points (input: indices)
def dist(start_point, goal_point):
    start_coor = X[int(start_point)]
    goal_coor = X[int(goal_point)]
    (x1, y1) = start_coor
    (x2, y2) = goal_coor
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def plot_path(astar_path):
    try:
        x = []
        y = []
        for i in range(len(astar_path)):
            x.append(X[int(astar_path[i]), 0])
            y.append(X[int(astar_path[i]), 1])
        plt.plot(x, y)
        plt.annotate('start', xy=(x[1], y[1]), xytext=(x[0], y[0]), arrowprops=dict(arrowstyle="->", connectionstyle="arc3"))
        plt.show()
    except IndexError:
        print('A_star: Easy! Start = goal')

# Driver Code

if __name__ == "__main__":
    # read waypoints
    map_data = readfile('positions.txt')
    # create connections between nearest neighbours
    X = np.array(map_data)
    nbrs = NearestNeighbors(n_neighbors=5, algorithm='ball_tree', p=2).fit(X)
    distances, indices = nbrs.kneighbors(X)
    # create edges
    edges1 = created_edges(indices)
    # build graph
    graph = build_graph(edges1)
    # Give the indices of start and goal points
    start_point = '1'
    goal_point = '55'

    '''# BFS Function Call
    bfs_path = bfs_shortest_path(graph, start_point, goal_point)
    print("BFS path: ", bfs_path)'''

    # build Networkx graph
    G = nx.Graph()
    for i in range(len(indices)):
        G.add_node(str(i), pos=(X[i,0], X[i,1]))
    pos = nx.get_node_attributes(G, 'pos')
    #G.add_nodes_from(indices[:,0])
    plt.figure(1)
    # add weighted edges
    for i in range(len(edges1)):
        G.add_edge(edges1[i][0],edges1[i][1], weight=dist(edges1[i][0], edges1[i][1]))
    # draw network, later plot together with A* path
    nx.draw(G, pos, with_labels=True)
    #plt.show()

    # astar_path = nx.astar_path(G, start_point, goal_point, heuristic=dist, weight="cost")
    astar_path = nx.astar_path(G, start_point, goal_point, heuristic=dist)
    print('A* Path: ', astar_path)
    # create new graph for A* path, draw
    G1 = nx.Graph()
    for i in range(len(astar_path) - 1):
        G1.add_edge(astar_path[i],astar_path[i+1])
    nx.draw_networkx_edges(G1, pos, edge_color='r', width=10)

    plt.figure(2)
    plot_path(astar_path)
    plt.show()

























