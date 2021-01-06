#!/usr/bin/env python
# usage:  python prm.py world_obstacles.txt start_goal.txt
from __future__ import division
from shapely import geometry
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from collections import defaultdict
import numpy as np
import random, math
import networkx as nx
 
class Point:
    x = 0.0
    y = 0.0

    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    node_id = None
    point = None
    
    def __init__(self, x, y, node_id):
        self.point = Point(x,y)
        self.node_id = node_id
        self.neighbors = []
        self.vertices = {}

    def add_edge(self,p2,cost):
        self.vertices = {(self.point,p2):cost}

    def get_edge(self,p2):
        return self.get()


class prm:

    nodes = []
    edges = []

    def __init__(self, poly, numNodes):
      self.poly = poly
      self.numNodes = numNodes
      self.edges = []
      self.graph = nx.Graph()
      self.nodes = []
   
    def create_graph(self,start,end,obstacles):
        total = 0
        start = Node(start[0],start[1],8888)
        self.nodes.append(start)
        self.graph.add_node(start)
        while total < self.numNodes:
            p = Node(np.random.choice(600),np.random.choice(600),total+2)

            collision = False
            check = geometry.Point(p.point.x,p.point.y)
            for poly in self.poly:
                if check.within(poly):
                    collision = True
            if collision == False: 
                self.graph.add_node(p)
                self.nodes.append(p)
                total+=1
        end = Node(end[0],end[1],99999)
        self.graph.add_node(end)
        self.nodes.append(end)

    def neighbors(self, obstacles):
        collision = False
        c = 0
        for i in self.nodes:
            distances = []
            for j in self.nodes:
                p1 = geometry.Point(i.point.x,i.point.y)
                p2 = geometry.Point(j.point.x,j.point.y)
                try:
                    line = geometry.LineString([p1,p2])
                except Exception as e:
                    print(e)
                    break
                for k in self.poly:
                    if k.intersects(line):
                        collision = True
                if collision == False:
                    distances.append((self.distance(i.point,j.point),j))
                collision = False

            distances = sorted(distances, key=lambda x: x[0])
            count = 0
            past = False
            for pair in distances:
                if count >= 10:
                    past = True
                if past == False:
                    i.neighbors.append(pair[1]);
                    dist = self.distance(i.point,pair[1].point)
                    self.graph.add_edge(i,pair[1],length=dist)
                else:
                    dist = self.distance(i.point,pair[1].point)
                    if dist <= 100:
                        i.neighbors.append(pair[1]);

                count+=1
                
    def distance(self,p1, p2):
        return math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y), 2))

    def get_node(self,node_id):
        for i in self.nodes:
            if i.node_id == node_id:
                return i

    def enhance(self, obstacles):
        for node in self.nodes:
            if len(node.neighbors) > 10:
                for i in range(11,len(node.neighbors)):
                    collision = False
                    check = node.neighbors[i]
                    p1 = geometry.Point(check.point.x,check.point.y)
                    p2 = geometry.Point(node.point.x,node.point.y)
                    line = geometry.LineString([p1,p2])
                    dist = self.distance(node.point,check.point)
                    for k in self.poly:
                        if k.intersects(line):
                            collision = True
                    self.graph.add_edge(node,check,length=dist)



def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='violet')

    ax.add_patch(pathpatch)
    ax.set_title('Sample-Based Motion Planning')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path,vertices

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='green'))
    ax.add_patch(patches.Circle(goal, facecolor='fuchsia'))

    return start, goal

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path,vertices = build_obstacle_course(args.obstacle_path, ax)
    start = 0
    polys = []
    temp = []
    for i in vertices:
        if i[0] == 0 and i[1] == 0:
            if start == 0:
                start += 1
                continue
            elif start == 1:
                poly = geometry.Polygon([[p[0], p[1]] for p in temp])
                polys.append(poly)
                temp = []
        else:
            temp.append(i)

    start, goal = add_start_and_goal(args.start_goal_path, ax)

    roadmap = prm(polys,200)
    roadmap.create_graph(start,goal,vertices)
    roadmap.neighbors(vertices)
    roadmap.enhance(polys)

    s = roadmap.get_node(8888)
    g = roadmap.get_node(99999)

    shortest_path = nx.shortest_path(roadmap.graph,s,g,weight='length')
    x = []
    y = []
    for i in shortest_path:
        x.append(i.point.x)
        y.append(i.point.y)
    plt.plot(x,y)
    plt.show()
