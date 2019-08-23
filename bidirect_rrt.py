# usage:  python bidirect_rrt.py world_obstacles.txt start_goal.txt

from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib import animation
import numpy as np
import random, math
from math import sqrt,cos,sin,atan2
from shapely import geometry
import networkx as nx

class Point:
    x = 0.0
    y = 0.0

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def print(self):
        print("(" + str(self.x) + ", " + str(self.y) + ")")

    def print(self,p2):
        print("[(" + str(self.x) + ", " + str(self.y) + "), (" + str(p2.x) + ", " + str(p2.y) + ")]")

def collision_check(n,obstacles):
    check = geometry.Point(n.x,n.y)
    for poly in obstacles:
        if check.within(poly):
            return True
    return False

def get_qrand(obstacles):
    collision = True
    while collision == True:
        x = np.random.choice(600)
        y = np.random.choice(600)
        q_rand = Point(x,y)
        collision = collision_check(q_rand,obstacles)
    return q_rand

def get_qnear(q_rand,G):
    min_dist = 9999999
    closest = None
    for n in G.nodes:
        # n.print()
        dist = distance(q_rand,n)
        if dist < min_dist:
            closest = n
            min_dist = dist
    return closest,min_dist

def t1(start_point,first,delta,S,obstacles):
    q_rand = get_qrand(obstacles)
    if first == True:
        q_near = start_point
        dist = distance(q_near,q_rand)
    else:
        q_near,dist = get_qnear(q_rand,S)

    q_new = step_from_to(q_near,q_rand,delta)
    
    if q_new != q_near:
        p1 = geometry.Point(q_new.x,q_new.y)
        p2 = geometry.Point(q_near.x,q_near.y)
        line = geometry.LineString([p1,p2])
        for p in obstacles:
            if p.intersects(line):
                return True
        S.add_node(q_new)
        S.add_edge(q_near,q_new,length=dist)
    collision = False
    #grow goal point to find a solution
    d = distance(q_new,goal_point)
    if d <= 3:
        p1 = geometry.Point(q_new.x,q_new.y)
        p2 = geometry.Point(q_near.x,q_near.y)
        line = geometry.LineString([p1,p2])
        for p in obstacles:
            if p.intersects(line):
                return True

        S.add_node(q_new)
        S.add_edge(q_near,q_new,length=d)
        return

def t2(start_point,first,delta,G,obstacles):
    q_rand = get_qrand(obstacles)
    if first == True:
        q_near = start_point
        dist = distance(q_near,q_rand)
    else:
        q_near,dist = get_qnear(q_rand,G)

    q_new = step_from_to(q_near,q_rand,delta)
    
    if q_new != q_near:
        p1 = geometry.Point(q_new.x,q_new.y)
        p2 = geometry.Point(q_near.x,q_near.y)
        line = geometry.LineString([p1,p2])
        for p in obstacles:
            if p.intersects(line):
                return True
        G.add_node(q_new)
        G.add_edge(q_near,q_new,length=dist)
    collision = False
    #grow goal point to find a solution
    d = distance(q_new,goal_point)
    if d <= 3:
        p1 = geometry.Point(q_new.x,q_new.y)
        p2 = geometry.Point(q_near.x,q_near.y)
        line = geometry.LineString([p1,p2])
        for p in obstacles:
            if p.intersects(line):
                return True

        G.add_node(q_new)
        G.add_edge(q_near,q_new,length=d)
        return


def build(start_point,goal_point,num_nodes,delta,S,G,obstacles):
    connected = False
    first = True
    swap = True
    count = 0
    for i in range(num_nodes):
        count += 1
        if connected == True:
            S.add_edge(start_connect,goal_connect,length=dist)
            G.add_edge(goal_connect,start_connect,length=dist)
            return True

        collide = True
        if swap:
            while collide == True:
                collide = t1(start_point,first,delta,S,obstacles)
            swap=False
        else:
            while collide == True:
                collide = t2(goal_point,first,delta,G,obstacles)
            swap=True
        if count >= num_nodes/5:
            connected,goal_connect,start_connect,dist = connect(S,G,delta)
        first = False
    return False

def connect(S,G,delta):
    min_dist = 9999999
    closest = None
    for n in G.nodes:
        for j in S.nodes:
            dist = distance(n,j)
            if dist <= delta:
                return True,n,j,dist
    return False,None,None,0
        

def distance(p1, p2):
    return math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y), 2))

def step_from_to(p1,p2,delta):
    if distance(p1,p2) < delta:
        return p2
    else:
        theta = atan2(p2.y-p1.y,p2.x-p1.x)
        return Point(p1.x + delta*cos(theta), p1.y + delta*sin(theta))


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

def update(i,edges,ax,color):
    xs = []
    ys = []
    
    es = []
    code = [Path.MOVETO] + [Path.LINETO] + [Path.CLOSEPOLY]
    for e1,e2 in edges:
        es.append((e1,e2))

    e = es[i]
    vertices = [(e[0].x,e[0].y),(e[1].x,e[1].y),(0,0)]
    vertices = np.array(vertices,float)
    path = Path(vertices,code)
    if color == 1:
        pathpatch = patches.PathPatch(path,facecolor='none',edgecolor='green')
    else:
        pathpatch = patches.PathPatch(path,facecolor='none',edgecolor='red')
    ax.add_patch(pathpatch)


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
    start, goal = add_start_and_goal(args.start_goal_path, ax)
    xtest = [56,63,56,46]
    ytest = [18,25,18,14]
    ax.plot(xtest,ytest)
    delta = 10
    s = 0
    polys = []
    temp = []

    for i in vertices:
        if i[0] == 0 and i[1] == 0:
            if s == 0:
                s = 1
                continue
            elif s == 1:
                poly = geometry.Polygon([[p[0], p[1]] for p in temp])
                polys.append(poly)
                temp = []
        else:
            temp.append(i)

    start_point = Point(start[0],start[1])
    goal_point = Point(goal[0],goal[1])
    S = nx.Graph()
    S.add_node(start_point)
    G = nx.Graph()
    G.add_node(goal_point)

    #initialize root
    connect = build(start_point,goal_point,2000,delta,S,G,polys)

    # for edge in G.edges:
    ani1=animation.FuncAnimation(fig,update,frames=len(S.edges),fargs=(S.edges,ax,1,),interval=30,repeat=True)
    ani2=animation.FuncAnimation(fig,update,frames=len(G.edges),fargs=(G.edges,ax,0,),interval=30,repeat=True)
    F = nx.compose(S,G)
    if connect != False:
        shortest_path = nx.shortest_path(F,start_point,goal_point,weight='length')
        x = []
        y = []
        for i in shortest_path:
            x.append(i.x)
            y.append(i.y)
        plt.plot(x,y)
    plt.show()
