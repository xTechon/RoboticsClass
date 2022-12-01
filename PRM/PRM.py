"""Creates a graph and traverses the graph using dijkstra's algorithm"""

import copy
import random
import math

import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.collections as cl

# treating values in centimeters
n_vertices = 200
d = 10  # distance
# the limit of the randomly generated points
limitX = 100
limitY = 100

# start and goal position
startx = 5
starty = 50

goalx = 90
goaly = 50

# init random generator
random.seed()


# Node object with x and y coord
# give prm a start(x,y) and goal position(x,y)
# after nodes created, make edges
# pass nodes and edges to nx
# path plan with nx
# print out with nx


class Node:
    """Node object contains x and y float coordinate."""

    def __init__(self, x, y):
        """Class constructor."""
        self.__x = x
        self.__y = y

    def point(self):
        """Return x,y as list."""
        return [self.__x, self.__y]

    def tup(self):
        """Return x,y as tupple."""
        return (self.__x, self.__y)

    def __str__(self):
        """Return the string value of the node."""
        return (self.__x, self.__y).__str__()

    def x(self):
        """Return the x value of the object."""
        return self.__x

    def y(self):
        """Return the y value of the object."""
        return self.__y


start = Node(startx, starty)
goal = Node(goalx, goaly)


# make nodes

verticies = []
gVerticies = []
vertx = []
verty = []

# generate nodes and points
for interator in range(n_vertices):
    xx = random.randint(0, limitX - 1) + random.random()
    yy = random.randint(0, limitY - 1) + random.random()
    temp = Node(xx, yy)
    verticies.append(copy.deepcopy(temp))
    gVerticies.append(copy.deepcopy(temp.tup()))
    vertx.append(temp.x())
    verty.append(temp.y())

# make edges
edges = []
plotEdge = []
for node in verticies:
    # print(node)
    for dest in verticies:
        if (math.dist(node.point(), dest.point()) <= d) and (
            node.point() != dest.point()
        ):
            edges.append((node.tup(), dest.tup()))
            plotEdge.append([node.tup(), dest.tup()])
            # print("edge added")

# remove duplicate edges
for edge in edges:
    for edger in edges:
        if (edge[1], edge[0]) == edger:
            edges.remove(edge)
            plotEdge.remove([edge[0], edge[1]])

# add start and goal points and edges
verticies.append(start)
gVerticies.append(start.tup())

# add start to graph
for vert in verticies:
    if (math.dist(start.point(), vert.point())) <= d:
        edges.append((start.tup(), vert.tup()))
        plotEdge.append([start.tup(), vert.tup()])
        break

# add goal to graph
for vert in verticies:
    if (math.dist(goal.point(), vert.point())) <= d:
        edges.append((goal.tup(), vert.tup()))
        plotEdge.append([goal.tup(), vert.tup()])
        break


# turn into graph with nx
G = nx.MultiGraph()

G.add_nodes_from(gVerticies)
G.add_edges_from(edges)

# draw with mathplotlib
fig, ax = plt.subplots()
ax.plot(vertx, verty, ".b")  # add points
ax.plot(start.x(), start.y(), ".g")
ax.plot(goal.x(), goal.y(), ".r")

edgeCollection = cl.LineCollection(plotEdge, linewidths=0.5)  # add edges
ax.add_collection(edgeCollection)
ax.grid(True)
ax.axis("equal")

# generate garph using dijkstra's through nx
path = nx.shortest_path(G, start.tup(), goal.tup())

# print path and scale to cm before showing graph
for point in path:
    print(str(round((point[0] / 100), 5)) + ", " + str(round((point[1] / 100), 5)))

pathPlotx = []
pathPloty = []
for point in path:
    pathPlotx.append(point[0])
    pathPloty.append(point[1])

ax.plot(pathPlotx, pathPloty, "orange")
plt.show()
