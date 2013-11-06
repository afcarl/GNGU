#!/usr/bin/env python

import sys
import random

from Tkinter import *
from math import sqrt
from string import split

class Edge:
    _edgeID = 0
    def __init__(self, node1, node2, age=0.0):
        self.id = Edge._edgeID
        Edge._edgeID = Edge._edgeID + 1
        self.node1 = node1
        self.node2 = node2
        self.age = age

    def __repr__(self):
        return "E(%d, (%d - %d), %.1f)" % (self.id, self.node1.id, self.node2.id, self.age)

    def __str__(self):
        return "E(%d, (%d - %d), %.1f)" % (self.id, self.node1.id, self.node2.id, self.age)

    def detach(self):
        self.node1.deleteEdge(self)
        self.node2.deleteEdge(self)

    def incrementAge(self):
        self.age = self.age + 1

class Node:
    _nodeID = 0
    def __init__(self, coords=[0.0], error=0.0, utility=0.0):
        self.id = Node._nodeID
        Node._nodeID = Node._nodeID + 1
        self.error = error
        self.utility = utility
        self.coords = coords
        self.nDims = len(self.coords)
        self.dimRange = range(self.nDims)
        self.edges = []
        self.neighbors = []
        self.lastError = 0.0

    def __repr__(self):
        #return "N(%d, %s, %.1f %.1f)" % (self.id, str(self.coords), self.error, self.utility)
        return self.__str__()

    def __str__(self):
        return "N(%d, %.1f, %s)" % (self.id, self.error, str(self.edges))

    def ageEdges(self):
        for e in self.edges:
            #print e
            e.incrementAge()

    def addToError(self, error):
        self.error = self.error + error

    def addToUtility(self, utility):
        self.utility = self.utility + utility

    def adaptCenter(self, inputVector, coeff):
        for i in self.dimRange:
            c = self.coords[i]
            v = inputVector[i]
            self.coords[i] = c + coeff * (v - c)

    def adaptNeighbors(self, inputVector, coeff):
        for n in self.neighbors:
            n.adaptCenter(inputVector, coeff)

    def connectedTo(self, node):
        return node in self.neighbors

    def getEdgeTo(self, node):
        for e in self.edges:
            if (e.node1 == node or e.node2 == node):
                return e
        return None

    def addEdge(self, edge):
        if (self == edge.node1):
            self.neighbors.append(edge.node2)
        else:
            self.neighbors.append(edge.node1)
        self.edges.append(edge)

    def addNeighbor(self, node):
        node.neighbors.append(self)
        self.neighbors.append(node)

        newEdge = Edge(self, node)

        self.edges.append(newEdge)
        node.edges.append(newEdge)

        return newEdge

    def deleteEdge(self, edge):
        #print self, edge, self.neighbors
        if (self == edge.node1):
            self.neighbors.remove(edge.node2)
            self.edges.remove(edge)
        elif (self == edge.node2):
            self.neighbors.remove(edge.node1)
            self.edges.remove(edge)
        else:
            print "not my edge", self, edge

    def distanceFrom(self, vector):
        d = 0
        for i in self.dimRange:
            diff = vector[i] - self.coords[i]
            d = d + diff*diff
        return d

    def detachFromNeighbors(self):
        for e in self.edges:
            if (self == e.node1):
                e.node2.edges.remove(e)
                e.node2.neighbors.remove(self)
            else:
                e.node1.edges.remove(e)
                e.node1.neighbors.remove(self)
        return self.edges

    def getWorstNeighbor(self):
        worstNeighbor = None
        worstError = -1.0
        #print "==", self, self.neighbors
        for n in self.neighbors:
            if (n.error > worstError):
                worstNeighbor = n
                worstError = n.error
            #print "-", n, worstNeighbor, worstError
        return worstNeighbor

class GNGU:
    def __init__(self, nDims, nodes=[], edges=[]):
        self.nodes = nodes
        self.edges = edges
        self.nDims = nDims
        self.unitCoeff = 0.2
        self.neighborCoeff = 0.002
        self.maxAge = 50
        self.insertInterval = 150
        self.errorAdj = 0.5
        self.utilDecay = 0.992
        self.errorDecay = 0.995
        self.errUtilRatio = 6.0
        self.nInputs = 0
        self.maxNodes = 100

    def __repr__(self):
        return "GNGU(%s, %s)" % (str(self.nodes), str(self.edges))

    def __str__(self):
        return self.__repr__()

    def connect(self, node1, node2):
        edge = self.createEdge(node1, node2)
        node1.addEdge(edge)
        node2.addEdge(edge)
        self.edges.append(edge)

    def removeExpiredEdges(self):
        if (len(self.edges) < 2):
            return

        deadEdges = []
        for e in self.edges:
            if (e.age > self.maxAge):
                deadEdges.append(e)
                e.detach()

        for e in deadEdges:
            self.edges.remove(e)

        deadNodes = []
        for n in self.nodes:
            if (len(n.edges) == 0):
                deadNodes.append(n)

        for d in deadNodes:
            self.nodes.remove(d)

    def getNearestNodes(self, inputVector):
        distanceTuples = []

        totalError = 0.0
        for n in self.nodes:
            n.error = n.error * self.errorDecay
            n.lastError = n.distanceFrom(inputVector)
            distanceTuples.append((n.lastError, n))

        distanceTuples.sort()

        return (distanceTuples[0][1], distanceTuples[1][1], totalError)

    def getWorstErrorNode(self):
        worstNode = self.nodes[0]
        worstError = -1.0

        for n in self.nodes:
            if (n.error > worstError):
                worstNode = n
                worstError = n.error
            #n.error = 0

        return worstNode

    def createNode(self, newCoords, error, utility):
        return Node(newCoords, error, utility)

    def createEdge(self, node1, node2):
        return Edge(node1, node2)

    def createNodeBetween(self, worst, worst2):
        newCoords = range(self.nDims)
        for i in range(self.nDims):
            newCoords[i] = (worst.coords[i] + worst2.coords[i])/2.0

        worst.error = worst.error * self.errorAdj
        worst2.error = worst2.error * self.errorAdj

        newNode = self.createNode(newCoords, worst.error, worst.utility)
        newEdge1 = newNode.addNeighbor(worst)
        newEdge2 = newNode.addNeighbor(worst2)
        return (newNode, newEdge1, newEdge2)

    def splitWorstPair(self):
        worst = self.getWorstErrorNode()
        worstNeighbor = worst.getWorstNeighbor()
        splitEdge = worst.getEdgeTo(worstNeighbor)

        worst.deleteEdge(splitEdge)
        worstNeighbor.deleteEdge(splitEdge)
        (newNode, newEdge1, newEdge2) = self.createNodeBetween(worst, worstNeighbor)

        self.edges.remove(splitEdge)

        self.nodes.append(newNode)
        self.edges.append(newEdge1)
        self.edges.append(newEdge2)

    def removeNode(self, node):
        if (len(self.nodes) < 3):
            return
        edges = node.detachFromNeighbors()
        for e in edges:
            self.edges.remove(e)
        self.nodes.remove(node)     

    def removeLowUtilityUnits(self):
        if (len(self.nodes) < 3):
            return
        doomedNodes = []
        for n in self.nodes:
            if (n.utility > 0.0 and n.error/n.utility >= self.errUtilRatio):
                doomedNodes.append(n)
            n.utility = n.utility * self.utilDecay
        for n in doomedNodes:
            self.removeNode(n)

    def adaptWinnerAndNeighbors(self, inputVector):
        (nearest, secondNearest, totalError) = self.getNearestNodes(inputVector)
        nearest.ageEdges()
        nearest.addToError(nearest.lastError)
        nearest.addToUtility(secondNearest.lastError - nearest.lastError)
        nearest.adaptCenter(inputVector, self.unitCoeff)
        nearest.adaptNeighbors(inputVector, self.neighborCoeff)
        return (nearest, secondNearest)

    def checkConnection(self, nearest, secondNearest):
        if (not nearest.connectedTo(secondNearest)):
            self.connect(nearest, secondNearest)
        else:
            edge = nearest.getEdgeTo(secondNearest)
            edge.age = 0

    def adaptToInput(self, inputVector):
        (nearest, secondNearest) = self.adaptWinnerAndNeighbors(inputVector)
        self.checkConnection(nearest, secondNearest)
        self.removeExpiredEdges()
        if (len(self.nodes) < self.maxNodes and self.nInputs > 0 and self.nInputs % self.insertInterval == 0):
            self.splitWorstPair()
        self.removeLowUtilityUnits()
        self.nInputs = self.nInputs + 1

LineList = []
OvalList = []
DataOvals = []
Toggle = 0

def displayNodes(gngu, c):
    global OvalList

    apply(c.delete, OvalList)
    OvalList = []
    for n in gngu.nodes:
        o = 2
        OvalList.append(c.create_oval(n.coords[0]-o, n.coords[1]-o, n.coords[0]+o, n.coords[1]+o, outline="red"))


def displayEdges(gngu, c):
    global LineList

    apply(c.delete, LineList)
    LineList = []
    for e in gngu.edges:
        LineList.append(c.create_line(e.node1.coords[0], e.node1.coords[1], e.node2.coords[0], e.node2.coords[1]))


def learnAndDisplay(root, c, gngu):
    try:
        global Toggle, DataOvals
        displayNodes(gngu, c)
        displayEdges(gngu, c)
        for i in range(50):
            if (gngu.nInputs % 5000 == 0):
                for o in DataOvals:
                    c.delete(o)
                DataOvals = []
                if (Toggle == 1):
                    Toggle = 0
                else:
                    Toggle = 1
            if (Toggle == 1):
                x = random.uniform(100.0, 350.0)
                y = random.uniform(100.0, 250.0)
            else:
                x = random.gauss(350, 20)
                y = random.gauss(300, 50)
            DataOvals.append(c.create_oval(x,y,x,y,outline="gray"))
            gngu.adaptToInput([x, y])

        root.after(1, learnAndDisplay, root, c, gngu)
    except KeyboardInterrupt:
        sys.exit()

def prepDisplay(root, canvas):
    canvas.pack()
    return (root, canvas)

def main():
    node1 = Node([100.0, 250.0])
    node2 = Node([400.0, 250.0])
    edge = Edge(node1, node2)
    node1.addEdge(edge)
    node2.addEdge(edge)
    gngu = GNGU(2, [node1, node2], [edge])

    (root, canvas) = prepDisplay(Tk(), Canvas(width=500,height=500))
    root.after(1, learnAndDisplay, root, canvas, gngu)
    try:
        root.mainloop()
    except:
        sys.exit()

if (__name__ == '__main__'):
    main()
