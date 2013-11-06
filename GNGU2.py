#!/usr/bin/python

import sys
import GNGU
import random

from Tkinter import *
from string import split
from math import exp, cos, sin

LineList = []
OvalList = []
DataOvals = []
Toggle = 0

class LinNode(GNGU.Node):
	maxApproxError = 0.0

	def __init__(self, coords=[0.0], error=0.0, utility=0.0):
		GNGU.Node.__init__(self, coords, error, utility)
		self.avg = 250.0
		self.width = 10.0
		self.targetError = 0.0
		self.avgTargetError = 0.0

	def computeWidth(self):
		d = 0.0
		for n in self.neighbors:
			x = n.coords[0] - self.coords[0]
			d = d + x*x
		self.width = sqrt(d)
		return self.width

	def computeActivation(self, vector):
		d = (vector[0] - self.coords[0])
		return exp(-d/(self.width*self.width))

	def distanceFrom(self, vector):
		d = (vector[0] - self.coords[0])
		self.targetError = (vector[1] - self.avg)
		self.avgTargetError = self.avgTargetError*0.995 + self.targetError*self.targetError
		return d*d

	def adaptCenter(self, inputVector, coeff):
		difficulty = self.avgTargetError/LinNode.maxApproxError
		#print self.id, difficulty
		for i in self.dimRange:
			c = self.coords[i]
			v = inputVector[i]
			self.coords[i] = c + coeff * (v - c) * difficulty
		self.avg = self.avg + coeff * self.targetError

class LinGNGU(GNGU.GNGU):
	def __init__(self, nDims, nodes=[], edges=[]):
		GNGU.GNGU.__init__(self, nDims, nodes, edges)
		self.maxAge = 40
		self.maxNodes = 80
		self.insertInterval = 200
		self.errUtilRatio = 3.0
		self.unitCoeff = 0.1
		self.neighborCoeff = 0.001

	def createNode(self, newCoords, error, utility):
		return LinNode(newCoords, error, utility)

	def createNodeBetween(self, worst, worst2):
		newCoords = range(self.nDims)
		for i in range(self.nDims):
			newCoords[i] = (worst.coords[i] + worst2.coords[i])/2.0

		worst.error = worst.error * self.errorAdj
		worst2.error = worst2.error * self.errorAdj

		worst.width = worst.width / 2.0
		worst2.width = worst2.width / 2.0
		worst.avgTargetError = worst.avgTargetError * self.errorAdj
		worst2.avgTargetError = worst.avgTargetError * self.errorAdj

		newNode = self.createNode(newCoords, worst.error, worst.utility)
		newNode.avg = (worst.avg + worst2.avg)/2.0
		newNode.width = worst.width
		newNode.avgTargetError = worst.avgTargetError
		newEdge1 = newNode.addNeighbor(worst)
		newEdge2 = newNode.addNeighbor(worst2)
		return (newNode, newEdge1, newEdge2)


	def adaptWinnerAndNeighbors(self, inputVector):
		(nearest, secondNearest, totalError) = self.getNearestNodes(inputVector)
		nearest.ageEdges()
		nearest.addToError(nearest.lastError)
		nearest.addToUtility(secondNearest.lastError - nearest.lastError)
		nearest.adaptCenter(inputVector, self.unitCoeff)
		nearest.adaptNeighbors(inputVector, self.neighborCoeff)
		return (nearest, secondNearest)


	def getNearestNodes(self, inputVector):
		distanceTuples = []

		totalError = 0.0
		LinNode.maxApproxError = -1000.0
		for n in self.nodes:
			n.error = n.error * self.errorDecay
			n.lastError = n.distanceFrom(inputVector)
			LinNode.maxApproxError = max(n.avgTargetError, LinNode.maxApproxError)
			distanceTuples.append((n.lastError, n))

		distanceTuples.sort()
		return (distanceTuples[0][1], distanceTuples[1][1], totalError)

def displayNodes(gngu, c):
	global OvalList
	apply(c.delete, OvalList)
	OvalList = []
	for n in gngu.nodes:
		o = 2
		OvalList.append(c.create_oval(n.coords[0]-o, n.avg-o, n.coords[0]+o, n.avg+o))

def displayEdges(gngu, c):
	global LineList
	apply(c.delete, LineList)
	LineList = []
	for e in gngu.edges:
		x1 = e.node1.coords[0]
		y1 = e.node1.avg
		w1 = e.node1.width
		x2 = e.node1.coords[0]
		y2 = e.node2.avg
		w2 = e.node2.width
		if (x1 < x2):
			LineList.append(c.create_line(x1-w1, y1, x1+w1, y1))
			LineList.append(c.create_line(x1+w1, y1, x1+w1, y2))
		else:
			LineList.append(c.create_line(x2-w2, y2, x2+w2, y2))
			LineList.append(c.create_line(x2+w2, y2, x2+w2, y1))


def displayEdges(gngu, c):
	global LineList
	apply(c.delete, LineList)
	LineList = []
	for e in gngu.edges:
		LineList.append(c.create_line(e.node1.coords[0], e.node1.avg, e.node2.coords[0], e.node2.avg))

def learnAndDisplay(root, c, gngu, data):
	try:
		global Toggle, DataOvals
		displayNodes(gngu, c)
		displayEdges(gngu, c)
		for i in range(50):
  			if (gngu.nInputs % 10000 == 0):
  				for o in DataOvals:
  					c.delete(o)
  				DataOvals = []
  				if (Toggle == 1):
  					Toggle = 0
  				else:
  					Toggle = 1
  			if (Toggle == 0):
				x = random.uniform(100.0, 350.0)
				y = 100*cos(x/10)*sin(x/20) + 250
  			else:
				x = random.uniform(100.0, 350.0)
				y = cos(x/20)*(x-250)*(x-250)*.01 + 250
			DataOvals.append(c.create_oval(x,y,x,y,outline="gray"))
			gngu.adaptToInput([x, y])
		if (gngu.nInputs == 5000):
			apply(c.delete, DataOvals)
			DataOvals = []

		root.after(1, learnAndDisplay, root, c, gngu, data)
	except KeyboardInterrupt:
		sys.exit()

def prepDisplay(root, canvas):
	canvas.pack()
	return (root, canvas)

def main():
	node1 = LinNode([100.0])
	node2 = LinNode([400.0])
	edge = GNGU.Edge(node1, node2)
	node1.addEdge(edge)
	node2.addEdge(edge)
	gngu = LinGNGU(1, [node1, node2], [edge])

	(root, canvas) = prepDisplay(Tk(), Canvas(width=500,height=500))
	root.after(1, learnAndDisplay, root, canvas, gngu, [])
	try:
		root.mainloop()
	except:
		sys.exit()

if (__name__ == '__main__'):
	main()
