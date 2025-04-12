import numpy as np
from utils.Node import Node
from utils.Utilize import Utilize
import time
import matplotlib.pyplot as plt

class RRT:

    ultz = Utilize()
    def __init__(self, stepSize, iteration, startPoint, targetPoint, obstacleList, momentOfBreak, lowLimit, maxLimit):
        self.stepSize = stepSize
        self.iteration = iteration        
        self.start = startPoint
        self.target = targetPoint
        self.obstacleList = obstacleList
        self.momentOfBreak = momentOfBreak
        self.lowLimit = lowLimit
        self.maxLimit = maxLimit
        self.tree = [self.start]
    
    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
    
    def nearestNeighbor(self, node):
        distancesList = [self.distance(n, node) for n in self.tree]
        return self.tree[np.argmin(distancesList)]
    
    def reConstructPath(self):
        path = []
        node = self.tree[-1]
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(self.tree[0])
        path.reverse()
        return path
        #return path[::-1]

    def nodeGenerator(self):
        randomNode = Node(np.array([np.random.uniform(self.lowLimit[0], self.maxLimit[0]), np.random.uniform(self.lowLimit[1], self.maxLimit[1])]))
        nearestNode = self.nearestNeighbor(randomNode)
        theta = np.arctan2(randomNode.y - nearestNode.y, randomNode.x - nearestNode.x)
        x = nearestNode.x + self.stepSize * np.cos(theta)
        y = nearestNode.y + self.stepSize * np.sin(theta)
        newNode = Node(np.array([x, y]))
        newNode.cost = nearestNode.cost + self.distance(nearestNode, newNode)
        newNode.parent = nearestNode
        return newNode

    def run(self):
        path, nodeCount = [], 1
        startTime = time.time()
        for i in range(self.iteration): #while(True):
            while(True):
                newNode = self.nodeGenerator()
                if self.ultz.checkCollision(newNode.parent, newNode, self.stepSize, self.obstacleList):
                    continue
                else:
                    self.tree.append(newNode)
                    break
            nodeCount += 1
            distanceToTarget = self.distance(self.tree[-1], self.target)
            #plt.pause(0.10)
            #plt.plot([newNode.parent.x, newNode.x], [newNode.parent.y, newNode.y], 'go', linestyle='-')
            if distanceToTarget < self.momentOfBreak:
                path = self.reConstructPath()
                break
        endTime = time.time()
        return path, nodeCount, endTime - startTime
