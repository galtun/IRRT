import numpy as np
from utils.Node import Node
from utils.Utilize import Utilize
import matplotlib.pyplot as plt
import time

class RRTConnect:

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
    
    def nearestNeighbor(self, node, nodeList):
        distancesList = [self.distance(n, node) for n in nodeList]
        return nodeList[np.argmin(distancesList)]
    
    def reconstruct_path(self, start, end):
        path = []
        node1 = start
        try:
            while node1.parent is not None:
                path.append(node1)
                node1 = node1.parent
            path.append(node1)
            path.reverse()
            node2 = end.parent
            while node2.parent is not None:
                path.append(node2)
                node2 = node2.parent
            path.append(node2)
        except Exception as e:
            print(e)
        return path
    
    def nodeGenerator(self, nodeList, randomNode = None):
        if randomNode == None:
            randomNode = Node(np.array([np.random.uniform(self.lowLimit[0], self.maxLimit[0]), np.random.uniform(self.lowLimit[1], self.maxLimit[1])]))
        nearestNode = self.nearestNeighbor(randomNode, nodeList)
        theta = np.arctan2(randomNode.y - nearestNode.y, randomNode.x - nearestNode.x)
        x = nearestNode.x + self.stepSize * np.cos(theta)
        y = nearestNode.y + self.stepSize * np.sin(theta)
        newNode = Node(np.array([x, y]))
        newNode.cost = nearestNode.cost + self.distance(nearestNode, newNode)
        newNode.parent = nearestNode
        return newNode

    def run(self): #(iteration, step_size, nodeTaret):
        nodeListA = [self.start]
        nodeListB = [self.target]
        path, nodeCount = [], 1
        startTime = time.time()
        for i in range(self.iteration): #while(True):
            while (True):
                newNodeA = self.nodeGenerator(nodeListA)           
                if self.ultz.checkCollision(newNodeA.parent, newNodeA, self.stepSize, self.obstacleList):
                    continue
                else:
                    nodeListA.append(newNodeA)
                    break
            nodeCount += 1
            #plt.pause(0.50)
            #plt.plot([newNodeA.parent.x, newNodeA.x], [newNodeA.parent.y, newNodeA.y], 'go', linestyle='-')
            for n in nodeListB:
                ds = self.distance(n, newNodeA)
                if  ds < self.momentOfBreak:
                    if nodeCount % 2 == 1:
                        path = self.reconstruct_path(newNodeA, n)
                    else:
                        path = self.reconstruct_path(n, newNodeA)
                        path.reverse()
                    endTime = time.time()
                    return path, nodeCount, endTime - startTime
            while(True):
                newNodeB = self.nodeGenerator(nodeListB, randomNode=newNodeA)
                if self.ultz.checkCollision(newNodeB.parent, newNodeB, self.stepSize, self.obstacleList):
                    break
                else:
                    nodeListB.append(newNodeB)
                    #plt.pause(0.50)
                    #plt.plot([newNodeB.parent.x, newNodeB.x], [newNodeB.parent.y, newNodeB.y], 'go', linestyle='-')
                    break
            for n in nodeListA:
                ds = self.distance(n, newNodeB)
                if  ds < self.momentOfBreak:
                    if nodeCount % 2 == 1:
                        path = self.reconstruct_path(newNodeB, n)
                        path.reverse()
                    else:
                        path = self.reconstruct_path(n, newNodeB)
                    endTime = time.time()
                    return path, nodeCount, endTime - startTime
            #nodeListA, nodeListB = nodeListB[:], nodeListA[:]