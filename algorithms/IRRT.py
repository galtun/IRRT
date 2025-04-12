import numpy as np
from utils.Node import Node
from utils.Utilize import Utilize
import time
import matplotlib.pyplot as plt

class IRRT:
    
    ultz = Utilize()
    def __init__(self, stepSize, iteration, startPoint, targetPoint, obstacleList, momentOfBreak, lowLimit, maxLimit):
        self.stepSize = stepSize
        self.iteration = iteration
        self.startPoint = startPoint
        self.targetPoint = targetPoint
        self.obstacleList = obstacleList
        self.momentOfBreak = momentOfBreak
        self.lowLimit = lowLimit
        self.maxLimit = maxLimit
        self.tree = []
        self.distBest = self.distance(self.startPoint, self.targetPoint)

    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
    
    def nearestNeighbor(self, node): # 1.0 - 0.25 # 0.55 - 0.4
        distancesList = [0.99 * self.distance(node, n) + 0.25 * self.distance(n, self.targetPoint) for n in self.tree]
        return self.tree[np.argmin(distancesList)]
    
    def reConstructPath(self):
        path = []
        node = self.tree[-1]
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(node)
        path.reverse()
        return path
        #return path[::-1]

    def nodeGenerator(self):
        randChose = np.random.rand()
        if randChose < 0.05:
            randomNode = Node(np.array([self.targetPoint.x,self.targetPoint.y]))
        else:
            randomNode = Node(np.array([np.random.uniform(self.lowLimit[0], self.maxLimit[0]), np.random.uniform(self.lowLimit[1], self.maxLimit[1])]))
        nearestNode = self.nearestNeighbor(randomNode)
        nearestDist = self.distance(nearestNode, self.targetPoint)
        theta = np.arctan2(randomNode.y - nearestNode.y, randomNode.x - nearestNode.x)
        coefficient =((self.distBest / self.stepSize) + 1.0) * nearestDist / self.distBest
        if coefficient > 1.0 :
            coefficient = 1.0
        x = nearestNode.x + self.stepSize * np.cos(theta) * coefficient
        y = nearestNode.y + self.stepSize * np.sin(theta) * coefficient
        newNode = Node(np.array([x, y]))
        newNode.cost = nearestNode.cost + self.distance(nearestNode, newNode)
        newNode.parent = nearestNode
        return newNode
    
    def smoothRoute(self,path):
        i = 0
        isFirstNodeAppend =False
        newPath = [path[0]]
        while(i < len(path)):
            j = i + 1
            k = i
            while(j < len(path)):
                if not self.ultz.checkCollision(path[k], path[j], self.stepSize, self.obstacleList) and not isFirstNodeAppend:
                    isFirstNodeAppend = True
                    newPath.append(path[j])
                    i = j - 1
                elif not self.ultz.checkCollision(path[k], path[j], self.stepSize, self.obstacleList): 
                    newPath.pop(-1)
                    newPath.append(path[j])
                    i = j - 1
                j += 1
            i += 1
            isFirstNodeAppend = False
        return newPath

    def run(self):
        path, nodeCount = [], 1
        startTime = time.time()
        self.tree.append(self.startPoint)
        for i in range(self.iteration): #while(True):
            while(True):
                newNode = self.nodeGenerator()
                if self.ultz.checkCollision(nearestNode=newNode.parent, newNode=newNode, stepSize=self.stepSize, obstacleList=self.obstacleList):
                    continue
                else:
                    self.tree.append(newNode)
                    break
            nodeCount += 1
            distanceToTarget = self.distance(self.tree[-1], self.targetPoint)
            #plt.pause(0.01)
            #plt.plot([newNode.parent.x, newNode.x], [newNode.parent.y, newNode.y], 'go', linestyle='-')
            if distanceToTarget < self.momentOfBreak:
                path = self.reConstructPath()
                break
        endTime = time.time()
        return self.smoothRoute(path), nodeCount, endTime - startTime
