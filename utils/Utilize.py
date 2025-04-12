import math
import numpy as np
import matplotlib.patches as patches
from utils.Node import Node

class Utilize:
    def __init__(self):
        pass


    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def checkCollision(self, nearestNode, newNode, stepSize, obstacleList):
        try:
            m  = (newNode.y - nearestNode.y) / (newNode.x - nearestNode.x)
            for i in range(1, stepSize * 10 + 1):
                x = nearestNode.x + (newNode.x - nearestNode.x) * i / (stepSize * 10)
                y = nearestNode.y + m * (x - nearestNode.x)
                for obstacle in obstacleList:
                    if obstacle.obsType == 1:
                        if obstacle.pointStart[0] > x and x > obstacle.pointEnd[0] and obstacle.pointEnd[1] > y and y > obstacle.pointStart[1]:
                            return True
                    elif obstacle.obsType == 0:
                        center, radius = obstacle.pointStart, obstacle.pointEnd
                        #d = np.abs(np.cross([newNode.x - nearestNode.x, newNode.y - nearestNode.y], [center[0] - nearestNode.x, center[1] - nearestNode.y])) / self.distance(nearestNode, newNode)
                        #if d <= radius:
                        #    return True
                        sv = (newNode.x - nearestNode.x, newNode.y - nearestNode.y)
                        dist = sv[0]**2 + sv[1]**2
                        t = ((center[0] - nearestNode.x) * sv[0] +
                            (center[1] - nearestNode.y) * sv[1]) / dist
                        t = max(0, min(1, t))
                        nPoint = (nearestNode.x + t * sv[0],
                                nearestNode.y + t * sv[1])
                        if self.distance(Node(np.array([nPoint[0],nPoint[1]])), Node(np.array([center[0],center[1]]))) <= radius:
                            return True
            return False
        except Exception as e:
            print(e)
    
    def checkCollision2(self, nearestNode, newNode, stepSize, obstacleList):
        m  = (newNode.y - nearestNode.y) / (newNode.x - nearestNode.x)
        for i in range(1, stepSize * 10 + 1):
            x = nearestNode.x + (newNode.x - nearestNode.x) * i / (stepSize * 10)
            y = nearestNode.y + m * (x - nearestNode.x)
            for i, obstacle in enumerate(obstacleList):
                if isinstance(obstacle, patches.Rectangle):
                    if obstacle.xy[0] + obstacle._width > x and x > obstacle.xy[0] and obstacle.xy[1] + obstacle._height > y and y > obstacle.xy[1]:
                        #print(i,obstacle.xy)
                        return True
                elif isinstance(obstacle, patches.Circle):
                    center, radius = obstacle.center, obstacle.radius
                    #d = np.abs(np.cross([newNode.x - nearestNode.x, newNode.y - nearestNode.y], [center[0] - nearestNode.x, center[1] - nearestNode.y])) / self.distance(nearestNode, newNode)
                    #if d <= radius:
                    #    return True
                    sv = (newNode.x - nearestNode.x, newNode.y - nearestNode.y)
                    dist = sv[0]**2 + sv[1]**2
                    t = ((center[0] - nearestNode.x) * sv[0] +
                         (center[1] - nearestNode.y) * sv[1]) / dist
                    t = max(0, min(1, t))
                    nPoint = (nearestNode.x + t * sv[0],
                              nearestNode.y + t * sv[1])
                    if self.distance(Node(np.array([nPoint[0],nPoint[1]])), Node(np.array([center[0],center[1]]))) <= radius:
                        return True
        return False
    
    def calculatePathDistance(self, path):
        distance = 0
        for i in range(len(path) - 1):
            distance += math.sqrt((path[i].x - path[i + 1].x) ** 2 + (path[i].y - path[i + 1].y) ** 2)
        return distance