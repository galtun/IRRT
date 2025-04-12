import numpy as np
from utils.Node import Node
from utils.Utilize import Utilize
import time

class APFRRT:

    ultz = Utilize()
    def __init__(self, stepSize, iteration, startPoint, targetPoint, obstacleList, momentOfBreak, lowLimit, maxLimit, targetBias=0.1):
                #(self, start, goal, bounds, obstacles, step_size=1.0, max_iter=500, goal_bias=0.1):
        self.start = startPoint
        self.target = targetPoint
        #self.bounds = bounds
        self.momentOfBreak = momentOfBreak
        self.lowLimit = lowLimit
        self.maxLimit = maxLimit

        self.obstacleList = obstacleList
        self.stepSize = stepSize
        self.iteration = iteration
        self.targetBias = targetBias
        self.tree = [self.start]
        self.path = None

    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def sample(self):
        if np.random.rand() < self.targetBias:
            return Node(np.array([self.target.x, self.target.y]))
        x = np.random.uniform(self.lowLimit[0], self.maxLimit[0])
        y = np.random.uniform(self.lowLimit[1], self.maxLimit[1])
        return Node(np.array([x, y]))
    
    def nearest(self, node):
        return min(self.tree, key=lambda n: self.distance(n, node))
    
    # Kullanilmamis
    def steer(self, from_node, to_node):
        dist = self.distance(from_node, to_node)
        if dist < self.stepSize:
            return to_node
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.stepSize * np.cos(theta)
        new_y = from_node.y + self.stepSize * np.sin(theta)
        return Node(np.array[new_x, new_y])
    
    def collision_free(self, node1, node2):
        for obs in self.obstacleList:
            center, radius = obs
            d = np.abs(np.cross([node2.x - node1.x, node2.y - node1.y], [center[0] - node1.x, center[1] - node1.y])) / self.distance(node1, node2)
            if d < radius:
                return False
        return True

    def potential_field(self, node, target):
        # Attractive force towards the goal
        ka = 1.0  # Attractive gain
        att_force = ka * np.array([target.x - node.x, target.y - node.y])

        # Repulsive force from obstacles
        kr = 100.0  # Repulsive gain
        rep_force = np.array([0.0, 0.0])
        for obs in self.obstacleList:
            #center, radius = obs
            #dist = self.distance(node, Node(center[0], center[1]))
            if obs.obsType ==1 :
                pointStart, pointEnd = obs.pointStart, obs.pointEnd
                xMin, xMax, yMin, yMax = pointEnd[0], pointStart[0], pointStart[1], pointEnd[1]
                xClosest = np.clip(node.x, xMin, xMax)
                yClosest = np.clip(node.y, yMin, yMax)
                closetPoint = Node(np.array([xClosest, yClosest]))
                #dist = self.distance(node, closetPoint)
                dist = np.linalg.norm(np.array([node.x, node.y]) - np.array([xClosest, yClosest]))
            elif obs.obsType ==0 :
                center, radius = obs.pointStart, obs.pointEnd
                dist = self.distance(node, Node(center))
            radius = self.stepSize
            if dist < radius:
                rep_force += kr * (1.0 / dist - 1.0 / radius) * (1.0 / dist**2) * np.array([node.x - closetPoint.x, node.y - closetPoint.y])

        # Total force
        total_force = att_force + rep_force
        return total_force

    def apply_potential_field(self, node, target):
        force = self.potential_field(node, target)
        norm = np.linalg.norm(force)
        if norm > self.stepSize:
            force = (force / norm) * self.stepSize
        return Node(np.array([node.x + force[0], node.y + force[1]]))

    def extract_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

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
    
    def run(self):
        path, nodeCount = [], 1
        startTime = time.time()
        for i in range(self.iteration): #while True:
            while True:
                rand_node = self.sample()
                nearest_node = self.nearest(rand_node)
                new_node = self.apply_potential_field(nearest_node, rand_node)
                #if self.collision_free(nearest_node, new_node):
                if self.ultz.checkCollision(nearest_node, new_node, self.stepSize, self.obstacleList):
                    continue
                else:
                    new_node.parent = nearest_node
                    self.tree.append(new_node)
                    break
            nodeCount += 1
            distanceToTarget = self.distance(self.tree[-1], self.target)
            if distanceToTarget < self.momentOfBreak:
                path = self.reConstructPath()
                break
        endTime = time.time()
        return path, nodeCount, endTime - startTime
                # Check if goal is reached
                #if self.distance(new_node, self.goal) < self.step_size:
                    #self.goal.parent = new_node
                    #self.tree.append(self.goal)
                    #self.path = self.extract_path(self.goal)
                    #return self.path
                #return None