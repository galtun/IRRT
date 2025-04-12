import numpy as np
from utils.Node import Node
from utils.Utilize import Utilize
import time

import matplotlib.pyplot as plt
class BiDirectionApfRRTStar:

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
        self.tree_start = [self.start]
        self.tree_target = [self.target]
        self.path = None

    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def sample(self):
        if np.random.rand() < self.targetBias:
            return Node(np.array([self.target.x, self.target.y]))
        x = np.random.uniform(self.lowLimit[0], self.maxLimit[0])
        y = np.random.uniform(self.lowLimit[1], self.maxLimit[1])
        return Node(np.array([x, y]))
    
    def nearest(self, node, tree):
        return min(tree, key=lambda n: self.distance(n, node))
    
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
            pointStart, pointEnd = obs.pointStart, obs.pointEnd
            xMin, xMax, yMin, yMax = pointEnd[0], pointStart[0], pointStart[1], pointEnd[1]
            xClosest = np.clip(node.x, xMin, xMax)
            yClosest = np.clip(node.y, yMin, yMax)
            closetPoint = Node(np.array([xClosest, yClosest]))
            #dist = self.distance(node, closetPoint)
            dist = np.linalg.norm(np.array([node.x, node.y]) - np.array([xClosest, yClosest]))
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
    
    def connect_trees(self, node1, node2):
        if self.distance(node1, node2) < self.stepSize and self.ultz.checkCollision(node1, node2, self.stepSize, self.obstacleList):#self.collision_free(node1, node2):
            return True
        return False
    
    def extract_path(self, node1, node2):
        path1, path2 = [], []
        while node1:
            path1.append(Node(np.array([node1.x, node1.y])))
            node1 = node1.parent
        while node2:
            path2.append(Node(np.array([node2.x, node2.y])))
            node2 = node2.parent
        return path1[::-1] + path2

    def reConstructPath2(self):
        path = []
        node = self.tree[-1]
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(self.tree[0])
        path.reverse()
        return path
        #return path[::-1]
    
    def reconstruct_path(self, start, end):
        path = []
        node1 = start
        while node1.parent is not None:
            path.append(node1)
            node1 = node1.parent
        path.append(node1)
        path.reverse()
        node2 = end
        while node2.parent is not None:
            path.append(node2)
            node2 = node2.parent
        path.append(node2)
        return path

    def run(self):
        path, nodeCount = [], 1
        startTime = time.time()
        for i in range(self.iteration): #while True:
            #print(str(i),end='\r')
            while True:
                rand_node = self.sample()
                nearest_node_star = self.nearest(rand_node, self.tree_start)
                new_node_star = self.apply_potential_field(nearest_node_star, rand_node)
                #if self.collision_free(nearest_node, new_node):
                if self.ultz.checkCollision(nearest_node_star, new_node_star, self.stepSize, self.obstacleList):
                    continue
                else:
                    new_node_star.parent = nearest_node_star
                    self.tree_start.append(new_node_star)
                    #plt.pause(0.10)
                    #plt.plot([new_node_star.x, new_node_star.x], [new_node_star.y, new_node_star.y], 'go', linestyle='-') 
                    break  
            nodeCount += 1
            for n in self.tree_target:
                ds = self.distance(n, new_node_star)
                if  ds < self.momentOfBreak:
                    if nodeCount % 2 == 1:
                        path = self.reconstruct_path(new_node_star, n)
                    else:
                        path = self.reconstruct_path(n, new_node_star)
                    endTime = time.time()
                    return path, nodeCount, endTime - startTime
            self.tree_start, self.tree_target = self.tree_target[:], self.tree_start[:]      
            '''   
            #while True:
            nearest_node_target = self.nearest(rand_node, self.tree_target)
            new_node_target = self.apply_potential_field(nearest_node_target, rand_node)
            if self.ultz.checkCollision(new_node_target, nearest_node_target, self.stepSize, self.obstacleList):
                continue
            else:
                new_node_target.parent = nearest_node_target
                self.tree_target.append(new_node_target)
                nodeCount += 1
                #plt.pause(0.10)
                #plt.plot([nearest_node_target.x, new_node_target.x], [nearest_node_target.y, new_node_target.y], 'go', linestyle='-')
                #break
            #for node_start in self.tree_start:
            for node_goal in self.tree_target:
                if self.connect_trees(new_node_star, node_goal):
                    path = self.extract_path(new_node_star, node_goal)
                    endTime = time.time()
                    return path, nodeCount, endTime - startTime
        endTime = time.time()
        return path, nodeCount, endTime - startTime
                # Check if goal is reached
                #if self.distance(new_node, self.goal) < self.step_size:
                    #self.goal.parent = new_node
                    #self.tree.append(self.goal)
                    #self.path = self.extract_path(self.goal)
                    #return self.path
                #return None
            '''