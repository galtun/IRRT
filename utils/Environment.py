import sys
sys.path.append('/Users/galtun/Projects/RRTWorkspacev2/utils')
from Obstacle import Obstacle
from Node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Environment:

    def __init__(self, pointStart = None, pointEnd = None):
        self.pointStart = pointStart
        self.pointEnd = pointEnd
        self.obsticleList = []

    def list1(self):
        self.obsticleList = []
        self.obsticleList.append(Obstacle(np.array([22, 10]), np.array([8, 30]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([22, 35]), np.array([8, 55]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([11, 70]), np.array([-11, 90]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([-8, 10]), np.array([-22, 30]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([-8, 35]), np.array([-22, 55]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([-33, 55]), np.array([-47, 75]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([-33, 85]), np.array([-47, 105]), 1, 0))
        return '1NoluOrtam',Node(np.array([0, 0])), Node(np.array([-35, 80])), self.obsticleList

    def list2(self):
        self.obsticleList = []
        self.obsticleList.append(Obstacle(np.array([35, -100]), np.array([15, 30])))
        self.obsticleList.append(Obstacle(np.array([35, 50]), np.array([15, 100])))
        self.obsticleList.append(Obstacle(np.array([-15, 30]), np.array([-35, 100])))
        self.obsticleList.append(Obstacle(np.array([-15, -100]), np.array([-35, -50])))
        return np.array([0, 0]), np.array([-35, 80]), self.obsticleList

    def list3(self):
        self.obsticleList = []
        self.obsticleList.append(Obstacle(np.array([120, 245]), np.array([0, 270]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([120, 205]), np.array([60, 230]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([350, 155]), np.array([25, 180]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([120, 80]), np.array([0, 105]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([45, 40]), np.array([0, 65]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([120, 40]), np.array([60, 65]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([395, 245]), np.array([270, 270]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([350, 205]), np.array([270, 230]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([420, 80]), np.array([270, 105]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([350, 35]), np.array([270, 60]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([210, 245]), np.array([135, 270]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([210, 205]), np.array([135, 230]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([210, 80]), np.array([135, 105]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([210, 40]), np.array([135, 65]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([160, 245]), np.array([135, 330]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([85, 180]), np.array([60, 230]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([90, 0]), np.array([60, 65]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([165, 80]), np.array([135, 230]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([160, 0]), np.array([135, 65]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([255, 255]), np.array([230, 330]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([255, 80]), np.array([230, 230]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([255, 0]), np.array([230, 65]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([350, 245]), np.array([325, 315]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([350, 155]), np.array([325, 230]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([350, 105]), np.array([325, 140]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([350, 0]), np.array([325, 60]), 1, 0))
        return '3NoluOrtam',Node(np.array([4, 20])), Node(np.array([375, 205])), self.obsticleList
    
    def list4(self):
        self.obsticleList = []
        self.obsticleList.append(Obstacle(np.array([300, 0]), np.array([100,235]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([195, 235]), np.array([100, 285]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([300, 250]), np.array([210, 300]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([300, 300]), np.array([100,400]), 1, 0))
        return '4NoluOrtam',Node(np.array([50,50])), Node(np.array([350, 50])), self.obsticleList

    def list5(self):
        self.obsticleList = []
        self.obsticleList.append(Obstacle(np.array([125, 0]), np.array([100,275]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([125, 300]), np.array([100,400]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([250, 0]), np.array([225,100]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([250, 125]), np.array([225,400]), 1, 0))
        return '2NoluOrtam',Node(np.array([50,50])), Node(np.array([350, 350])), self.obsticleList
    
    def list6(self):
        self.obsticleList = []
        #self.obsticleList.append(Obstacle(np.array([-60, -40]), np.array([-80,0])))#1
        self.obsticleList.append(Obstacle(np.array([20, 20]), np.array([-20,40]), 1, 0))#2
        self.obsticleList.append(Obstacle(np.array([130, 45]), np.array([90,65]), 1, 0))#3
        self.obsticleList.append(Obstacle(np.array([-125, 80]), np.array([-165,100]), 1, 0))#4
        self.obsticleList.append(Obstacle(np.array([-60, 110]), np.array([-80,150]), 1, 0))#5
        self.obsticleList.append(Obstacle(np.array([-35, 20]), np.array([-115,60]), 1, 0))#6
        self.obsticleList.append(Obstacle(np.array([90, 90]), np.array([10,130]), 1, 0))#7
        self.obsticleList.append(Obstacle(np.array([60, -90]), np.array([-20,-50]), 1, 0))#8
        self.obsticleList.append(Obstacle(np.array([-30, 170]), np.array([-150,230]), 1, 0))#9
        return 'DinamikOrtam',Node(np.array([15,-20])), Node(np.array([-150, 150], 1, 0)), self.obsticleList
    
    def list7(self):
        self.obsticleList= []
        self.obsticleList.append(Obstacle(np.array([-40, 125]), np.array([-100, 175]), 1, 0))#Sabit Engel Sol
        self.obsticleList.append(Obstacle(np.array([100, 125]), np.array([40,175]), 1, 0))#Sabit Engel Sağ

        self.obsticleList.append(Obstacle(np.array([60, 80]), np.array([40,100]), 1, 1))#Kare alt 1
        self.obsticleList.append(Obstacle(np.array([10, 80]), np.array([-10,100]), 1, 1))#Kare alt 2
        self.obsticleList.append(Obstacle(np.array([-40, 80]), np.array([-60,100]), 1, 1))#SKare alt 3

        self.obsticleList.append(Obstacle(np.array([75, 50]), 10, 0, 1))#Daire alt 1
        self.obsticleList.append(Obstacle(np.array([25, 50]), 10, 0, 1))#Daire alt 2
        self.obsticleList.append(Obstacle(np.array([-25, 50]), 10, 0, 1))#Daire alt 3
        self.obsticleList.append(Obstacle(np.array([-75, 50]), 10, 0, 1))#Daire alt 4

        self.obsticleList.append(Obstacle(np.array([60, 240]), np.array([40,260]), 1, 1))#Kare üst 1
        self.obsticleList.append(Obstacle(np.array([10, 240]), np.array([-10,260]), 1, 1))#Kare üst 2
        self.obsticleList.append(Obstacle(np.array([-40, 240]), np.array([-60,260]), 1, 1))#SKare üst 3

        self.obsticleList.append(Obstacle(np.array([75, 205]), 10, 0, 1))#Daire üst 1
        self.obsticleList.append(Obstacle(np.array([25, 205]), 10, 0, 1))#Daire üst 2
        self.obsticleList.append(Obstacle(np.array([-25, 205]), 10, 0, 1))#Daire üst 3
        self.obsticleList.append(Obstacle(np.array([-75, 205]), 10, 0, 1))#Daire üst 4

        return 'DinamikOrtam -2',Node(np.array([0,0])), Node(np.array([25, 300])), self.obsticleList

    def list8(self):
        self.obsticleList = []
        self.obsticleList.append(Obstacle(np.array([0, 33]), np.array([0,36]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([3, 41]), np.array([-5,44]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([-2, 56]), np.array([-9,59]), 1, 0))
        self.obsticleList.append(Obstacle(np.array([-4, 41]), np.array([1,44]), 1, 0))
        return '8NoluOrtam',Node(np.array([-6,48])), Node(np.array([20, 100])), self.obsticleList

    def showEviroment(self):
        name, startNode, goalNode, oList = Environment().list7()    
        #nodes = [Node(startNode)]
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, aspect='equal')
        pathDist = 0.0

        for obsticle in oList:
            if obsticle.obsType == 1 and obsticle.moveType ==1 :
                width = obsticle.pointStart[0] - obsticle.pointEnd[0]
                height = obsticle.pointEnd[1] - obsticle.pointStart[1]
                square = patches.Rectangle((obsticle.pointEnd[0], obsticle.pointStart[1]), width=width, height=height, edgecolor='gray', facecolor='gray')
                ax.add_patch(square)
            elif obsticle.obsType == 0:
                circle = patches.Circle((obsticle.pointStart[0], obsticle.pointStart[1]),obsticle.pointEnd, color='black')#
                ax.add_patch(circle)
            else:
                width = obsticle.pointStart[0] - obsticle.pointEnd[0]
                height = obsticle.pointEnd[1] - obsticle.pointStart[1]
                square = patches.Rectangle((obsticle.pointEnd[0], obsticle.pointStart[1]), width=width, height=height, edgecolor='black', facecolor='black')
                ax.add_patch(square)
        plt.scatter(startNode.x, startNode.y, color='green', marker='o', label='Start')
        plt.scatter(goalNode.x, goalNode.y, color='red', marker='x', label='Destination')
        plt.legend(fontsize=12)
        plt.xlim(0, 410)
        #plt.grid()
        plt.xlabel('X Axis')
        plt.ylabel('Y Axis')
        #plt.title(name)
        #plt.savefig(name+'.png', dpi=300, bbox_inches='tight')
        plt.show()

#x = Environment()
#x.showEviroment()
