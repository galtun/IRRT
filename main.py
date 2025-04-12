import time
import threading
import numpy as np
from utils.Node import Node
from algorithms.IRRT import IRRT
from algorithms.RRTConnect import RRTConnect
from algorithms.RRTStar import RRTStar
from algorithms.RRT import RRT
from algorithms.BiDAfpRRTStar import OneWayAPFRRT
from utils.Obstacle import Obstacle
from utils.Environment import Environment
from utils.Utilize import Utilize
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

def plotTriangle(ax, vertices):
    """Plot a triangle given its vertices."""
    """This triangle is a UAV"""
    triangle = plt.Polygon(vertices, closed=True, edgecolor='black', facecolor='blue')
    ax.add_patch(triangle)
    return triangle

def plotRectangle(ax, clouds):
    cloudsPoisition = []
    cloudsPyp = []
    for cloud in clouds:
        if cloud.obsType == 1 and cloud.moveType == 1:
            width = cloud.pointStart[0] - cloud.pointEnd[0]
            height = cloud.pointEnd[1] - cloud.pointStart[1]
            cloudsPoisition.append([cloud.pointEnd[0], cloud.pointStart[1]])
            cloudsPyp.append([patches.Rectangle((cloud.pointEnd[0], cloud.pointStart[1]), width=width, height= height, edgecolor='gray', facecolor='gray'),cloud.moveType])
            ax.add_patch(cloudsPyp[-1][0])
        elif cloud.obsType == 0 and cloud.moveType == 1:
            cloudsPoisition.append([cloud.pointStart[0], cloud.pointStart[1]])
            cloudsPyp.append([patches.Circle((cloud.pointStart[0], cloud.pointStart[1]), cloud.pointEnd),cloud.moveType])
            ax.add_patch(cloudsPyp[-1][0])
        elif cloud.obsType == 1 and cloud.moveType == 0:
            width = cloud.pointStart[0] - cloud.pointEnd[0]
            height = cloud.pointEnd[1] - cloud.pointStart[1]
            cloudsPoisition.append([cloud.pointEnd[0], cloud.pointStart[1]])
            cloudsPyp.append([patches.Rectangle((cloud.pointEnd[0], cloud.pointStart[1]), width=width, height= height, edgecolor='black', facecolor='black'),cloud.moveType])
            ax.add_patch(cloudsPyp[-1][0])
            #ax.add_patch(patches.Rectangle((cloud.pointEnd[0], cloud.pointStart[1]), width=width, height= height, color='black'))


    return cloudsPyp, cloudsPoisition

def plotUpdate(frame, triangle, clouds, uavPosition, cloudPosition):
    """Update the position of the triangle."""
    global uavX, uavY, cloudX, cloudY
    new_uav_position = updatePositionUAV(uavPosition, uavX, uavY)
    triangle.set_xy(new_uav_position)
    for i,c in enumerate(cloudPosition):
        new_cloud_position = updatePositionCloud(c, cloudX, cloudY)
        if isinstance(clouds[i][0], patches.Rectangle) and  clouds[i][1] == 1:
            clouds[i][0].set_xy(new_cloud_position)
        elif isinstance(clouds[i][0], patches.Circle) and  clouds[i][1] == 1:
            clouds[i][0].set_center(new_cloud_position)
    return triangle,

def updatePositionUAV(uavPosition, uavX, uavY):
    """Move the triangle by a certain distance dx and dy."""
    return uavPosition + np.array([uavX, uavY])

def updatePositionCloud(cloudPosition, cloudX, cloudY):
    """Move the rectangle by a certain distance dx and dy."""
    #realCloud.pointStart= realCloud.pointStart + np.array([cloudX , cloudY])
    #realCloud.pointEnd  = realCloud.pointEnd + np.array([cloudX , cloudY])
    return cloudPosition + np.array([cloudX , cloudY])

def moveUAV(uavPosition):
    global path, uavX, uavY, pathIndex, amoungOfTime, pauseDrone
    movementTime = time.time()
    while True:
        time.sleep(amoungOfTime)
        if not pauseDrone:
            xS = round(path[pathIndex].x,2)
            xE = round(path[pathIndex+1].x,2)
            yS = round(path[pathIndex].y,2)
            yE = round(path[pathIndex+1].y,2)
            m = (yE - yS) / (xE - xS)
            uavX += np.abs(1 / np.sqrt(1 + m**2)) * ((xE - xS) / np.abs(xE - xS))
            uavY += np.abs(m / np.sqrt(1 + m**2)) * ((yE - yS) / np.abs(yE - yS))
            if round(uavX, 2) == xE and round(uavY, 2) == yE:
                pathIndex += 1
                if pathIndex == len(path) - 1:
                    print(time.time() - movementTime)
                    break
            elif np.abs(uavX) < np.abs(path[pathIndex + 1].x) * 1.02 and np.abs(uavX) > np.abs(path[pathIndex + 1].x) *0.98 and np.abs(uavY) < np.abs(path[pathIndex + 1].y) * 1.02 and np.abs(uavY) > np.abs(path[pathIndex + 1].y) *0.98:
                uavX = path[pathIndex + 1].x
                uavY = path[pathIndex + 1].y
                pathIndex += 1
                if pathIndex == len(path) - 1:
                    print(time.time() - movementTime)
                    break
        
def moveClouds():
    global cloudX, cloudY, pathIndex, cloudSpeed
    while True:
        time.sleep(0.3)
        cloudX += 1.0
        cloudY = 0.0            
        if pathIndex == len(path) - 1:
                break

def chcekEviroment(clouds):
    global path, pathIndex, amoungOfTime,goalNode,momentOfBreak,lowLimit,maxLimit, pauseDrone
    #colorList = ['tab:gray','b','g','c','m','y','k','moccasin', 'papayawhip','lanchedalmond','navalowhite','tan','darkorange','coral','rosybrown','lightcoral','indianred','brown','maroon'] 
    colorList = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
    colorNumber = 0
    utz = Utilize()
    obsList = [row[0] for row in clouds]
    while True:
        time.sleep(0.1)
        exPath = path.copy()
        exPath[0].x = uavX
        exPath[0].y = uavY 
        for index in range(pathIndex, pathIndex+2):
            try:
                if utz.checkCollision2(nearestNode=exPath[index + 1], newNode=exPath[index], stepSize=15, obstacleList=obsList):#cloudsPosition) :
                    print("Collision detected")
                    amoungOfTime = 0.1 # 1.0 
                    pauseDrone = True
                    pathIndex = 0
                    realClouds = []
                    for c in clouds:
                        if isinstance(c[0], patches.Rectangle):
                            realClouds.append(Obstacle(np.array([c[0]._x0 + c[0]._width, c[0]._y0]), np.array([c[0]._x0,c[0]._y0 + c[0]._height]), obsType=1, moveType=1))
                        elif isinstance(c[0], patches.Circle):
                            realClouds.append(Obstacle(np.array([c[0].center[0], c[0].center[1]]),c[0].radius, obsType=0, moveType=1))
                    alg3 = IRRT(stepSize=15, iteration=30000, startPoint= Node(np.array([uavX,uavY])), targetPoint= goalNode, 
                            obstacleList= realClouds, momentOfBreak=momentOfBreak, lowLimit=lowLimit, maxLimit=maxLimit)
                    path = []
                    path, nodeCount3, completedTime3 = alg3.run()
                    print('İşlem Zamani : %f - Yol Uzunluğu : %f - Manevra Sayısı : %d' % (completedTime3, utz.calculatePathDistance(path=path), len(path)-1))
                    for i in range(len(path)-1):
                        plt.plot([path[i].x, path[i+1].x], [path[i].y, path[i+1].y], color = colorList[colorNumber])
                    #plt.savefig('images/' + str(colorNumber) +'_Path.png', dpi=300, bbox_inches='tight')
                    amoungOfTime = 0.1
                    pauseDrone = False
                    colorNumber += 1
                    if colorNumber >=10:
                        colorNumber =0
            except:
                continue
        if pathIndex == len(path) - 1:
                break

def main():
    global path, pathIndex
    """ To determined the start and goal points """
    uavPosition = np.array([[-6.0, -6.0], [0 + 6.0, 0 + -6.0], [0 + 0, 0 + 6.0]])

    """ To determinded Plot setting"""
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(-200, 200)
    ax.set_ylim(-100, 350)
    ax.set_aspect('equal')

    """ Initial plot of the triangle """
    clouds, cloudsPosition = plotRectangle(ax, cloudList)
    triangle = plotTriangle(ax, uavPosition)
    circle = patches.Circle((goalNode.x, goalNode.y),2.5, color='green')
    ax.add_patch(circle)

    for i in range(len(path)-1):
        plt.plot([path[i].x, path[i+1].x], [path[i].y, path[i+1].y], 'r-')
    #plt.grid()
    #plt.savefig('images/' +'StartPath.png', dpi=300, bbox_inches='tight')
    """ Create an animation"""
    ani = FuncAnimation(fig, plotUpdate, frames=100, fargs=(triangle, clouds, uavPosition, cloudsPosition), interval=100)

    """ To determine Thread for control uav and clouds """
    droneController = threading.Thread(target=moveUAV, args=(uavPosition,))
    cloudController = threading.Thread(target=moveClouds)
    terminalController = threading.Thread(target=chcekEviroment, args=(clouds,))

    droneController.start()
    cloudController.start()
    terminalController.start()
    plt.show()
""" To determined the Space Boundary"""
lowLimit = np.array([-200,-100])
maxLimit = np.array([200,400])
momentOfBreak = 2.0
pathIndex = 0
path = []
uavX, uavY = 0.0, 0.0
cloudX, cloudY = 0.0, 0.0
amoungOfTime = 0.1
pauseDrone = False
envName, startNode, goalNode, cloudList = Environment(pointStart=None, pointEnd=None).list7()

#### Drone Hareketlerini izlemek için sahanı oluşturulması

'''fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, aspect='equal')
plt.scatter(startNode.x, startNode.y, color='green', marker='o')
plt.scatter(goalNode.x, goalNode.y, color='red', marker='x')
for obsticle in cloudList:
    width = obsticle.pointStart[0] - obsticle.pointEnd[0]
    height = obsticle.pointEnd[1] - obsticle.pointStart[1]
    square = patches.Rectangle((obsticle.pointEnd[0], obsticle.pointStart[1]), width=width, height= height, edgecolor='black', facecolor='black')
    ax.add_patch(square)
'''
#### Gösterim Tamamlandı. 

alg3 = IRRT(stepSize=15, iteration=30000, startPoint= startNode, targetPoint= goalNode, 
            obstacleList= cloudList, momentOfBreak=momentOfBreak, lowLimit=lowLimit, maxLimit=maxLimit)
path, nodeCount3, completedTime3 = alg3.run()

print('İşlem Zamanı: %f - Yol Uzunluğu: %f - Manevra Sayısı: %d'% (completedTime3, Utilize().calculatePathDistance(path=path), len(path)-1))
uavX, uavY = path[0].x, path[0].y
alg3 = None
if __name__ == "__main__":
    main()