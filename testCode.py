import numpy as np
import pandas as pd
from algorithms.IRRT import IRRT
from algorithms.RRT import RRT
from algorithms.RRTStar import RRTStar
#from algorithms.RRTBiDirection import RRTBiDirection
from algorithms.RRTConnect import RRTConnect
from algorithms.AFPRRT import APFRRT
from algorithms.BiDAfpRRTStar import OneWayAPFRRT
from utils.Environment import Environment
import matplotlib.pyplot as plt
import matplotlib.patches as patches   
# Define the start and goal points
loopNumber = 1
env = Environment(pointStart= None, pointEnd= None)
name, start, target, obsList = env.list8()
df = pd.DataFrame(data=np.zeros((loopNumber, 18)), columns=['RRT-Node','RRTStar-Node','IRRT-Node', 'RRTConnect-Node','AFPRRT-Node', 'BiAFPRRT-Node','RRT-Time','RRTStar-Time','IRRT-Time', 'RRTConnect-Time','AFPRRT-Time', 'BiAFPRRT-Time','RRT-Dist','RRTStar-Dist','IRRT-Dist', 'RRTConnect-Dist','AFPRRT-Dist', 'BiAFPRRT-Dist'])
lowLimit = np.array([-150,-150])
maxLimit = np.array([150,150])
momentOfBreak = 2.0
pathDist = 0
for _ in range(loopNumber):
    
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, aspect='equal')
    for obsticle in obsList:
        if obsticle.obsType ==1:
            width = obsticle.pointStart[0] - obsticle.pointEnd[0]
            height = obsticle.pointEnd[1] - obsticle.pointStart[1]
            square = patches.Rectangle((obsticle.pointEnd[0], obsticle.pointStart[1]), width=width, height= height, edgecolor='black', facecolor='black')
            ax.add_patch(square)
        else:
            circle = patches.Circle((obsticle.pointStart[0], obsticle.pointStart[1]), obsticle.pointEnd)
            ax.add_patch(circle)
    '''
    alg1 = RRT(stepSize=15, iteration=18000, startPoint= start, targetPoint= target, obstacleList= obsList, momentOfBreak=momentOfBreak, lowLimit=lowLimit, maxLimit=maxLimit)
    path1, df.loc[_, 'RRT-Node'], df.loc[_, 'RRT-Time'] = alg1.run()
    print('RRT tamamlandi')

    alg2 = RRTStar(stepSize=15, iteration=5000, startPoint= start, targetPoint= target, obstacleList= obsList, momentOfBreak=momentOfBreak, lowLimit=lowLimit, maxLimit=maxLimit)
    path2, df.loc[_,'RRTStar-Node'], df.loc[_,'RRTStar-Time'] = alg2.run()
    print('RRTStar tamamlandi')
    '''
    alg3 = IRRT(stepSize=15, iteration=18000, startPoint= start, targetPoint= target, obstacleList= obsList, momentOfBreak=momentOfBreak, lowLimit=lowLimit, maxLimit=maxLimit)
    path3, df.loc[_,'IRRT-Node'], df.loc[_,'IRRT-Time'] = alg3.run()
    print('IRRT tamamlandi')
    '''
    alg4 = RRTConnect(stepSize=15, iteration=5000, startPoint= start, targetPoint= target, obstacleList= obsList, momentOfBreak=momentOfBreak, lowLimit=lowLimit, maxLimit=maxLimit)
    path4, df.loc[_,'RRTConnect-Node'], df.loc[_,'RRTConnect-Time'] = alg4.run()
    print('RRTConnect tamamlandi')

    alg5 = APFRRT(stepSize = 15, iteration = 8000, startPoint = start, targetPoint = target, obstacleList = obsList, momentOfBreak = momentOfBreak, lowLimit = lowLimit, maxLimit = maxLimit, targetBias = 0.1)
    path5, df.loc[_,'AFPRRT-Node'], df.loc[_,'AFPRRT-Time'] = alg5.run()
    print('AFPRRT tamamlandi')

    alg6 = BiDirectionApfRRTStar(stepSize = 15, iteration = 18000, startPoint = start, targetPoint = target, obstacleList = obsList, momentOfBreak = momentOfBreak, lowLimit = lowLimit, maxLimit = maxLimit, targetBias = 0.1)
    path6, df.loc[_,'BiAFPRRT-Node'], df.loc[_,'BiAFPRRT-Time'] = alg6.run()
    print('BiAFPRRT tamamlandi')

    alg6 = OneWayAPFRRT(stepSize = 15, iteration = 18000, startPoint = start, targetPoint = target, obstacleList = obsList, momentOfBreak = momentOfBreak, lowLimit = lowLimit, maxLimit = maxLimit, searchRadius=2.0, ka=1.0, kr=100.0, p0=15.0)
    path6, df.loc[_,'BiAFPRRT-Node'], df.loc[_,'BiAFPRRT-Time'] = alg6.run()
    print('BiAFPRRT tamamlandi')

    pathDist = 0
    for i in range(len(path1)-1):
        pathDist += alg1.distance(path1[i], path1[i+1])
        plt.plot([path1[i].x, path1[i+1].x], [path1[i].y, path1[i+1].y], 'g-')
    df.loc[_,'RRT-Dist'] = pathDist
    print(str(_),'-RRT    \t| Node:',str(df['RRT-Node'][_]), '\t\t| Time:', str(df['RRT-Time'][_]), '\t\t| Dist:', str(pathDist))
    if path1:
        plt.plot([path1[0].x, path1[1].x], [path1[0].y, path1[1].y], 'g-', label='RRT')

    pathDist = 0
    for i in range(len(path2)-1):
        pathDist += alg2.distance(path2[i], path2[i+1])
        plt.plot([path2[i].x, path2[i+1].x], [path2[i].y, path2[i+1].y], 'b-')
    df.loc[_,'RRTStar-Dist'] = pathDist
    print(str(_),'-RRTStar\t| Node:',str(df['RRTStar-Node'][_]), '\t\t| Time:', str(df['RRTStar-Time'][_]), '\t\t| Dist:', str(pathDist))
    if path2:
        plt.plot([path2[0].x, path2[1].x], [path2[0].y, path2[1].y], 'b-', label='RRT Star')
    '''
    pathDist = 0
    for i in range(len(path3)-1):
        pathDist += alg3.distance(path3[i], path3[i+1])
        plt.plot([path3[i].x, path3[i+1].x], [path3[i].y, path3[i+1].y], 'r-')
    df.loc[_,'IRRT-Dist'] = pathDist
    print(str(_),'-IRRT   \t| Node:',str(df['IRRT-Node'][_]), '\t\t| Time:', str(df['IRRT-Time'][_]), '\t\t| Dist:', str(pathDist))
    if path3:
        plt.plot([path3[0].x, path3[1].x], [path3[0].y, path3[1].y], 'r-', label='IRRT')
    '''
    pathDist = 0
    for i in range(len(path4)-1):
        pathDist += alg4.distance(path4[i], path4[i+1])
        plt.plot([path4[i].x, path4[i+1].x], [path4[i].y, path4[i+1].y], 'y-')
    df.loc[_,'RRTConnect-Dist'] = pathDist
    print(str(_),'-RRTConnect\t| Node:',str(df['RRTConnect-Node'][_]), '\t\t| Time:', str(df['RRTConnect-Time'][_]), '\t\t| Dist:', str(pathDist))
    if path4:
        plt.plot([path4[0].x, path4[1].x], [path4[0].y, path4[1].y], 'y-', label='RRT Connect')

    pathDist = 0
    for i in range(len(path5)-1):
        pathDist += alg5.distance(path5[i], path5[i+1])
        plt.plot([path5[i].x, path5[i+1].x], [path5[i].y, path5[i+1].y], 'c-')
    df.loc[_,'AFPRRT-Dist'] = pathDist
    print(str(_), '-APFRRT\t| Node:', str(df['AFPRRT-Node'][_]), '\t\t| Time:',str(df['AFPRRT-Time'][_]), '\t\t| Dist:', str(pathDist))
    if path5:
        plt.plot([path5[0].x, path5[1].x], [path5[0].y, path5[1].y], 'c-', label='AFPRRT')

    pathDist = 0
    for i in range(len(path6)-1):
        pathDist += alg6.distance(path6[i], path6[i+1])
        plt.plot([path6[i].x, path6[i+1].x], [path6[i].y, path6[i+1].y], 'r-')
    df.loc[_,'BiAFPRRT-Dist'] = pathDist
    print(str(_),'-BiAFPRRT   \t| Node:',str(df['BiAFPRRT-Node'][_]), '\t\t| Time:', str(df['BiAFPRRT-Time'][_]), '\t\t| Dist:', str(pathDist))
    if path6:
        plt.plot([path6[0].x, path6[1].x], [path6[0].y, path6[1].y], 'r-', label='BiAFPRRT')

    pathDist = 0
    for i in range(len(path6)-1):
        pathDist += alg6.distance(path6[i], path6[i+1])
        plt.plot([path6[i].x, path6[i+1].x], [path6[i].y, path6[i+1].y], 'orange')
    df.loc[_,'BiAFPRRT-Dist'] = pathDist
    print(str(_),'-BiAFPRRT   \t| Node:',str(df['BiAFPRRT-Node'][_]), '\t\t| Time:', str(df['BiAFPRRT-Time'][_]), '\t\t| Dist:', str(pathDist))
    if len(path6)>0:
        plt.plot([path6[0].x, path6[1].x], [path6[0].y, path6[1].y], 'orange', label='BiAFPRRT')
    '''
    plt.scatter(start.x, start.y, color='green', marker='o')
    plt.text(start.x, start.y, 'Başlangıç', ha='center')
    plt.scatter(target.x, target.y, color='red', marker='x')
    plt.text(target.x, target.y, 'Hedef', ha='center')
    plt.legend()
    plt.xlabel('X Ekseni')
    plt.ylabel('Y Ekseni')
print(f'===================={name}====================')
print(str(loopNumber),'Ortalama RRT        \t| Node:',str(df['RRT-Node'].mean()), '\t| Time:', str(df['RRT-Time'].mean()), '\t| Dist:', str(df['RRT-Dist'].mean()))
print(str(loopNumber),'Ortalama RRTStar    \t| Node:',str(df['RRTStar-Node'].mean()), '\t| Time:', str(df['RRTStar-Time'].mean()), '\t| Dist:', str(df['RRTStar-Dist'].mean()))
print(str(loopNumber),'Ortalama IRRT       \t| Node:',str(df['IRRT-Node'].mean()), '\t| Time:', str(df['IRRT-Time'].mean()), '\t| Dist:', str(df['IRRT-Dist'].mean()))
print(str(loopNumber),'Ortalama RRTConnect \t| Node:',str(df['RRTConnect-Node'].mean()), '\t| Time:', str(df['RRTConnect-Time'].mean()), '\t| Dist:', str(df['RRTConnect-Dist'].mean()))
print(str(loopNumber),'Ortalama APFRRT     \t| Node:',str(df['AFPRRT-Node'].mean()), '\t| Time:', str(df['AFPRRT-Time'].mean()), '\t| Dist:', str(df['AFPRRT-Dist'].mean()))
print(str(loopNumber),'Ortalama BiAFPRRT     \t| Node:',str(df['BiAFPRRT-Node'].mean()), '\t| Time:', str(df['BiAFPRRT-Time'].mean()), '\t| Dist:', str(df['BiAFPRRT-Dist'].mean()))

print(str(loopNumber),'Standart Sap RRT        \t| Node:',str(df['RRT-Node'].std()), '\t| Time:', str(df['RRT-Time'].std()), '\t| Dist:', str(df['RRT-Dist'].std()))
print(str(loopNumber),'Standart Sap RRTStar    \t| Node:',str(df['RRTStar-Node'].std()), '\t| Time:', str(df['RRTStar-Time'].std()), '\t| Dist:', str(df['RRTStar-Dist'].std()))
print(str(loopNumber),'Standart Sap IRRT       \t| Node:',str(df['IRRT-Node'].std()), '\t| Time:', str(df['IRRT-Time'].std()), '\t| Dist:', str(df['IRRT-Dist'].std()))
print(str(loopNumber),'Standart Sap RRTConnect \t| Node:',str(df['RRTConnect-Node'].std()), '\t| Time:', str(df['RRTConnect-Time'].std()), '\t| Dist:', str(df['RRTConnect-Dist'].std()))
print(str(loopNumber),'Standart Sap APFRRT     \t| Node:',str(df['AFPRRT-Node'].std()), '\t| Time:', str(df['AFPRRT-Time'].std()), '\t| Dist:', str(df['AFPRRT-Dist'].std()))
print(str(loopNumber),'Standart Sap BiAFPRRT     \t| Node:',str(df['BiAFPRRT-Node'].std()), '\t| Time:', str(df['BiAFPRRT-Time'].std()), '\t| Dist:', str(df['BiAFPRRT-Dist'].std()))

#df.to_excel("output.xlsx", index=False)  
#plt.plot(x, y1, label='Line 1')

#plt.title('Distance :' + str(pathDist) + " TreeNode :" + str(3))
plt.savefig(name + '.png', dpi=300, bbox_inches='tight')
plt.show()