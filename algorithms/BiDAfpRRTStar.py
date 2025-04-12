import time
import numpy as np
from utils.Node import Node
import matplotlib.pyplot as plt
from utils.Utilize import Utilize

#Aktif Kullanılan Bi Direction Apf RRT Star
class OneWayAPFRRT:
    ultz = Utilize()
    def __init__(self, stepSize, iteration, startPoint, targetPoint, obstacleList, momentOfBreak, lowLimit, maxLimit,
                 searchRadius=2.0, ka=1.0, kr=100.0, p0=15.0):
        self.startPoint = startPoint
        self.targetPoint = targetPoint
        self.searchRadius = searchRadius
        self.obstacleList = obstacleList  # [(x, y, radius), ...]
        #self.bounds = bounds
        self.momentOfBreak = momentOfBreak
        self.lowLimit = lowLimit
        self.maxLimit = maxLimit
        self.iteration = iteration
        self.stepSize = stepSize
        self.ka = ka  # Çekim kuvveti katsayısı
        self.kr = kr  # İtme kuvveti katsayısı
        self.p0 = p0  # Güvenli mesafe eşiği
        self.vertices = [self.startPoint]

        self.start_tree = [self.startPoint]
        self.target_tree = [self.targetPoint]
        self.path = []

    def distance(self, node1, node2):
        """İki düğüm arasındaki mesafeyi hesaplar"""
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def sample(self):
        """Hedef yönelimli rastgele örnekleme."""
        if np.random.random() > 0.1:  # %80 rastgele, %20 hedefe yönelim
            x = np.random.uniform(self.lowLimit[0], self.maxLimit[0]) #(self.bounds[0], self.bounds[1])
            y = np.random.uniform(self.lowLimit[1], self.maxLimit[1]) #(self.bounds[0], self.bounds[1])
            return Node(np.array([x, y]))
        else:
            return self.targetPoint

    def nearest(self, point, tree):
        """En yakın düğümü bul."""
        distances = [self.distance(node, point) for node in tree]
        return tree[np.argmin(distances)]

    def steer(self, from_node, to_node):
        """Yönlendirme fonksiyonu."""
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        theta = np.arctan2(dy, dx)
        new_x = from_node.x + self.stepSize * np.cos(theta)
        new_y = from_node.y + self.stepSize * np.sin(theta)
        new_node = Node(np.array([new_x, new_y]))
        new_node.parent = from_node
        new_node.cost = from_node.cost + self.stepSize
        return new_node
    ############################################En Son Bakılacak###########################
    def potential_field(self, node, target):
        # Attractive force towards the goal
        ka = 1.0  # Attractive gain
        att_force = ka * np.array([target.x - node.x, target.y - node.y])

        # Repulsive force from obstacleList
        kr = 100.0  # Repulsive gain
        rep_force = np.array([0.0, 0.0])
        for obs in self.obstacleList:
            #obs_x, obs_y, obs_r = obs#
            #dist = np.sqrt((node.x - obs_x) ** 2 + (node.y - obs_y) ** 2)
            if obs.obsType == 1:
                pointStart, pointEnd = obs.pointStart, obs.pointEnd
                xMin, xMax, yMin, yMax = pointEnd[0], pointStart[0], pointStart[1], pointEnd[1]
                xClosest = np.clip(node.x, xMin, xMax)
                yClosest = np.clip(node.y, yMin, yMax)
                closetPoint = Node(np.array([xClosest, yClosest]))
                #dist = self.distance(node, closetPoint)
                dist = np.linalg.norm(np.array([node.x, node.y]) - np.array([xClosest, yClosest]))
            elif obs.obsType == 0:
                center, radius = obs.pointStart, obs.pointEnd
                dist = self.distance(node, Node(center))

            radius = self.stepSize
            if dist < radius:
                direction = np.array([node.x - closetPoint.x, node.y - closetPoint.y]) / dist
                rep_force += kr * (1.0 / dist - 1.0 / radius) * (1.0 / dist**2) * direction

        # Total force
        total_force = att_force + rep_force
        return total_force
    
    def apply_potential_field(self, node, target):
        force = self.potential_field(node, target)
        norm = np.linalg.norm(force)
        if norm > self.stepSize:
            force = (force / norm) * self.stepSize
        return Node(np.array([node.x + force[0], node.y + force[1]]))
    
    def apf(self, nearest_node, new_node):
        """Yapay potansiyel alan (APF) ile düğüm iyileştirme."""
        # Çekim kuvveti (hedefe doğru)
        F_att = np.array([self.goal.x - nearest_node.x, self.goal.y - nearest_node.y])
        F_att = self.ka * F_att / np.linalg.norm(F_att)

        # İtme kuvveti (engellerden kaçınma)
        F_rep = np.zeros(2)
        for obs in self.obstacleList:
            obs_x, obs_y, obs_r = obs
            d = np.sqrt((nearest_node.x - obs_x) ** 2 + (nearest_node.y - obs_y) ** 2)
            if d <= self.p0:
                rep_force = self.kr * (1 / d - 1 / self.p0) * (1 / d ** 2)
                direction = np.array([nearest_node.x - obs_x, nearest_node.y - obs_y]) / d
                F_rep += rep_force * direction

        # Toplam kuvvet
        F_total = F_att + F_rep
        F_total = F_total / np.linalg.norm(F_total)  # Birim vektör

        # Yeni düğüm pozisyonu
        new_x = nearest_node.x + self.step_size * F_total[0]
        new_y = nearest_node.y + self.step_size * F_total[1]
        improved_node = Node(new_x, new_y)
        improved_node.parent = nearest_node
        improved_node.cost = nearest_node.cost + self.step_size
        return improved_node

    def find_near_nodes(self, new_node, tree):
        # Yarı çap dinamik olarak hesaplayan bir kod. Duruma göre açılacak.
        numberOfNode = len(tree)
        self.searchRadius = 50 * np.sqrt((np.log(numberOfNode) / numberOfNode))
        """Belirli bir yarıçap içindeki düğümleri bulur"""
        near_nodes = []
        for node in tree:
            if self.distance(new_node, node) <= self.searchRadius:
                near_nodes.append(node)
        return near_nodes

    def rewire(self, new_node, near_nodes):
        """Yakındaki düğümleri yeniden bağlar"""
        for near_node in near_nodes:
            potential_cost = (new_node.cost +
                            self.distance(new_node, near_node))

            if (potential_cost < near_node.cost and not self.ultz.checkCollision(near_node, new_node, self.stepSize, self.obstacleList)):
                #self.is_collision_free(new_node, near_node)):############Collision Güncellenecek##################
                near_node.parent = new_node
                near_node.cost = potential_cost

    def is_collision_free(self, node1, node2):
        """Çarpışma kontrolü."""
        for obs in self.obstacleList:
            obs_x, obs_y, obs_r = obs
            for t in np.linspace(0, 1, 10):
                x = node1.x + t * (node2.x - node1.x)
                y = node1.y + t * (node2.y - node1.y)
                if np.sqrt((x - obs_x) ** 2 + (y - obs_y) ** 2) <= obs_r:
                    return False
        return True

    def connect_trees(self, tree1, tree2):
        """İki ağacı birleştirmeye çalışır"""
        for node1 in tree1:
            for node2 in tree2:
                if (self.distance(node1, node2) < self.momentOfBreak and
                    not self.ultz.checkCollision(node1, node2, self.stepSize, self.obstacleList)):
                     #self.is_collision_free(node1, node2)):############Collision Güncellenecek##################
                    return node1, node2
        return None, None
    
    def construct_path(self, connect_node1, connect_node2):
        """Bulunan yolu oluşturur"""
        path = []
        current = connect_node1
        while current is not None:
            path.append(Node(np.array([current.x, current.y])))
            current = current.parent
        path = path[::-1]
        current = connect_node2
        while current is not None:
            path.append(Node(np.array([current.x, current.y])))
            current = current.parent

        return np.array(path)

    def run(self):
        """Ana planlama fonksiyonu."""
        startTime = time.time()
        path, nodeCount = [], 1
        for _ in range(self.iteration):
            rand_node = self.sample()
            nearest_start = self.nearest(rand_node, self.start_tree)
            new_node_star = self.steer(nearest_start, rand_node)
            improved_node = self.apply_potential_field(nearest_start, rand_node) #self.apf(nearest_node, new_node)

            if not self.ultz.checkCollision(nearest_start, improved_node, self.stepSize, self.obstacleList):
              #self.is_collision_free(nearest_start, improved_node):############Collision Güncellenecek##################
              improved_node.parent = nearest_start
              near_nodes_start = self.find_near_nodes(improved_node, self.start_tree)
              self.start_tree.append(improved_node)
              self.rewire(improved_node, near_nodes_start)
              #plt.pause(0.01)
              #plt.plot([improved_node.parent.x, improved_node.x], [improved_node.parent.y, improved_node.y], 'go', linestyle='-')

              nearest_target = self.nearest(rand_node, self.target_tree)
              new_node_target = self.steer(nearest_target, rand_node)
              #???
              improved_node_target = self.apply_potential_field(nearest_target, rand_node)
              #???
              nodeCount += 1
              if not self.ultz.checkCollision(nearest_target, new_node_target, self.stepSize, self.obstacleList):
                  #self.is_collision_free(nearest_target, improved_node_target):############Collision Güncellenecek##################
                  new_node_target.parent = nearest_target
                  near_nodes_target = self.find_near_nodes(new_node_target, self.target_tree)
                  self.target_tree.append(new_node_target)
                  self.rewire(new_node_target, near_nodes_target)
                  #plt.pause(0.01)
                  #plt.plot([new_node_target.parent.x, new_node_target.x], [new_node_target.parent.y, new_node_target.y], 'go', linestyle='-')

                  nodeCount += 1

                  connectStar, connectTarget = self.connect_trees(self.start_tree, self.target_tree)
                  if connectStar and connectTarget:
                      path = self.construct_path(connectStar, connectTarget)
                      endTime = time.time()
                      return path, nodeCount, endTime - startTime

            #if self.is_collision_free(nearest_node, improved_node):
            #  improved_node.parent = nearest_node
            #  if self.distance(improved_node, self.goal) < self.step_size:
            #    self.goal.parent = improved_node
            #    return self.get_path()
        endTime = time.time()
        return path, nodeCount, endTime - startTime

    def get_path(self):
        """Bulunan yolu oluştur."""
        path = []
        current = self.goal
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]

    def plot(self, path=None):
        """Sonuçları görselleştir."""
        plt.figure(figsize=(10, 10))
        for obs in self.obstacleList:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.5)
            plt.gca().add_patch(circle)
        for node in self.vertices:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-', alpha=0.5)
        if len(path)>0:
            path = np.array(path)
            plt.plot(path[:, 0], path[:, 1], 'g-', linewidth=2)
        plt.plot(self.startPoint.x, self.startPoint.y, 'go', markersize=10, label="Start")
        plt.plot(self.targetPoint.x, self.targetPoint.y, 'ro', markersize=10, label="Goal")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()
'''
# Test
if __name__ == "__main__":
    start = Node(np.array([10, 10]))
    goal = Node(np.array([90, 90]))
    bounds = [0, 100]
    obstacleList = [
        (30, 30, 10),
        (60, 60, 15),
        (40, 80, 12),
        (80, 40, 10)
    ]

    planner = OneWayAPFRRT(stepSize=5.0, iteration=1000, startPoint=start, targetPoint=goal, obstacleList= obstacleList, momentOfBreak=2.0, lowLimit=0, maxLimit=100,
                 searchRadius=2.0, ka=1.0, kr=100.0, p0=15.0)
    (start, goal, obstacleList, bounds)
    path = planner.plan()
    if len(path) > 0:
        print("Yol bulundu!")
        planner.plot(path)
    else:
        print("Yol bulunamadı!")
'''