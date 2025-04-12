class Obstacle:
    def __init__(self, pointStart, pointEnd, obsType = 1, moveType = 1):
        self.pointStart = pointStart
        self.pointEnd = pointEnd
        self.obsType = obsType # 1 dikdörtgen engel - 0 dairesel engel
        self.moveType = moveType # 0 sabit engel - 1 hareketli engel
        self.obsticleList = []
