class Obstacle:
    def __init__(self, pointStart, pointEnd, obsType = 1, moveType = 1):
        self.pointStart = pointStart
        self.pointEnd = pointEnd
        self.obsType = obsType # 1 rectangular obstacle - 0 circular obstacle
        self.moveType = moveType # 0 fixed obstacle - 1 moving obstacle
        self.obsticleList = []
