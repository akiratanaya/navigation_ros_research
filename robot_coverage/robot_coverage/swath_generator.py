import fields2cover as f2c
import math

class SwathGenerator:
    def __init__(self, cov_width: float):
        self.cov_width = cov_width
        self.sg_bf = f2c.SG_BruteForce()

    def generate(self, no_headland: f2c.Cells, angle: float = math.pi) -> f2c.Swaths:
        return self.sg_bf.generateSwaths(angle, self.cov_width, no_headland.getGeometry(0))

    