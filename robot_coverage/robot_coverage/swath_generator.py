import fields2cover as f2c
import math

class SwathGenerator:
    def __init__(self, cov_width: float):
        self.cov_width = cov_width
        self.sg_bf = f2c.SG_BruteForce()

    def generate(self, no_headland: f2c.Cells, angle: float = math.pi) -> f2c.Swaths:
        all_swaths = f2c.Swaths()
        for i in range(no_headland.size()):
            cell_geom = no_headland.getGeometry(i)
            swaths_i = self.sg_bf.generateSwaths(angle, self.cov_width, cell_geom)
            for j in range(swaths_i.size()):
                all_swaths.push_back(swaths_i[j])
        return all_swaths