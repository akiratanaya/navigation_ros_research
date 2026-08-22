import fields2cover as f2c
class HeadlandGenerator:
    def __init__(self, robot_width: float):
        self.robot_width = robot_width
        self.hl_gen = f2c.HG_Const_gen()
    def generate(self, cells: f2c.Cells, headland_swaths: int = 3) -> f2c.Cells:
        hl_width = headland_swaths * self.robot_width
        return self.hl_gen.generateHeadlands(cells, hl_width)
