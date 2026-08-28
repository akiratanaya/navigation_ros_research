import fields2cover as f2c
import math

class DecompGenerator:
    def __init__(self, method: str = "boustrophedon", split_angle: float = 0.5 * math.pi):
        self.method = method.lower()
        self.split_angle = split_angle

    def decompose(self, cells: f2c.Cells) -> f2c.Cells:
        if self.method in ["trapezoidal", "trapezoid"]:
            decomp = f2c.DECOMP_TrapezoidalDecomp()
            decomp.setSplitAngle(self.split_angle)
            return decomp.decompose(cells)

        elif self.method in ["boustrophedon", "boustro"]:
            decomp = f2c.DECOMP_Boustrophedon()
            decomp.setSplitAngle(self.split_angle)
            return decomp.decompose(cells)

        elif self.method in ["none", "false", "no"]:
            # Tanpa dekomposisi untuk lahan sederhana (persegi/konveks)
            return cells

        else:
            raise ValueError(
                f"Unknown decomposition method: '{self.method}'. "
                "Pilihan yang tersedia: 'boustrophedon', 'trapezoidal', atau 'none'."
            )