import fields2cover as f2c
import math

class DecompGenerator:
    def __init__(self, method: str = "trapezoidal", split_angle: float = 0.5 * math.pi):
        self.method = method.lower()
        self.split_angle = split_angle

    def decompose(self, cells: f2c.Cells) -> f2c.Cells:
        if self.method == "trapezoidal":
            decomp = f2c.DECOMP_TrapezoidalDecomp()
            decomp.setSplitAngle(self.split_angle)
            return decomp.decompose(cells)
        
        elif self.method == "boustrophedon":
            # Kasus khusus: Menggunakan TrapezoidalDecomp bawaan F2C
            # karena F2C membungkus optimasi pemotongan lahan di kelas ini
            decomp = f2c.DECOMP_TrapezoidalDecomp()
            decomp.setSplitAngle(self.split_angle)
            return decomp.decompose(cells)
            
        elif self.method == "none":
            # Tanpa dekomposisi untuk lahan sederhana (persegi/konveks)
            return cells
            
        else:
            raise ValueError(f"Unknown decomposition method: {self.method}")