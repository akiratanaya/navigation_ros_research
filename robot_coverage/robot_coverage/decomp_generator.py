import os
import sys
import ctypes

f2c_so = '/home/akiratanaya/Fields2Cover/build/libFields2Cover.so'
if os.path.exists(f2c_so):
    try:
        ctypes.CDLL(f2c_so, mode=ctypes.RTLD_GLOBAL)
    except Exception:
        pass
f2c_py = '/home/akiratanaya/Fields2Cover/build/swig/python'
if f2c_py not in sys.path and os.path.exists(f2c_py):
    sys.path.insert(0, f2c_py)

import fields2cover as f2c
import math


class DecompGenerator:
    def __init__(self, method: str = "trapezoidal", split_angle: float = 0.5 * math.pi):
        self.method = method.lower()
        self.split_angle = split_angle

    def decompose(self, cells: f2c.Cells) -> f2c.Cells:
        if self.method in ["none", "false", "no"]:
            return cells

        if self.method in ["boustrophedon", "boustro"]:
            decomp = f2c.DECOMP_Boustrophedon()
            decomp.setSplitAngle(self.split_angle)
            return decomp.decompose(cells)
        else:
            # Trapezoidal decomposition menghasilkan partisi kuadran lorong yang paling rapi dan teratur di sekitar obstacle
            decomp = f2c.DECOMP_TrapezoidalDecomp()
            decomp.setSplitAngle(self.split_angle)
            return decomp.decompose(cells)