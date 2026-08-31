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

class SwathGenerator:
    def __init__(self, cov_width: float):
        self.cov_width = cov_width
        self.sg_bf = f2c.SG_BruteForce()

    def generate(self, no_headland: f2c.Cells, angle: float = None) -> f2c.Swaths:
        all_swaths = f2c.Swaths()
        for i in range(no_headland.size()):
            cell_geom = no_headland.getGeometry(i)
            # Tentukan sudut sapuan optimal untuk tiap sub-cell (sesuai arah lorong terpanjang)
            if angle is None or angle < 0:
                ext = cell_geom.getExteriorRing()
                xs = [ext.getGeometry(k).getX() for k in range(ext.size())]
                ys = [ext.getGeometry(k).getY() for k in range(ext.size())]
                w = max(xs) - min(xs) if xs else 1.0
                h = max(ys) - min(ys) if ys else 1.0
                eff_angle = 0.0 if w > h else 0.5 * math.pi
            else:
                eff_angle = angle

            swaths_i = self.sg_bf.generateSwaths(eff_angle, self.cov_width, cell_geom)
            for j in range(swaths_i.size()):
                all_swaths.push_back(swaths_i[j])
        return all_swaths