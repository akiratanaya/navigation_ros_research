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


class RouteGenerator:
    def __init__(self, pattern: str = "boustrophedon", spiral_width: int = 4):
        self.pattern = pattern.lower()
        self.spiral_width = max(1, spiral_width)

        if self.pattern in ["boustrophedon", "boustro"]:
            self.rp = f2c.RP_Boustrophedon()
        elif self.pattern in ["snake"]:
            self.rp = f2c.RP_Snake()
        elif self.pattern in ["spiral"]:
            self.rp = f2c.RP_Spiral(self.spiral_width)
        else:
            self.rp = f2c.RP_RoutePlannerBase()

    def generate(self, cells: f2c.Cells, swaths: f2c.Swaths, mid_hl: f2c.Cells = None):
        if swaths.size() == 0:
            return swaths

        # Jika mid_hl tersedia, gunakan RoutePlannerBase resmi Fields2Cover untuk routing keliling obstacle!
        if mid_hl is not None and mid_hl.size() > 0:
            try:
                rp_base = f2c.RP_RoutePlannerBase()
                # Set titik awal ke titik awal swath pertama
                first_cell_swaths = swaths[0] if hasattr(swaths, '__getitem__') else swaths
                if first_cell_swaths.size() > 0:
                    first_swath = first_cell_swaths[0] if hasattr(first_cell_swaths, '__getitem__') else first_cell_swaths
                    rp_base.setStartAndEndPoint(first_swath.startPoint())
                route = rp_base.genRoute(mid_hl, swaths)
                return route
            except Exception:
                pass

        try:
            return self.rp.genSortedSwaths(swaths)
        except Exception:
            return swaths