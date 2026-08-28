import fields2cover as f2c
from typing import Union

class RouteGenerator:
    def __init__(self, pattern: str = "or_tools", spiral_width: int = 2):
        self.pattern = pattern.lower()
        self.spiral_width = max(1, spiral_width)

    def generate(self, cells: f2c.Cells, swaths: f2c.Swaths) -> Union[f2c.Route, f2c.Swaths]:
        if self.pattern == "or_tools":
            planner = f2c.RP_RoutePlannerBase()
            # Bungkus Swaths menjadi SwathsByCells untuk OR-Tools TSP solver
            swaths_by_cells = f2c.SwathsByCells()
            swaths_by_cells.push_back(swaths)
            return planner.genRoute(cells, swaths_by_cells)

        elif self.pattern == "snake":
            sorter = f2c.RP_Snake()
            return sorter.genSortedSwaths(swaths)

        elif self.pattern == "boustrophedon":
            sorter = f2c.RP_Boustrophedon()
            return sorter.genSortedSwaths(swaths)

        elif self.pattern == "spiral":
            sorter = f2c.RP_Spiral(self.spiral_width)
            return sorter.genSortedSwaths(swaths)

        else:
            raise ValueError(
                f"Unknown route pattern: '{self.pattern}'. "
                "Pilihan yang tersedia: 'or_tools', 'snake', 'boustrophedon', 'spiral'."
            )