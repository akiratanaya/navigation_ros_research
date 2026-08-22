import fields2cover as f2c

class RouteGenerator:
   def __init__(self, pattern: str = "or_tools"):
       self.pattern = pattern.lower()

   def generate(self, cells: f2c.Cells, swaths: f2c.Swaths) -> f2c.Route:
       if self.pattern == "or_tools":
           planner = f2c.RP_RoutePlannerBase()

           # Bungkus Swaths menjadi SwathsByCells untuk OR-Tools solver
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
           sorter = f2c.RP_Spiral(6)
           return sorter.genSortedSwaths(swaths)

       else:
           raise ValueError(f"Unknown route pattern: {self.pattern}")