import fields2cover as f2c

class PathGenerator:
    def __init__(self, turning_radius: float, curve_type: str = "dubins"):
        self.turning_radius = turning_radius
        self.curve_type = curve_type.lower()
        self.planner = f2c.PP_PathPlanning()

    def generate(self, robot: f2c.Robot, swaths: f2c.Swaths) -> f2c.Path:
        robot.setMinTurningRadius(self.turning_radius)
        
        if self.curve_type == "dubins":
            curve = f2c.PP_DubinsCurves()
        elif self.curve_type == "reeds_shepp":
            curve = f2c.PP_ReedsSheppCurves()
        else:
            raise ValueError(f"Unknown curve type: {self.curve_type}")

        return self.planner.planPath(robot, swaths, curve)