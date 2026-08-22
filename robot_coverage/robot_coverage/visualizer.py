import fields2cover as f2c
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class Visualizer:
    def __init__(self, frame_id: str = "map"):
        self.frame_id = frame_id

    def create_path_markers(self, f2c_path: f2c.Path) -> MarkerArray:
        marker_array = MarkerArray()
        
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.ns = "coverage_trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.15  # Ketebalan garis di RViz
        
        # Warna Hijau Toska
        line_marker.color = ColorRGBA(r=0.0, g=0.8, b=0.8, a=1.0)

        for i in range(f2c_path.size()):
            state = f2c_path.getState(i)
            p = Point()
            p.x = state.point.getX()
            p.y = state.point.getY()
            p.z = 0.0
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)
        return marker_array