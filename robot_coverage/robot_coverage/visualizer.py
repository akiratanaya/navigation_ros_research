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

    def create_cells_markers(self, cells: f2c.Cells) -> MarkerArray:
        """Visualisasi garis batas dan label nama setiap sub-sel hasil dekomposisi di RViz."""
        marker_array = MarkerArray()
        colors = [
            ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.85),  # Orange
            ColorRGBA(r=0.2, g=0.9, b=0.2, a=0.85),  # Hijau Terang
            ColorRGBA(r=0.9, g=0.2, b=0.9, a=0.85),  # Magenta
            ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.85),  # Biru Langit
            ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.85),  # Kuning
            ColorRGBA(r=0.0, g=1.0, b=0.8, a=0.85),  # Cyan
        ]

        for i in range(cells.size()):
            c = cells.getGeometry(i)
            ring = c.getGeometry(0)
            num_pts = ring.size()
            if num_pts == 0:
                continue

            line = Marker()
            line.header.frame_id = self.frame_id
            line.ns = "decomposed_cell_borders"
            line.id = i
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.04  # Garis border sub-sel
            line.color = colors[i % len(colors)]

            cx, cy = 0.0, 0.0
            for j in range(num_pts):
                pt = ring.getGeometry(j)
                p = Point()
                p.x = pt.getX()
                p.y = pt.getY()
                p.z = 0.01
                line.points.append(p)
                cx += pt.getX()
                cy += pt.getY()

            # Tutup loop poligon
            first_pt = ring.getGeometry(0)
            p0 = Point()
            p0.x = first_pt.getX()
            p0.y = first_pt.getY()
            p0.z = 0.01
            line.points.append(p0)
            cx /= num_pts
            cy /= num_pts

            marker_array.markers.append(line)

            # Label teks
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns = "decomposed_cell_labels"
            text.id = 100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.15
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.pose.position.x = cx
            text.pose.position.y = cy
            text.pose.position.z = 0.08
            text.text = f"Region {i} ({c.area():.2f}m²)"
            marker_array.markers.append(text)

        return marker_array