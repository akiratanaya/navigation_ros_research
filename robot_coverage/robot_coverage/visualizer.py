import fields2cover as f2c
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class Visualizer:
    def __init__(self, frame_id: str = "map"):
        self.frame_id = frame_id

    def create_path_markers(self, path_obj, perim_count: int = 0) -> MarkerArray:
        marker_array = MarkerArray()

        if hasattr(path_obj, 'poses'):
            # nav_msgs/Path
            poses = path_obj.poses
            if not poses:
                return marker_array

            if perim_count > 0 and perim_count <= len(poses):
                # 1. Perimeter Marker (Biru Toska / Cyan)
                p_marker = Marker()
                p_marker.header.frame_id = self.frame_id
                p_marker.ns = "coverage_perimeter"
                p_marker.id = 0
                p_marker.type = Marker.LINE_STRIP
                p_marker.action = Marker.ADD
                p_marker.scale.x = 0.12
                p_marker.color = ColorRGBA(r=0.0, g=0.85, b=0.95, a=1.0)
                for p in poses[:perim_count]:
                    pt = Point()
                    pt.x = p.pose.position.x
                    pt.y = p.pose.position.y
                    pt.z = 0.015
                    p_marker.points.append(pt)
                marker_array.markers.append(p_marker)

                # 2. Infill Marker (Hijau Stabilo / Lime Green)
                i_marker = Marker()
                i_marker.header.frame_id = self.frame_id
                i_marker.ns = "coverage_infill"
                i_marker.id = 1
                i_marker.type = Marker.LINE_STRIP
                i_marker.action = Marker.ADD
                i_marker.scale.x = 0.10
                i_marker.color = ColorRGBA(r=0.2, g=0.95, b=0.2, a=1.0)
                for p in poses[perim_count:]:
                    pt = Point()
                    pt.x = p.pose.position.x
                    pt.y = p.pose.position.y
                    pt.z = 0.015
                    i_marker.points.append(pt)
                marker_array.markers.append(i_marker)
            else:
                line_marker = Marker()
                line_marker.header.frame_id = self.frame_id
                line_marker.ns = "coverage_trajectory"
                line_marker.id = 0
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.scale.x = 0.12
                line_marker.color = ColorRGBA(r=0.0, g=0.85, b=0.85, a=1.0)
                for p in poses:
                    pt = Point()
                    pt.x = p.pose.position.x
                    pt.y = p.pose.position.y
                    pt.z = 0.015
                    line_marker.points.append(pt)
                marker_array.markers.append(line_marker)
            return marker_array

        elif hasattr(path_obj, 'size'):
            # f2c.Path
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.ns = "coverage_trajectory"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.15
            line_marker.color = ColorRGBA(r=0.0, g=0.8, b=0.8, a=1.0)
            for i in range(path_obj.size()):
                state = path_obj.getState(i)
                p = Point()
                p.x = state.point.getX()
                p.y = state.point.getY()
                p.z = 0.0
                line_marker.points.append(p)
            marker_array.markers.append(line_marker)
            return marker_array
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