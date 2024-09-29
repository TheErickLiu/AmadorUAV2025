from shapely.geometry import Point, LineString, Polygon
from typing import List, Tuple
import numpy as np
import math
import json

class GeoFence:
    def __init__(self, coords: List[Tuple]):
        self.coords = coords
        self.polygon = Polygon(coords)
    
    def edges(self) -> List:
        """Return a list of geoFence edges"""
        edgelist = []
        for i in range(len(self.coords) - 1):
            edgelist.append(LineString((self.coords[i],self.coords[i + 1])))
        return edgelist

    def intersected_lines(self, line: LineString) -> List[LineString]:
        """Return a list of geoFence edges that intersect with line"""
        intersection_lines = []
        intersections = line.intersection(self.polygon)
        if intersections.geom_type == "LineString":
            return intersection_lines
        elif intersections.geom_type == "MultiLineString":
            for seg in intersections.geoms:
                intersection_lines.append(seg)
        return intersection_lines

    def get_point_mid_degree(self, i):
        line1 = LineString((self.coords[i], self.coords[i-1]))
        line2 = LineString((self.coords[i], self.coords[i+1]))
        line_degrees = [d + 360 if d < 0 else d for d in [line_degree(line1), line_degree(line2)]]
        mid_angle = line_degrees[1] + (line_degrees[0] - line_degrees[1]) / 2
        #print(line_degree(line1), line_degree(line2), line_degrees, mid_angle)
        return mid_angle

    def get_adjust_point(self, id, dist=25):
        coords = self.polygon.exterior.coords
        mid_angle = self.get_point_mid_degree(id)
        radius = dist / 364000
        (x, y) = get_point_in_radius(coords[id], mid_angle, radius)
        if self.polygon.contains(Point(x, y)):
            return (x, y)
        return get_point_in_radius(coords[id], mid_angle+180, radius)
    
    def find_polygon_edge(self, line):
        for i, edge in enumerate(self.edges()):
            if line.intersects(edge):
                return i, edge
        return -1, None
    
    def nearest_point_on_polygon(self, point):
        """Return the nearest exterior point on polygon"""
        nearest_pt, nearest_dist = None, np.inf
        nearest_id = -1
        for idx, coord in enumerate(self.coords):
            dist = Point(point).distance(Point(coord))
            if dist < nearest_dist:
                nearest_id = idx
                nearest_pt = coord
                nearest_dist = dist
        return nearest_id, nearest_pt

class Plan:

    def __init__(self, geoFence: GeoFence, wayPoints: List[Tuple]):
        self.geoFence = geoFence
        self.wayPoints = wayPoints
    
    def home_point(self, altitude=50):
        return self.wayPoints[0] + [altitude]
    
    def find_detour_point(self, line, betweenpoint, dist=25):
        # Unpack coordinates
        x1, y1 = line.coords[0]
        x2, y2 = line.coords[1]
        px, py = betweenpoint
        
        # Find vector perpendicular from line
        slope = (y2 - y1) / (x2 - x1)
        inv_slope = -1 / slope if slope != 0 else 0
        print(f"slope = {slope}, inv_slope = {inv_slope}")

        # Find direction based on the position of the point relative to the line
        cross_product = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
        #print(f"cross_product = {cross_product}")
        
        # Point is to the left of the line, adjust direction
        direction = np.arctan(inv_slope)
        direction += np.pi  # Adding 180 degrees in radians
        #print(f"direction = {direction}")

        # Convert distance in feet to latitude
        dist = dist / 364000

        # Calculate the detour point
        detour_x = px + dist * np.cos(direction)
        detour_y = py + dist * np.sin(direction)

        return Point(detour_x, detour_y)

    def adjust_outside_waypoints(self):
        """Adjust waypoints that are outside of geoFence to inside"""
        outside_points = []
        for i, pt in enumerate(self.wayPoints):
            if not self.geoFence.polygon.contains(Point(pt)):
                outside_points.append((i, pt))

        for id, pt in outside_points:
            nearest_id, _ = self.geoFence.nearest_point_on_polygon(pt)
            adjusted_point = self.geoFence.get_adjust_point(nearest_id)
            self.wayPoints[id] = adjusted_point
    
    def add_detour_points(self):
        """Add a detour point for any path between two waypoints intersects geoFence"""

        detour_points = []
        for i in range(len(self.wayPoints) - 1):
            line = LineString((self.wayPoints[i], self.wayPoints[i+1]))
            interline = self.geoFence.intersected_lines(line)
            if len(interline) > 0:
                edges = []
                for l in interline:
                    edges.append(self.geoFence.find_polygon_edge(l))

                target_point_id = edges[1][0]
                detour_point = self.geoFence.get_adjust_point(target_point_id)
                detour_points.append((i+1, detour_point))
        
        #print(detour_points)
        
        for i, d in enumerate(detour_points):
            self.wayPoints.insert(d[0] + i, d[1])

    def point_to_mission_dict(self, latitude, longitude, altitude=30.48780487804878):
        """Convert a point to a mission item for plan"""
        return {
            "AMSLAltAboveTerrain": altitude,
            "Altitude": altitude,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,
            "doJumpId": 29,
            "frame": 3,
            "params": [0, 0, 0, None, latitude, longitude, altitude],
            "type": "SimpleItem"
        }

    def adjust_points(self):
        """Adjust outside waypoints and then add detour points for intersections"""
        self.adjust_outside_waypoints()
        self.add_detour_points()

    def create_plan_dict(self) -> dict:
        """
        return the initial empty plan
        args:
        - geofence_coords: a list of (latitude, longitude)
        - start_point: [latitude, longtidue, altitude]
        """
        items = []
        for pt in self.wayPoints:
            items.append(self.point_to_mission_dict(pt[0], pt[1]))

        return {
            "fileType": "Plan",
            "version": 1,
            "geoFence": {
                "circles": [],
                "polygons": [
                    {
                        "inclusion": True,
                        "polygon": self.geoFence.coords,
                        "version": 1
                    }
                ],
                "version": 2
            },
            "rallyPoints": {
                "points": [],
                "version": 2
            },
            "groundStation": "QGroundControl",
            "mission": {
                "plannedHomePosition": self.home_point(),
                "vehicleType": 2,
                "version": 2,
                "cruiseSpeed": 15,
                "firmwareType": 12,
                "globalPlanAltitudeMode": 1,
                "hoverSpeed": 15,
                "items": items,        
            }
        }

def get_point_in_radius(point, angle, radius):
    x = point[0] + radius * math.cos(math.radians(angle))
    y = point[1] + radius * math.sin(math.radians(angle))
    return (x, y)

# get degree of an edge
def line_degree(line):
    angle = math.atan2(line.coords[1][1] - line.coords[0][1], line.coords[1][0] - line.coords[0][0])
    angle_degree = math.degrees(angle)
    return angle_degree

def read_input(in_file):
    """Read inpunt geofence and waypoints from a file"""
    with open(in_file, "r") as fh:
        header = fh.readline()
        rec = header.split(" ")
        (N, M) = int(rec[0]), int(rec[1])

        points = []
        for line in fh:
            rec = line.strip().split(" ")
            points.append([float(rec[0]), float(rec[1])])
        
        geofence_coords = points[:N]
        waypoints = points[N:N+M]

    return geofence_coords, waypoints

def shared_point(line1, line2):
        """Return the shared point of two lines"""
        if line1.coords[0] == line2.coords[1]:
            return line1.coords[0]
        if line1.coords[1] == line2.coords[0]:
            return line1.coords[1]
        return None

def main():
    input = read_input("navigate.txt")
    geofence = GeoFence(input[0])
    plan = Plan(geofence, input[1])
    plan.adjust_points()
    plandict = plan.create_plan_dict()
    #print(plan.wayPoints)
    
    with open("newnavigate.plan", "w") as outfile:
        outfile.write(json.dumps(plandict, indent=4))

if __name__ == "__main__":
    main()