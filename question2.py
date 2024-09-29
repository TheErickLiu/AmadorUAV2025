from shapely.geometry import Point, LineString, Polygon
from typing import List, Tuple
import numpy as np
import math

class GeoFence:
    def __init__(self, coords: List[Tuple]):
        self.coords = coords
        self.polygon = Polygon(coords)

    def points(self) -> List[Point]:
        """Return a list of geoFence points"""
        return [Point(self.coords[i]) for i in range(len(self.coords))]
    
    def edges(self) -> List:
        """Return a list of geoFence edges"""
        edgelist = []
        for i in range(len(self.coords) - 1):
            edgelist.append(LineString((self.coords[i],self.coords[i + 1])))
        return edgelist

    def intersected_lines(self, line: LineString) -> List[LineString]:
        """Return a list of geoFence edges that intersect with line"""
        intersectlist = []
        for i in range(len(self.coords) - 1):
            if (line.intersects(LineString((self.coords[i],self.coords[i + 1])))):
                intersectlist.append(LineString((self.coords[i],self.coords[i + 1])))
        return intersectlist

    def find_target_point_of_intersections(self, line: LineString):
        """Find the exterior point on geoFence that shared by two intersection edges"""
        intersectlist = self.intersected_lines(self, line)

        if len(intersectlist) < 2:
            return None
        
        if intersectlist[0].coords[1] == intersectlist[1].coords[0]:
            return intersectlist[0].coords[1]

    def get_point_mid_degree(self, i):
        line1 = LineString((self.coords[i], self.coords[i-1]))
        line2 = LineString((self.coords[i], self.coords[i+1]))
        line_degrees = [d + 360 if d < 0 else d for d in [line_degree(line1), line_degree(line2)]]
        mid_angle = line_degrees[1] + (line_degrees[0] - line_degrees[1]) / 2
        #print(line_degree(line1), line_degree(line2), line_degrees, mid_angle)
        return mid_angle

    def get_adjust_point(self, id, dist=25):
        coords = self.polygon.exterior.coords
        mid_angle = self.get_point_mid_degree(id, coords)
        radius = dist / 364000
        (x, y) = self.get_point_in_radius(coords[id], mid_angle, radius)
        if self.polygon.contains(Point(x, y)):
            return (x, y)
        return self.get_point_in_radius(coords[id], mid_angle+180, radius)

class Plan:

    def __init__(self, geoFence: GeoFence, wayPoints: List[Tuple]):
        self.geoFence = geoFence
        self.wayPoints = wayPoints
    
    def home_point(self, altitude=50):
        return self.wayPoints[0] + [altitude]
    
    def find_detour_point(line, betweenpoint, dist=25):
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
        #print(np.cos(direction), np.sin(direction))
        detour_x = px + dist * np.cos(direction)
        detour_y = py + dist * np.sin(direction)

        return Point(detour_x, detour_y)
    
    def nearest_point_on_polygon(point, polygon):
        """Return the nearest exterior point on polygon"""
        nearest_pt, nearest_dist = None, np.inf
        coords = polygon.exterior.coords
        for idx, coord in enumerate(coords):
            dist = Point(point).distance(Point(coord))
            if dist < nearest_dist:
                nearest_pt = coord
                nearest_dist = dist
        return nearest_pt
    
    def adjust_outside_waypoints(self):
        """Adjust waypoints that are outside of geoFence to inside"""
        outside_points = []
        for i, pt in enumerate(self.wayPoints):
            if not self.geoFence.polygon.contains(Point(pt)):
                #print(f"Point {pt} is OUTSIDE geofence")
                outside_points.append((i, pt))

        #print(outside_points)
        for o in outside_points:
            opt_id, opt = outside_points[o]
            npt = self.geoFence.get_adjust_point(opt_id)

            ref_line = LineString((self.wayPoints[opt_id-1], self.wayPoints[opt_id+1]))
            adjusted_point = self.find_detour_point(ref_line, npt)
            self.wayPoints[opt_id] = adjusted_point
    
    def find_polygon_edge(line, polygon_edges):
        for edge in polygon_edges:
            if line.intersects(edge):
                return edge
        return None
    
    def shared_point(line1, line2):
        """Return the shared point of two lines"""
        if line1.coords[0] == line2.coords[1]:
            return line1.coords[0]
        if line1.coords[1] == line2.coords[0]:
            return line1.coords[1]
        return None

    def add_detour_points(self):
        """Add a detour point for any path between two waypoints intersects geoFence"""

        intersectinglines = []
        for i in range(len(self.wayPoints) - 1):
            line = LineString(self.wayPoints[i], self.wayPoints[i+1])
            if line.crosses(self.geoFence.polygon):
                #print(f"Point {pt} is OUTSIDE geofence")
                intersectinglines.append((i, line))
        
        for line in intersectinglines:
            i, l = line
            edges = self.find_polygon_edge(l, self.geoFence.edges())
            target_point = self.shared_point(edges[0], edges[1])
            detour_point = self.find_detour_point(l, target_point)
            self.wayPoints.insert(i+1, detour_point)


    def adjust_points(self):
        """Adjust outside waypoints and then add detour points for intersections"""
        pass

    def create_plan_dict(self) -> dict:
        """
        return the initial empty plan
        args:
        - geofence_coords: a list of (latitude, longitude)
        - start_point: [latitude, longtidue, altitude]
        """
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
                "items": [],        
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


def point_to_mission_dict(latitude, longitude, altitude=30.48780487804878):
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
