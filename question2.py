from shapely.geometry import Point, LineString, Polygon
from typing import List, Tuple
import numpy as np
import math
import json
import matplotlib.pyplot as plt
from shapely.plotting import plot_polygon

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

    def find_polygon_point(self, line):
        """Find the geofence point with edges intersected by a waypoint line"""
        edges = []
        for idx, edge in enumerate(self.edges()):
            if line.intersects(edge):
                edges.append((idx, edge))

        pt_id, pt = -1, None
        if len(edges) == 2:
            pt_id, pt = edges[1][0], edges[1][1].coords[0]
        
        return pt_id, pt

    def get_point_mid_degree(self, i):
        """Finds angle of bisecting angle between lines"""
        line1 = LineString((self.coords[i], self.coords[i-1]))
        line2 = LineString((self.coords[i], self.coords[i+1]))
        line_degrees = [d + 360 if d < 0 else d for d in [line_degree(line1), line_degree(line2)]]
        mid_angle = line_degrees[1] + (line_degrees[0] - line_degrees[1]) / 2
        return mid_angle

    def get_adjust_point(self, id, dist=25):
        """Finds point 25 ft from a geofence point to adjust outside waypoint to and detour intersecting path"""
        coords = self.polygon.exterior.coords
        mid_angle = self.get_point_mid_degree(id)
        radius = dist / 364000
        (x, y) = get_point_in_radius(coords[id], mid_angle, radius)
        if self.polygon.contains(Point(x, y)):
            return (x, y)
        return get_point_in_radius(coords[id], mid_angle+180, radius)

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
            line = LineString(self.wayPoints[i:i+2])
            pt_id, pt = self.geoFence.find_polygon_point(line)
            if pt:
                detour_point = self.geoFence.get_adjust_point(pt_id)
                detour_points.append((i+1, detour_point))

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
    
    def plot(self):
        """Plots the plan & saves as png"""
        fig = plt.figure(figsize=(9, 9))
        ax = fig.add_subplot(111)
        plot_polygon(self.geoFence.polygon, ax=ax, add_points=True, color="blue")
        plot_polygon(Polygon(self.wayPoints), ax=ax, add_points=True, color="green")
        plt.savefig("nav.png")

def get_point_in_radius(point, angle, radius):
    """Gets x and y coords of an edge"""
    x = point[0] + radius * math.cos(math.radians(angle))
    y = point[1] + radius * math.sin(math.radians(angle))
    return (x, y)

def line_degree(line):
    """Get degree of an edge"""
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

def main():
    input = read_input("navigate.txt")
    geofence = GeoFence(input[0])
    plan = Plan(geofence, input[1])
    plan.adjust_points()
    plandict = plan.create_plan_dict()
    print(plan.wayPoints)
    #plan.plot()

    with open("newnavigate.plan", "w") as outfile:
        outfile.write(json.dumps(plandict, indent=4))

if __name__ == "__main__":
    main()