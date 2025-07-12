import functools
import numpy as np
from numpy import linspace

from shapely.geometry import Polygon, Point
from shapely.affinity import rotate, translate
from shapely.prepared import prep
from shapely.plotting import plot_polygon
from shapely.strtree import STRtree

from IPEnvironment import CollisionChecker
from IPPerfMonitor import IPPerfMonitor

class MultiMobileRobotCollisionChecker(CollisionChecker):
    def __init__(self, num_robots: int, robot_shapes: list[Polygon], scene: dict, dofs: list[int], dim: int, limits=[[0, 22], [0, 22], [0, 360]]):
        super().__init__(scene, limits=limits)
        self.num_robots = num_robots
        self.robot_shapes = robot_shapes
        self.dofs = dofs
        self.dim = dim

        # Collision Checker Optimizations
        self.static_polygons = list(scene.values())
        self.static_tree = STRtree(list(self.scene.values()))
        self.prepared_static = [prep(poly) for poly in self.static_polygons]

    def getDim(self):
        return self.dim
    
    def _get_transformed_robot(self, robot_index, pos):
        x, y = pos[0], pos[1]
        theta = pos[2] if len(pos) == 3 else 0
        robot = rotate(self.robot_shapes[robot_index], theta, origin='centroid', use_radians=False)
        return translate(robot, xoff=x, yoff=y)

    def _bounds_overlap(self, bounds1, bounds2):
            """Quick AABB overlap check. bounds = (minx, miny, maxx, maxy)"""
            return not (bounds1[2] < bounds2[0] or bounds2[2] < bounds1[0] or 
                    bounds1[3] < bounds2[1] or bounds2[3] < bounds1[1])

    @IPPerfMonitor
    def pointInCollision(self, pos):
        # "point" in this case refers to high-dimensional point in configuration space (e.g. 9-dim array if three 3dof robots are present)
                
        transformed_robots = []
        idx = 0
        for i, dof in enumerate(self.dofs):
            transformed_robots.append(self._get_transformed_robot(i, pos[idx:idx+dof]))
            idx += dof
        
        # Check robot-static collisions using pre-built tree
        for robot in transformed_robots:
            candidates = self.static_tree.query(robot)
            for candidate_idx in candidates:
                # Quick AABB check before expensive intersection test
                if not self._bounds_overlap(robot.bounds, self.static_polygons[candidate_idx].bounds):
                    continue
                if self.prepared_static[candidate_idx].intersects(robot):
                    return True
        
        # Check robot-robot collisions
        for i in range(len(transformed_robots)):
            for j in range(i + 1, len(transformed_robots)):
                # Quick AABB check before expensive intersection test
                if not self._bounds_overlap(transformed_robots[i].bounds, transformed_robots[j].bounds):
                    continue
                if transformed_robots[i].intersects(transformed_robots[j]):
                    return True
        
        return False
        
    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        for alpha in linspace(0, 1, 41):
            p = [(1 - alpha) * s + alpha * e for s, e in zip(startPos, endPos)]
            if self.pointInCollision(p):
                return True
        return False

    def drawObstacles(self, ax, inWorkspace=False):
        for obstacle in self.scene.values():
            plot_polygon(obstacle, ax=ax, add_points=False, color='red')
