from shapely.geometry import Polygon, Point
import shapely.affinity
from IPEnvironment import CollisionChecker
from shapely import plotting

class MobileRobotCollisionChecker(CollisionChecker):
    def __init__(self, robot_shape: Polygon, scene: dict, limits=[[0, 22], [0, 22], [0, 360]]):
        super().__init__(scene, limits=limits)
        self.robot_shape = robot_shape
        self.dim = len(limits)

    def getDim(self):
        return self.dim

    def _get_transformed_robot(self, pos):
        assert len(pos) in [2, 3], "Position must be 2D (x,y) or 3D (x,y,theta)"
        x, y = pos[0], pos[1]
        theta = pos[2] if len(pos) == 3 else 0
        robot = shapely.affinity.rotate(self.robot_shape, theta, origin='centroid', use_radians=False)
        robot = shapely.affinity.translate(robot, xoff=x, yoff=y)
        return robot

    def pointInCollision(self, pos):
        robot = self._get_transformed_robot(pos)

        # --- 1. Prüfe, ob der Roboter innerhalb der Grenzen liegt ---
        minx, miny, maxx, maxy = robot.bounds
        if (minx < self.limits[0][0] or maxx > self.limits[0][1] or
            miny < self.limits[1][0] or maxy > self.limits[1][1]):
            return True  # Außerhalb der Grenzen → Kollision

        # --- 2. Prüfe auf Hinderniskollision ---
        for obstacle in self.scene.values():
            if obstacle.intersects(robot):
                return True

        return False


    def lineInCollision(self, startPos, endPos):
        import numpy as np
        p1 = np.array(startPos)
        p2 = np.array(endPos)
        k = 40
        for i in range(k + 1):
            alpha = i / k
            p = p1 + alpha * (p2 - p1)
            if self.pointInCollision(p):
                return True
        return False
    
    def drawObstacles(self, ax, inWorkspace=False):
        for key, value in self.scene.items():
            plotting.plot_polygon(value, add_points=False, color='red', ax=ax)

    