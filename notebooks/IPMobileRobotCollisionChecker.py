from shapely.geometry import Polygon
from shapely.affinity import rotate, translate
from IPEnvironment import CollisionChecker
from IPPerfMonitor import IPPerfMonitor

class MobileRobotCollisionChecker(CollisionChecker):
    def __init__(self, robot_shape: Polygon, scene: dict, limits=[[0, 22], [0, 22], [0, 360]]):
        super().__init__(scene, limits=limits)
        self.robot_shape = robot_shape
        self.dim = len(limits)

    def getDim(self):
        return self.dim

    def _get_transformed_robot(self, pos):
        x, y = pos[0], pos[1]
        theta = pos[2] if len(pos) == 3 else 0
        robot = rotate(self.robot_shape, theta, origin='centroid', use_radians=False)
        return translate(robot, xoff=x, yoff=y)

    @IPPerfMonitor
    def pointInCollision(self, pos):
        robot = self._get_transformed_robot(pos)
        minx, miny, maxx, maxy = robot.bounds

        if (minx < self.limits[0][0] or maxx > self.limits[0][1] or
            miny < self.limits[1][0] or maxy > self.limits[1][1]):
            return True

        return any(obstacle.intersects(robot) for obstacle in self.scene.values())

    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        from numpy import linspace
        for alpha in linspace(0, 1, 41):
            p = [(1 - alpha) * s + alpha * e for s, e in zip(startPos, endPos)]
            if self.pointInCollision(p):
                return True
        return False


    def drawObstacles(self, ax, inWorkspace=False):
        from shapely.plotting import plot_polygon
        for obstacle in self.scene.values():
            plot_polygon(obstacle, ax=ax, add_points=False, color='red')
