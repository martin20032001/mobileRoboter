from shapely.geometry import Polygon
from shapely.affinity import rotate, translate
from IPEnvironment import CollisionChecker
from IPPerfMonitor import IPPerfMonitor

class MultiMobileRobotCollisionChecker(CollisionChecker):
    def __init__(self, num_robots: int, robot_shapes: list[Polygon], scene: dict, dofs: list[int], dim: int, limits=[[0, 22], [0, 22], [0, 360]]):
        super().__init__(scene, limits=limits)
        self.num_robots = num_robots
        self.robot_shapes = robot_shapes
        self.dofs = dofs
        self.dim = dim

    def getDim(self):
        return self.dim

    def _get_transformed_robot(self, robot_index, pos):
        x, y = pos[0], pos[1]
        theta = pos[2] if len(pos) == 3 else 0
        robot = rotate(self.robot_shapes[robot_index], theta, origin='centroid', use_radians=False)
        return translate(robot, xoff=x, yoff=y)

    @IPPerfMonitor
    def pointInCollision(self, pos):
        # "point" in this case refers to high-dimensional point in configuration space (e.g. 9-dim array if three 3dof robots are present)
        
        current_idx = 0        
        for i, dof in enumerate(self.dofs):
            robot = self._get_transformed_robot(i, pos[current_idx:current_idx+dof])
                    
            # minx, miny, maxx, maxy = robot.bounds
            # if (minx < self.limits[0][0] or maxx > self.limits[0][1] or
            #     miny < self.limits[1][0] or maxy > self.limits[1][1]):
            #     return True

            if any(obstacle.intersects(robot) for obstacle in self.scene.values()):
                return True
            
            current_idx += dof
        return False

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
