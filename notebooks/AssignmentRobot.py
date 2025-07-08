# coding: utf-8
"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein). It is based on the slides/notebooks given during the course, so please **read this information first**

Important remark: Kudos to Gergely Soti, who did provide this code to integrate planar robots into the lecture

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import sympy as sp
import numpy as np
import random
import math
import copy

from shapely.geometry import Point, Polygon, LineString
from shapely import plotting, affinity

from IPEnvironment import CollisionChecker

def shape_transformation(shape, translation=[0,0], rotation=0, scale=[1,1]):
    shape = affinity.scale(shape, scale[0], scale[1])
    shape = affinity.rotate(shape, rotation)
    shape = affinity.translate(shape, translation[0], translation[1])

    return shape

class AssignmentRobotCollisionChecker(CollisionChecker):
    def __init__(self, robots, scene, limits=[[-3.0, 3.0], [-3.0, 3.0], [-math.pi, math.pi], [-3.0, 3.0], [-3.0, 3.0], [-math.pi, math.pi], [-3.0, 3.0], [-3.0, 3.0], [-math.pi, math.pi]], statistic=None, fk_resolution=0.1):
        super(AssignmentRobotCollisionChecker, self).__init__(scene, limits, statistic)
        self.robots = robots
        self.dim = len(limits)
        print(self.dim)
        self.fk_resolution = fk_resolution

    def interpolate_line(self, startPos, endPos, step_l):
        steps = []
        line = np.array(endPos) - np.array(startPos)
        line_l = np.linalg.norm(line)
        step = line / line_l * step_l
        n_steps = np.floor(line_l / step_l).astype(np.int32)
        c_step = np.array(startPos)
        for i in range(n_steps):
            steps.append(copy.deepcopy(c_step))
            c_step += step
        if not (c_step == np.array(endPos)).all():
            steps.append(np.array(endPos))
        return steps

    def getDim(self):
        return self.dim
    
    def pointInCollision(self, configuration):
        index = 0
        for robot_name, robot in self.robots.items():
            transformed_shape = shape_transformation(robot, [configuration[index], configuration[index+1]], configuration[index+2] * 180 / math.pi, [1,1])
            index += 3
        
            other_index = 0
            for other_robot_name, other_robot in self.robots.items():
                other_transformed_shape = shape_transformation(other_robot, [configuration[other_index], configuration[other_index+1]], configuration[other_index+2] * 180 / math.pi, [1,1])
                other_index += 3

                if robot_name == other_robot_name:
                    continue

                if transformed_shape.intersects(other_transformed_shape):
                    return True

            if self.segmentInCollision(transformed_shape):
                return True
            
        return False
    
    def lineInCollision(self, startPos, endPos):
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        steps = self.interpolate_line(startPos, endPos, self.fk_resolution)
        for pos in steps:
            if self.pointInCollision(pos):
                return True
        return False
    
    def segmentInCollision(self, shape):
        for obstacle in self.scene.values():
            if obstacle.intersects(shape):
                return True
        return False

if __name__ == "__main__":
    print("yx")