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
from shapely import plotting

from IPEnvironment import CollisionChecker

class AssignmentRobotCollisionChecker(CollisionChecker):
    def __init__(self, robot, scene, limits=[[-3.0, 3.0], [-3.0, 3.0], [-math.pi, math.pi]], statistic=None, fk_resolution=0.1):
        super(AssignmentRobotCollisionChecker, self).__init__(scene, limits, statistic)
        self.robot = robot
        self.dim = robot.dim
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
    
    def pointInCollision(self, pos):
        self.robot.move(pos)

        sub = {self.robot.sym_x: self.robot.x, self.robot.sym_y: self.robot.y, self.robot.sym_theta: self.robot.theta}

        for part in self.robot.parts:

            matrix = np.array(self.robot.M.subs(sub))

            start = matrix.dot(np.array([part.start[0], part.start[1], 1]))[:2]
            end   = matrix.dot(np.array([part.end[0]  , part.end[1]  , 1]))[:2]
            
            if self.segmentInCollision(start, end, part.size):
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
    
    def segmentInCollision(self, startPos, endPos, size):
        for obstacle in self.scene.values():
            if obstacle.intersects(LineString([(startPos[0], startPos[1]), (endPos[0], endPos[1])]).buffer(size)):
                return True
        return False
    
    def drawObstacles(self, ax, inWorkspace=False):
        if inWorkspace:
            for key, value in self.scene.items():
                plotting.plot_polygon(value, add_points=False, color='red', ax=ax)

class AssignmentRobotPart:
    def __init__(self, start=(0,0), rotation=0, size=[1.0, 0.25]):
        self.start = start

        self.length = size[0]
        self.rotation = rotation

        self.end = (start[0] + self.length * math.cos(rotation), start[1] + self.length * math.sin(rotation))

        self.size = size[1]
        self.children = 0

class AssignmentRobot:

    def __init__(self, part_count, x=0, y=0, theta=0, dim=2):

        self.dim = dim

        self.x = x
        self.y = y
        self.theta = theta

        self.sym_x = sp.symbols('x')
        self.sym_y = sp.symbols('y')
        self.sym_theta = sp.symbols('theta')

        self.M = sp.Matrix([[sp.cos(self.sym_theta), -sp.sin(self.sym_theta), self.sym_x],
                            [sp.sin(self.sym_theta), sp.cos(self.sym_theta) , self.sym_y],
                            [0                     , 0                      , 1]])
        
        self.parts = []
        self.parts.append(AssignmentRobotPart())

        for part_index in range(0, part_count):

            parent_part_index = random.randint(0, len(self.parts) - 1)
            new_part = self.generate_part(self.parts[parent_part_index], 0.8, math.pi)

            while self.self_colliding(new_part, self.parts[parent_part_index]):
                parent_part_index = random.randint(0, len(self.parts) - 1)
                new_part = self.generate_part(self.parts[parent_part_index], 0.8, math.pi)

            self.parts.append(new_part)

    def move(self, position):
        self.x = position[0]
        self.y = position[1]

    def visualize(self, ax):

        sub = {self.sym_x: self.x, self.sym_y: self.y, self.sym_theta: self.theta}

        for part in self.parts:

            matrix = np.array(self.M.subs(sub))

            start = matrix.dot(np.array([part.start[0], part.start[1], 1]))[:2]
            end   = matrix.dot(np.array([part.end[0]  , part.end[1]  , 1]))[:2]

            robot_part = LineString([start, end]).buffer(part.size)

            plotting.plot_polygon(robot_part, add_points=False, color='blue', ax=ax)

    def self_colliding(self, part, parent_part):

        geometry = LineString([part.start, part.end]).buffer(part.size)

        if Point(part.end).buffer(part.size).intersects(LineString([parent_part.start, parent_part.end]).buffer(parent_part.size)):
            return True

        for test_part in self.parts:

            if test_part == parent_part:
                continue

            test_geometry = LineString([test_part.start, test_part.end]).buffer(test_part.size)

            if geometry.intersects(test_geometry):
                return True
            
        return False

    def generate_part(self, base_part, scale_factor, max_rotation):

        start_factor = random.uniform(0.0, 1.0)

        part_start_x = base_part.start[0] * start_factor + base_part.end[0] * (1.0 - start_factor)
        part_start_y = base_part.start[1] * start_factor + base_part.end[1] * (1.0 - start_factor)

        part_length = base_part.length * scale_factor
        part_rotation = base_part.rotation + random.uniform(-max_rotation, max_rotation)

        part_size = base_part.size * scale_factor

        return AssignmentRobotPart((part_start_x, part_start_y), part_length, part_rotation, part_size)

class AssignmentRotatingRobot:

    def __init__(self, part_count=0, position=[0,0], theta=0, start_size=[1.0, 0.25], scale_factor=[0.4, 0.8], max_radius=1.0, dim=3):

        self.dim = dim

        self.x = position[0]
        self.y = position[1]
        self.theta = theta

        self.max_radius = max_radius

        self.sym_x = sp.symbols('x')
        self.sym_y = sp.symbols('y')
        self.sym_theta = sp.symbols('theta')

        self.M = sp.Matrix([[sp.cos(self.sym_theta), -sp.sin(self.sym_theta), self.sym_x],
                            [sp.sin(self.sym_theta), sp.cos(self.sym_theta) , self.sym_y],
                            [0                     , 0                      , 1]])
        
        self.parts = []

        base_start_factor = random.uniform(-0.5, 0.5)
        base_rotation = random.uniform(-math.pi, math.pi)

        base_length = random.uniform(scale_factor[0], scale_factor[1]) * start_size[0]
        base_size = random.uniform(scale_factor[0], scale_factor[1]) * start_size[1]

        base_start_x = base_length * math.cos(base_rotation) * base_start_factor
        base_start_y = base_length * math.sin(base_rotation) * base_start_factor

        base_end_x = base_length * math.cos(base_rotation) * (1.0 + base_start_factor)
        base_end_y = base_length * math.sin(base_rotation) * (1.0 + base_start_factor)


        base_part = self.generate_base_part(start_size, scale_factor, math.pi)

        while self.valid_base_part(base_part):
            base_part = self.generate_base_part(start_size, scale_factor, math.pi)

        self.parts.append(base_part)


        for part_index in range(0, part_count):

            parent_part_index = random.randint(0, len(self.parts) - 1)
            new_part = self.generate_part(self.parts[parent_part_index], scale_factor, math.pi)

            while self.valid_part(new_part, self.parts[parent_part_index]):
                parent_part_index = random.randint(0, len(self.parts) - 1)
                new_part = self.generate_part(self.parts[parent_part_index], scale_factor, math.pi)

            self.parts.append(new_part)

    def move(self, position):
        self.x = position[0]
        self.y = position[1]
        self.theta = position[2]

    def visualize(self, ax):

        sub = {self.sym_x: self.x, self.sym_y: self.y, self.sym_theta: self.theta}

        for part in self.parts:

            matrix = np.array(self.M.subs(sub))

            start = matrix.dot(np.array([part.start[0], part.start[1], 1]))[:2]
            end   = matrix.dot(np.array([part.end[0]  , part.end[1]  , 1]))[:2]

            robot_part = LineString([start, end]).buffer(part.size)

            plotting.plot_polygon(robot_part, add_points=False, color='blue', ax=ax)

    def valid_base_part(self, part):

        geometry = LineString([part.start, part.end]).buffer(part.size)
        
        if np.linalg.norm(part.start) + part.size > self.max_radius:
            return True

        if np.linalg.norm(part.end) + part.size > self.max_radius:
            return True

        return False

    def valid_part(self, part, parent_part):

        geometry = LineString([part.start, part.end]).buffer(part.size)

        if Point(part.end).buffer(part.size).intersects(LineString([parent_part.start, parent_part.end]).buffer(parent_part.size)):
            return True
        
        if np.linalg.norm(part.start) + part.size > self.max_radius:
            return True

        if np.linalg.norm(part.end) + part.size > self.max_radius:
            return True

        for test_part in self.parts:

            if test_part == parent_part:
                continue

            test_geometry = LineString([test_part.start, test_part.end]).buffer(test_part.size)

            if geometry.intersects(test_geometry):
                return True
            
        return False

    def generate_base_part(self, start_size, scale_factor, max_rotation):

        part_start_factor = random.uniform(-0.5, 0.5)

        part_rotation = random.uniform(-max_rotation, max_rotation)
        
        part_length = random.uniform(scale_factor[0], scale_factor[1]) * start_size[0]
        part_size = random.uniform(scale_factor[0], scale_factor[1]) * start_size[1]

        part_start_x = part_length * math.cos(part_length) * part_start_factor
        part_start_y = part_length * math.sin(part_length) * part_start_factor

        return AssignmentRobotPart((part_start_x, part_start_y), part_rotation, [part_length, part_size])

    def generate_part(self, base_part, scale_factor, max_rotation):

        part_start_factor = random.uniform(0.0, 1.0)

        part_start_x = base_part.start[0] * part_start_factor + base_part.end[0] * (1.0 - part_start_factor)
        part_start_y = base_part.start[1] * part_start_factor + base_part.end[1] * (1.0 - part_start_factor)

        part_rotation = base_part.rotation + random.uniform(-max_rotation, max_rotation)
        
        part_length = random.uniform(scale_factor[0], scale_factor[1]) * base_part.length
        part_size = random.uniform(scale_factor[0], scale_factor[1]) * base_part.size

        return AssignmentRobotPart((part_start_x, part_start_y), part_rotation, [part_length, part_size])

    """def get_transforms(self):
        ts = []
        sub = {}
        for joint in self.joints:
            sub = {**sub, **joint.get_subs()}
        for M in self.Ms:
            ts.append(np.squeeze(np.array(M.subs(sub) * sp.Matrix([0, 0, 1]))).astype(np.float32)[:2])
        return ts

    def move(self, new_thetas):
        assert len(new_thetas) == len(self.joints)
        for i in range(len(self.joints)):
            self.joints[i].move(new_thetas[i])"""

if __name__ == "__main__":
    print("yx")