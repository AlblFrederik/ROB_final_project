#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import random

import numpy as np
from copy import deepcopy

from robotics_toolbox.core import SE3, SE2
from robotics_toolbox.robots.robot_base import RobotBase
from ..utils import interpolate, distance_between_configurations


class Node:
    def __init__(self, data):
        self.data = data
        self.parent = None


class RRT:
    def __init__(self, robot: RobotBase, delta_q=0.2, p_sample_goal=0.5) -> None:
        """RRT planner for a given robot.
        Args:
            robot: robot used to sample configuration and check collisions
            delta_q: maximum distance between two configurations
            p_sample_goal: probability of sampling goal as q_rand
        """
        self.p_sample_goal = p_sample_goal
        self.robot = robot
        self.delta_q = delta_q

    def plan(
            self,
            q_start: SE2 | SE3,
            q_goal: SE2 | SE3,
            max_iterations: int = 10000,
    ):
        """RRT algorithm for motion planning."""
        assert not self.robot.set_configuration(q_start).in_collision()
        assert not self.robot.set_configuration(q_goal).in_collision()
        # todo: hw06opt implement RRT
        start_node = Node(q_start)
        goal_node = Node(q_goal)
        nodes = [start_node]
        path = []

        for i in range(max_iterations):

            q_rand = self.random_config(q_goal)  # array | SE
            q_near = self.nearest_neighbor(q_rand, nodes)  # node
            q_new = self.steer(q_near, q_rand)  # Node

            if not self.robot.set_configuration(q_new.data).in_collision():
                nodes.append(q_new)
                q_new.parent = q_near

                if self.is_goal(q_near, goal_node):
                    path = self.extract_path(q_near)
                    break

        path = self.joints_to_se(path)
        return path

    def joints_to_se(self, joint_path):
        robot_path = []
        for step in joint_path:
            config = self.robot.set_configuration(step).configuration()
            robot_path.append(config)
        return robot_path

    def steer(self, q_near: Node, q_rand):
        dist = distance_between_configurations(q_near.data, q_rand)
        if dist < self.delta_q:
            return Node(q_rand)

        direct = q_rand - q_near.data
        q_new = direct/np.linalg.norm(direct) * self.delta_q

        q_new = q_near.data + q_new

        return Node(q_new)

    def random_config(self, goal_pos):
        if random.random() < self.p_sample_goal:
            return goal_pos
        else:
            return self.robot.sample_configuration()

    @staticmethod
    def extract_path(q_new):
        path = []
        node = q_new

        while node is not None:
            path.append(node.data)
            node = node.parent

        path.reverse()
        return path

    @staticmethod
    def is_goal(q_new, goal_node):
        if distance_between_configurations(goal_node.data, q_new.data) < 1e-5:
            return True
        else:
            return False

    @staticmethod
    def nearest_neighbor(q_rand, nodes):
        min_dist = float('inf')
        nearest_node = nodes[0]

        for node in nodes:
            dist = np.linalg.norm(node.data - q_rand)
            # dist = distance_between_configurations(node.data, q_rand)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        # print("nearest node: " + str(nearest_node.data))
        if type(nearest_node.data) is not (SE2 or SE3):
            new = np.ones(len(nearest_node.data))
            for i in range(len(nearest_node.data)):
                new[i] = nearest_node.data[i]
            nearest_node.data = new
        return nearest_node

    def random_shortcut(
            self, path, max_iterations=100
    ):
        """Random shortcut algorithm that pick two points on the path randomly and tries
        to interpolate between them. If collision free interpolation exists,
        the path between selected points is replaced by the interpolation."""
        # todo: hw06opt implement random shortcut algorithm
        # for n in range(max_iterations):
        #   i, j = random
        #   c1, c2 = path(i), ptah(j)
        #   if ...i...j: check not to go against the movement direction if i > j ...
        #       path = interpolation(c1, c2)
        #       cut part and put path in
        print(path)
        for i in range(max_iterations):
            a = np.random.randint(0, high=len(path))
            b = np.random.randint(0, high=len(path))
            if a > b:
                a, b = b, a
            c1, c2 = path[a], path[b]
            temp = interpolate(c1, c2, self.delta_q)
            if temp.all() and distance_between_configurations(temp, c1) <= self.delta_q:
                pat = path[:a]
                pat.append(c1)
                pat.append(temp)
                pat = pat + path[b + 1:]
                path = pat
        out = deepcopy(path)
        return out
