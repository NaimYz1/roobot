#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PRM Path Planning Logic - ROS1 (Melodic) Port
Pure Python logic — no ROS API used directly here.
"""

import numpy as np
import random
import math


class PRM:
    def __init__(self, start, goal, occupancy_grid,
                 num_nodes=3000, connection_radius=15, logger=None):

        self.start  = tuple(start)
        self.goal   = tuple(goal)
        self.num_nodes = num_nodes
        self.radius = connection_radius
        self.grid   = occupancy_grid
        self.logger = logger

        self.nodes  = []
        self.graph  = {}

    def log(self, msg):
        if self.logger:
            self.logger("[PRM] " + msg)

    def sample_nodes(self):
        height, width = self.grid.shape
        self.nodes    = [self.start, self.goal]

        attempts     = 0
        max_attempts = 10000

        self.log("Sampling nodes...")

        while len(self.nodes) < self.num_nodes and attempts < max_attempts:
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)

            if not self.is_in_collision(x, y):
                self.nodes.append((x, y))

            attempts += 1

        self.log("Sampled {} nodes (attempts={})".format(len(self.nodes), attempts))

    def is_in_collision(self, x, y):
        try:
            val = self.grid[int(y)][int(x)]
            return val > 50
        except IndexError:
            return True

    def distance(self, n1, n2):
        return math.hypot(n1[0] - n2[0], n1[1] - n2[1])

    def edge_intersects_obstacle(self, p1, p2):
        dist = int(self.distance(p1, p2))
        if dist == 0:
            return False

        for i in range(dist + 1):
            t = i / float(dist)
            x = int(p1[0] + (p2[0] - p1[0]) * t)
            y = int(p1[1] + (p2[1] - p1[1]) * t)
            if self.is_in_collision(x, y):
                return True

        return False

    def build_graph(self):
        self.log("Building graph...")
        self.graph = {node: [] for node in self.nodes}
        edge_count = 0

        for i, node in enumerate(self.nodes):
            for j in range(i + 1, len(self.nodes)):
                other = self.nodes[j]
                if self.distance(node, other) < self.radius:
                    if not self.edge_intersects_obstacle(node, other):
                        self.graph[node].append(other)
                        self.graph[other].append(node)
                        edge_count += 1

        self.log("Graph built with {} edges".format(edge_count))

    def a_star(self):
        self.log("Running A* search...")
        open_set = {self.start}
        came_from = {}

        g_score = {node: float('inf') for node in self.nodes}
        g_score[self.start] = 0

        f_score = {node: float('inf') for node in self.nodes}
        f_score[self.start] = self.distance(self.start, self.goal)

        while open_set:
            current = min(open_set, key=lambda node: f_score[node])

            if current == self.goal:
                self.log("Goal reached!")
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)

            for neighbor in self.graph[current]:
                tentative_g = g_score[current] + self.distance(current, neighbor)
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor]  = current
                    g_score[neighbor]    = tentative_g
                    f_score[neighbor]    = tentative_g + self.distance(neighbor, self.goal)
                    open_set.add(neighbor)

        self.log("A* failed: No path found")
        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        self.log("Path reconstructed with {} points".format(len(path)))
        return path

    def find_path(self):
        self.log("Starting PRM planning...")
        self.sample_nodes()
        self.build_graph()
        path = self.a_star()
        if not path:
            self.log("PRM FAILED to find path")
        return path
