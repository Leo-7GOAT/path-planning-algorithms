"""
RRT* (Rapidly-exploring Random Tree Star) Path Planner

An asymptotically optimal extension of RRT. Key improvements over RRT:
  1. Choose parent: when adding a new node, check nearby nodes for a
     lower-cost connection instead of always connecting to the nearest.
  2. Rewire: after adding a node, rewire nearby nodes through the new
     node if it provides a shorter path.

As iterations increase, the path converges to the optimal solution.

Time complexity: O(n * log n) per iteration
"""

import math
import random
import time

from common import ObstacleMap, PlannerConfig, PlanningResult, calc_path_length


class _Node:
    __slots__ = ("x", "y", "parent", "cost")

    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost


class RRTStarPlanner:
    name = "RRT*"

    def __init__(
        self,
        config: PlannerConfig = None,
        expand_distance: float = 3.0,
        goal_sample_rate: float = 0.1,
        max_iter: int = 3000,
        search_radius: float = 10.0,
        seed: int = 42,
    ):
        self.config = config or PlannerConfig()
        self.expand_distance = expand_distance
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.seed = seed

    def _near_nodes(self, tree, node, radius):
        return [
            n for n in tree
            if math.hypot(n.x - node.x, n.y - node.y) <= radius
        ]

    def plan(self, sx, sy, gx, gy, ox, oy):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)
        rng = random.Random(self.seed)

        obs_map = ObstacleMap(ox, oy, self.config)
        start_node = _Node(sx, sy, cost=0.0)
        tree = [start_node]
        goal_threshold = self.expand_distance

        best_goal_node = None
        best_goal_cost = float("inf")

        for iteration in range(self.max_iter):
            # Sample
            if rng.random() < self.goal_sample_rate:
                sample_x, sample_y = gx, gy
            else:
                sample_x = rng.uniform(obs_map.min_x, obs_map.max_x)
                sample_y = rng.uniform(obs_map.min_y, obs_map.max_y)

            # Find nearest
            nearest = min(tree, key=lambda n: math.hypot(n.x - sample_x, n.y - sample_y))

            # Steer
            dx = sample_x - nearest.x
            dy = sample_y - nearest.y
            dist = math.hypot(dx, dy)
            if dist < 1e-6:
                continue
            if dist > self.expand_distance:
                dx = dx / dist * self.expand_distance
                dy = dy / dist * self.expand_distance

            new_x = nearest.x + dx
            new_y = nearest.y + dy

            if not obs_map.is_line_collision_free(nearest.x, nearest.y, new_x, new_y):
                continue

            new_node = _Node(new_x, new_y)

            # Choose best parent from nearby nodes
            near = self._near_nodes(tree, new_node, self.search_radius)
            best_parent = nearest
            best_cost = nearest.cost + math.hypot(new_x - nearest.x, new_y - nearest.y)

            for near_node in near:
                edge_cost = math.hypot(new_x - near_node.x, new_y - near_node.y)
                candidate_cost = near_node.cost + edge_cost
                if candidate_cost < best_cost:
                    if obs_map.is_line_collision_free(near_node.x, near_node.y, new_x, new_y):
                        best_parent = near_node
                        best_cost = candidate_cost

            new_node.parent = best_parent
            new_node.cost = best_cost
            tree.append(new_node)

            # Rewire nearby nodes through new_node
            for near_node in near:
                edge_cost = math.hypot(new_x - near_node.x, new_y - near_node.y)
                candidate_cost = new_node.cost + edge_cost
                if candidate_cost < near_node.cost:
                    if obs_map.is_line_collision_free(new_x, new_y, near_node.x, near_node.y):
                        near_node.parent = new_node
                        near_node.cost = candidate_cost

            # Check goal
            dist_to_goal = math.hypot(new_x - gx, new_y - gy)
            if dist_to_goal <= goal_threshold:
                if obs_map.is_line_collision_free(new_x, new_y, gx, gy):
                    goal_cost = new_node.cost + dist_to_goal
                    if goal_cost < best_goal_cost:
                        best_goal_node = new_node
                        best_goal_cost = goal_cost

        if best_goal_node is not None:
            path_x, path_y = [gx], [gy]
            node = best_goal_node
            while node is not None:
                path_x.append(node.x)
                path_y.append(node.y)
                node = node.parent
            path_x.reverse()
            path_y.reverse()

            result.path_x = path_x
            result.path_y = path_y
            result.path_length = calc_path_length(path_x, path_y)
            result.success = True

        result.explored_nodes = len(tree)
        result.planning_time = time.perf_counter() - t0
        return result
