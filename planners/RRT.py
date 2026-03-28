"""
RRT (Rapidly-exploring Random Tree) Path Planner

A sampling-based planner that incrementally builds a tree by randomly
sampling the configuration space and extending toward samples.

Advantages:
  - Works well in high-dimensional spaces
  - No grid discretization needed
  - Probabilistically complete

Disadvantages:
  - Not optimal (paths tend to be jagged)
  - Performance depends on sampling strategy

Time complexity: O(n * log n) per iteration with KD-tree (here O(n) with brute force)
"""

import math
import random
import time

from .common import ObstacleMap, PlannerConfig, PlanningResult, calc_path_length


class _Node:
    __slots__ = ("x", "y", "parent")

    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


class RRTPlanner:
    name = "RRT"

    def __init__(
        self,
        config: PlannerConfig = None,
        expand_distance: float = 3.0,
        goal_sample_rate: float = 0.1,
        max_iter: int = 3000,
        seed: int = 42,
    ):
        self.config = config or PlannerConfig()
        self.expand_distance = expand_distance
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.seed = seed

    def plan(self, sx, sy, gx, gy, ox, oy):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)
        rng = random.Random(self.seed)

        obs_map = ObstacleMap(ox, oy, self.config)
        start_node = _Node(sx, sy)
        goal_node = _Node(gx, gy)
        tree = [start_node]

        goal_threshold = self.expand_distance

        for _ in range(self.max_iter):
            # Sample random point (with goal bias)
            if rng.random() < self.goal_sample_rate:
                sample_x, sample_y = gx, gy
            else:
                sample_x = rng.uniform(obs_map.min_x, obs_map.max_x)
                sample_y = rng.uniform(obs_map.min_y, obs_map.max_y)

            # Find nearest node in tree
            nearest = min(tree, key=lambda n: math.hypot(n.x - sample_x, n.y - sample_y))

            # Steer toward sample
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

            # Check collision
            if not obs_map.is_line_collision_free(nearest.x, nearest.y, new_x, new_y):
                continue

            new_node = _Node(new_x, new_y, parent=nearest)
            tree.append(new_node)

            # Check if we reached the goal
            if math.hypot(new_x - gx, new_y - gy) <= goal_threshold:
                # Connect to goal
                if obs_map.is_line_collision_free(new_x, new_y, gx, gy):
                    goal_reached = _Node(gx, gy, parent=new_node)
                    tree.append(goal_reached)

                    path_x, path_y = [], []
                    node = goal_reached
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

        result.explored_nodes = len(tree)
        result.planning_time = time.perf_counter() - t0
        return result
