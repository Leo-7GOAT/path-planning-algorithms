"""
PRM (Probabilistic Roadmap) Path Planner

A multi-query sampling-based planner that works in two phases:
  1. Learning phase: randomly sample collision-free configurations and
     connect nearby samples to build a roadmap graph.
  2. Query phase: connect start/goal to the roadmap and search for the
     shortest path using Dijkstra's algorithm.

Advantages:
  - Efficient for multiple queries in the same environment
  - Good coverage of configuration space
  - Can capture narrow passages with enough samples

Time complexity: O(n^2) for roadmap construction, O(n log n) for query
"""

import heapq
import math
import random
import time
from collections import defaultdict

from common import ObstacleMap, PlannerConfig, PlanningResult, calc_path_length


class PRMPlanner:
    name = "PRM"

    def __init__(
        self,
        config: PlannerConfig = None,
        n_samples: int = 600,
        n_neighbors: int = 10,
        seed: int = 42,
    ):
        self.config = config or PlannerConfig()
        self.n_samples = n_samples
        self.n_neighbors = n_neighbors
        self.seed = seed

    def plan(self, sx, sy, gx, gy, ox, oy):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)
        rng = random.Random(self.seed)

        obs_map = ObstacleMap(ox, oy, self.config)

        # Phase 1: Sample collision-free nodes
        nodes = [(sx, sy), (gx, gy)]  # index 0 = start, 1 = goal
        while len(nodes) < self.n_samples + 2:
            x = rng.uniform(obs_map.min_x, obs_map.max_x)
            y = rng.uniform(obs_map.min_y, obs_map.max_y)
            if obs_map.is_collision_free(x, y):
                nodes.append((x, y))

        # Phase 2: Build roadmap (adjacency list)
        graph = defaultdict(list)  # node_idx -> [(neighbor_idx, cost), ...]
        n = len(nodes)

        # Pre-compute distances and connect k-nearest collision-free neighbors
        for i in range(n):
            dists = []
            for j in range(n):
                if i == j:
                    continue
                d = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                dists.append((d, j))
            dists.sort()

            for d, j in dists[: self.n_neighbors]:
                if obs_map.is_line_collision_free(
                    nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1]
                ):
                    graph[i].append((j, d))
                    graph[j].append((i, d))

        # Phase 3: Dijkstra search on roadmap
        start_idx, goal_idx = 0, 1
        cost_so_far = {start_idx: 0.0}
        came_from = {start_idx: None}
        open_set = [(0.0, start_idx)]
        explored = 0

        while open_set:
            cost, current = heapq.heappop(open_set)

            if current == goal_idx:
                path_x, path_y = [], []
                node = goal_idx
                while node is not None:
                    path_x.append(nodes[node][0])
                    path_y.append(nodes[node][1])
                    node = came_from[node]
                path_x.reverse()
                path_y.reverse()

                result.path_x = path_x
                result.path_y = path_y
                result.path_length = calc_path_length(path_x, path_y)
                result.success = True
                result.explored_nodes = n
                result.planning_time = time.perf_counter() - t0
                return result

            if cost > cost_so_far.get(current, float("inf")):
                continue

            explored += 1

            for neighbor, edge_cost in graph[current]:
                new_cost = cost + edge_cost
                if new_cost < cost_so_far.get(neighbor, float("inf")):
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (new_cost, neighbor))

        result.explored_nodes = n
        result.planning_time = time.perf_counter() - t0
        return result
