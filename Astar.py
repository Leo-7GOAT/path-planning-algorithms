"""
A* Path Planner

A* extends Dijkstra by adding a heuristic function h(n) to guide search
toward the goal. With an admissible heuristic, A* guarantees optimal paths
while exploring significantly fewer nodes than Dijkstra.

f(n) = g(n) + h(n)
  g(n): cost from start to n
  h(n): estimated cost from n to goal (Euclidean distance)

Time complexity: O(V log V) in practice (heuristic-dependent)
Space complexity: O(V)
"""

import heapq
import math
import time

from common import ObstacleMap, PlannerConfig, PlanningResult, calc_path_length

MOTIONS = [
    (1, 0, 1.0),
    (0, 1, 1.0),
    (-1, 0, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
]


class AStarPlanner:
    name = "A*"

    def __init__(self, config: PlannerConfig = None):
        self.config = config or PlannerConfig()

    @staticmethod
    def _heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, sx, sy, gx, gy, ox, oy):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)

        obs_map = ObstacleMap(ox, oy, self.config)
        start = (obs_map.calc_grid_x(sx), obs_map.calc_grid_y(sy))
        goal = (obs_map.calc_grid_x(gx), obs_map.calc_grid_y(gy))

        cost_so_far = {start: 0.0}
        came_from = {start: None}
        # priority queue: (f_score, node)
        open_set = [(self._heuristic(start, goal), start)]
        closed_set = set()
        explored = 0

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal:
                path_x, path_y = [], []
                node = goal
                while node is not None:
                    path_x.append(obs_map.calc_world_x(node[0]))
                    path_y.append(obs_map.calc_world_y(node[1]))
                    node = came_from[node]
                path_x.reverse()
                path_y.reverse()

                result.path_x = path_x
                result.path_y = path_y
                result.path_length = calc_path_length(path_x, path_y)
                result.success = True
                result.explored_nodes = explored
                result.planning_time = time.perf_counter() - t0
                return result

            g_current = cost_so_far[current]
            closed_set.add(current)
            explored += 1

            for dx, dy, move_cost in MOTIONS:
                nx, ny = current[0] + dx, current[1] + dy
                if not obs_map.is_valid(nx, ny):
                    continue
                neighbor = (nx, ny)
                new_cost = g_current + move_cost * self.config.resolution
                if new_cost < cost_so_far.get(neighbor, float("inf")):
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    f_score = new_cost + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        result.planning_time = time.perf_counter() - t0
        result.explored_nodes = explored
        return result
