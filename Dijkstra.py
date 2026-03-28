"""
Dijkstra's Algorithm Path Planner

Classic graph search algorithm that explores all directions uniformly.
Guarantees finding the shortest path but explores more nodes than A*.

Time complexity: O(V log V + E) with priority queue
Space complexity: O(V)
"""

import heapq
import math
import time

from common import ObstacleMap, PlannerConfig, PlanningResult, calc_path_length

# 8-connected grid motions: [dx, dy, cost]
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


class DijkstraPlanner:
    name = "Dijkstra"

    def __init__(self, config: PlannerConfig = None):
        self.config = config or PlannerConfig()

    def plan(self, sx, sy, gx, gy, ox, oy):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)

        obs_map = ObstacleMap(ox, oy, self.config)
        start = (obs_map.calc_grid_x(sx), obs_map.calc_grid_y(sy))
        goal = (obs_map.calc_grid_x(gx), obs_map.calc_grid_y(gy))

        # cost_so_far[node] = best known cost
        cost_so_far = {start: 0.0}
        came_from = {start: None}
        # priority queue: (cost, node)
        open_set = [(0.0, start)]
        explored = 0

        while open_set:
            cost, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
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

            if cost > cost_so_far.get(current, float("inf")):
                continue

            explored += 1

            for dx, dy, move_cost in MOTIONS:
                nx, ny = current[0] + dx, current[1] + dy
                if not obs_map.is_valid(nx, ny):
                    continue
                neighbor = (nx, ny)
                new_cost = cost + move_cost * self.config.resolution
                if new_cost < cost_so_far.get(neighbor, float("inf")):
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (new_cost, neighbor))

        result.planning_time = time.perf_counter() - t0
        result.explored_nodes = explored
        return result
