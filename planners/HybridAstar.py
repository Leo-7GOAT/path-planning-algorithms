"""
Hybrid A* Planner

A heading-aware extension of A* that searches in continuous state space
with discretized pose keys and bicycle-model motion primitives.
"""

import heapq
import math
import time

from .common import PlannerConfig, PlanningResult, ObstacleMap, calc_path_length, resample_path


class _PoseNode:
    __slots__ = ("x", "y", "yaw", "g", "parent")

    def __init__(self, x, y, yaw, g=0.0, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.g = g
        self.parent = parent


class HybridAStarPlanner:
    name = "Hybrid A*"

    def __init__(
        self,
        config: PlannerConfig = None,
        wheelbase: float = 2.7,
        max_steer_deg: float = 35.0,
        step_size: float = 2.2,
        heading_bins: int = 24,
    ):
        self.config = config or PlannerConfig()
        self.wheelbase = wheelbase
        self.max_steer = math.radians(max_steer_deg)
        self.step_size = step_size
        self.heading_bins = heading_bins
        self.steer_set = [-self.max_steer, -0.5 * self.max_steer, 0.0, 0.5 * self.max_steer, self.max_steer]

    def _key(self, obs_map, x, y, yaw):
        theta = int(round(((yaw + math.pi) % (2.0 * math.pi)) / (2.0 * math.pi / self.heading_bins))) % self.heading_bins
        return (obs_map.calc_grid_x(x), obs_map.calc_grid_y(y), theta)

    @staticmethod
    def _heuristic(x, y, gx, gy):
        return math.hypot(gx - x, gy - y)

    def _simulate(self, x, y, yaw, steer):
        n_steps = 5
        ds = self.step_size / n_steps
        points = []
        cx, cy, cyaw = x, y, yaw
        for _ in range(n_steps):
            cx += ds * math.cos(cyaw)
            cy += ds * math.sin(cyaw)
            cyaw += ds / self.wheelbase * math.tan(steer)
            points.append((cx, cy, cyaw))
        return points

    def plan(self, sx, sy, gx, gy, ox, oy, scenario=None):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)

        obs_map = ObstacleMap(ox, oy, self.config)
        start_yaw = math.atan2(gy - sy, gx - sx)
        goal_yaw = start_yaw
        if scenario is not None:
            start_yaw = scenario.get("start_yaw", start_yaw)
            goal_yaw = scenario.get("goal_yaw", goal_yaw)

        start = _PoseNode(sx, sy, start_yaw, g=0.0, parent=None)
        open_heap = [(self._heuristic(sx, sy, gx, gy), 0, start)]
        best_cost = {self._key(obs_map, sx, sy, start_yaw): 0.0}
        explored = 0
        tie = 0

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            current_key = self._key(obs_map, current.x, current.y, current.yaw)
            if current.g > best_cost.get(current_key, float("inf")):
                continue

            explored += 1
            if math.hypot(current.x - gx, current.y - gy) <= self.step_size * 1.2:
                if obs_map.is_line_collision_free(current.x, current.y, gx, gy):
                    path_x = [gx]
                    path_y = [gy]
                    node = current
                    while node is not None:
                        path_x.append(node.x)
                        path_y.append(node.y)
                        node = node.parent
                    path_x.reverse()
                    path_y.reverse()
                    path_x, path_y = resample_path(path_x, path_y, spacing=0.5)
                    result.path_x = path_x
                    result.path_y = path_y
                    result.path_length = calc_path_length(path_x, path_y)
                    result.success = True
                    result.explored_nodes = explored
                    result.planning_time = time.perf_counter() - t0
                    return result

            for steer in self.steer_set:
                rollout = self._simulate(current.x, current.y, current.yaw, steer)
                valid = True
                for px, py, _ in rollout:
                    if not obs_map.is_collision_free(px, py):
                        valid = False
                        break
                if not valid:
                    continue

                nx, ny, nyaw = rollout[-1]
                key = self._key(obs_map, nx, ny, nyaw)
                steer_cost = 0.15 * abs(steer) / max(self.max_steer, 1e-6)
                yaw_cost = 0.03 * abs((nyaw - goal_yaw + math.pi) % (2.0 * math.pi) - math.pi)
                new_g = current.g + self.step_size + steer_cost + yaw_cost
                if new_g >= best_cost.get(key, float("inf")):
                    continue

                best_cost[key] = new_g
                next_node = _PoseNode(nx, ny, nyaw, g=new_g, parent=current)
                tie += 1
                priority = new_g + self._heuristic(nx, ny, gx, gy)
                heapq.heappush(open_heap, (priority, tie, next_node))

        result.explored_nodes = explored
        result.planning_time = time.perf_counter() - t0
        return result
