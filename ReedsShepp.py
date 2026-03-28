"""
Reeds-Shepp-style Planner

Uses a bidirectional Hybrid A* search with forward and reverse motion
primitives to emulate Reeds-Shepp-style curvature-constrained maneuvers.
"""

import heapq
import math
import time

from common import PlannerConfig, PlanningResult, ObstacleMap, calc_path_length, resample_path


class _RSNode:
    __slots__ = ("x", "y", "yaw", "g", "parent")

    def __init__(self, x, y, yaw, g=0.0, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.g = g
        self.parent = parent


class ReedsSheppPlanner:
    name = "Reeds-Shepp"

    def __init__(
        self,
        config: PlannerConfig = None,
        wheelbase: float = 2.7,
        max_steer_deg: float = 35.0,
        step_size: float = 2.0,
        heading_bins: int = 24,
    ):
        self.config = config or PlannerConfig()
        self.wheelbase = wheelbase
        self.max_steer = math.radians(max_steer_deg)
        self.step_size = step_size
        self.heading_bins = heading_bins
        self.steer_set = [-self.max_steer, 0.0, self.max_steer]
        self.directions = [1.0, -1.0]

    def _key(self, obs_map, x, y, yaw):
        theta = int(round(((yaw + math.pi) % (2.0 * math.pi)) / (2.0 * math.pi / self.heading_bins))) % self.heading_bins
        return (obs_map.calc_grid_x(x), obs_map.calc_grid_y(y), theta)

    def _simulate(self, x, y, yaw, steer, direction):
        steps = 5
        ds = self.step_size / steps
        points = []
        cx, cy, cyaw = x, y, yaw
        for _ in range(steps):
            cx += direction * ds * math.cos(cyaw)
            cy += direction * ds * math.sin(cyaw)
            cyaw += direction * ds / self.wheelbase * math.tan(steer)
            points.append((cx, cy, cyaw))
        return points

    def plan(self, sx, sy, gx, gy, ox, oy, scenario=None):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)
        obs_map = ObstacleMap(ox, oy, self.config)

        start_yaw = math.atan2(gy - sy, gx - sx)
        if scenario is not None:
            start_yaw = scenario.get("start_yaw", start_yaw)

        start = _RSNode(sx, sy, start_yaw, g=0.0, parent=None)
        queue = [(math.hypot(gx - sx, gy - sy), 0, start)]
        best = {self._key(obs_map, sx, sy, start_yaw): 0.0}
        tie = 0
        explored = 0

        while queue:
            _, _, current = heapq.heappop(queue)
            key = self._key(obs_map, current.x, current.y, current.yaw)
            if current.g > best.get(key, float("inf")):
                continue

            explored += 1
            if math.hypot(current.x - gx, current.y - gy) <= self.step_size * 1.2 and obs_map.is_line_collision_free(current.x, current.y, gx, gy):
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

            for direction in self.directions:
                for steer in self.steer_set:
                    rollout = self._simulate(current.x, current.y, current.yaw, steer, direction)
                    if any(not obs_map.is_collision_free(px, py) for px, py, _ in rollout):
                        continue
                    nx, ny, nyaw = rollout[-1]
                    reverse_penalty = 0.8 if direction < 0.0 else 0.0
                    steer_penalty = 0.2 * abs(steer) / max(self.max_steer, 1e-6)
                    new_cost = current.g + self.step_size + reverse_penalty + steer_penalty
                    nkey = self._key(obs_map, nx, ny, nyaw)
                    if new_cost >= best.get(nkey, float("inf")):
                        continue

                    best[nkey] = new_cost
                    node = _RSNode(nx, ny, nyaw, g=new_cost, parent=current)
                    tie += 1
                    heuristic = math.hypot(gx - nx, gy - ny)
                    heapq.heappush(queue, (new_cost + heuristic, tie, node))

        result.explored_nodes = explored
        result.planning_time = time.perf_counter() - t0
        return result
