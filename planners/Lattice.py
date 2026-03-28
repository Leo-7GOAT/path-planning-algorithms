"""
State Lattice Planner

Searches over a fixed library of motion primitives. Compared with Hybrid A*,
the motion set is more structured and explicitly primitive-based.
"""

import heapq
import math
import time

from .common import PlannerConfig, PlanningResult, ObstacleMap, calc_path_length, resample_path


class _LatticeNode:
    __slots__ = ("x", "y", "yaw", "cost", "parent")

    def __init__(self, x, y, yaw, cost=0.0, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = cost
        self.parent = parent


class LatticePlanner:
    name = "Lattice"

    def __init__(self, config: PlannerConfig = None, heading_bins: int = 16):
        self.config = config or PlannerConfig()
        self.heading_bins = heading_bins
        self.primitives = [
            (3.0, 0.0, 0.0),
            (3.5, 0.0, math.radians(20.0)),
            (3.5, 0.0, -math.radians(20.0)),
            (4.0, 0.0, math.radians(35.0)),
            (4.0, 0.0, -math.radians(35.0)),
            (2.5, 0.0, math.radians(45.0)),
            (2.5, 0.0, -math.radians(45.0)),
        ]

    def _key(self, obs_map, x, y, yaw):
        theta = int(round(((yaw + math.pi) % (2.0 * math.pi)) / (2.0 * math.pi / self.heading_bins))) % self.heading_bins
        return (obs_map.calc_grid_x(x), obs_map.calc_grid_y(y), theta)

    def _apply_primitive(self, x, y, yaw, length, lateral, delta_yaw):
        n_steps = max(4, int(length / 0.6))
        points = []
        for i in range(1, n_steps + 1):
            ratio = i / n_steps
            local_x = length * ratio
            local_y = lateral * math.sin(math.pi * ratio)
            pyaw = yaw + delta_yaw * ratio
            world_x = x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
            world_y = y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
            points.append((world_x, world_y, pyaw))
        return points

    def plan(self, sx, sy, gx, gy, ox, oy, scenario=None):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)
        obs_map = ObstacleMap(ox, oy, self.config)

        start_yaw = math.atan2(gy - sy, gx - sx)
        if scenario is not None:
            start_yaw = scenario.get("start_yaw", start_yaw)

        start = _LatticeNode(sx, sy, start_yaw, cost=0.0, parent=None)
        queue = [(math.hypot(gx - sx, gy - sy), 0, start)]
        best = {self._key(obs_map, sx, sy, start_yaw): 0.0}
        explored = 0
        tie = 0

        while queue:
            _, _, current = heapq.heappop(queue)
            key = self._key(obs_map, current.x, current.y, current.yaw)
            if current.cost > best.get(key, float("inf")):
                continue

            explored += 1
            if math.hypot(current.x - gx, current.y - gy) <= 3.0 and obs_map.is_line_collision_free(current.x, current.y, gx, gy):
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

            for length, lateral, delta_yaw in self.primitives:
                rollout = self._apply_primitive(current.x, current.y, current.yaw, length, lateral, delta_yaw)
                if any(not obs_map.is_collision_free(px, py) for px, py, _ in rollout):
                    continue
                nx, ny, nyaw = rollout[-1]
                new_cost = current.cost + length + 0.5 * abs(delta_yaw)
                nkey = self._key(obs_map, nx, ny, nyaw)
                if new_cost >= best.get(nkey, float("inf")):
                    continue

                best[nkey] = new_cost
                node = _LatticeNode(nx, ny, nyaw, cost=new_cost, parent=current)
                tie += 1
                heuristic = math.hypot(gx - nx, gy - ny)
                heapq.heappush(queue, (new_cost + heuristic, tie, node))

        result.explored_nodes = explored
        result.planning_time = time.perf_counter() - t0
        return result
