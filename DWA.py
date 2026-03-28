"""
Dynamic Window Approach Planner

Implements a guide-path-aware DWA rollout. A* provides a coarse global route,
while DWA selects short-horizon velocity commands that track the guide path
and remain collision-free.
"""

import math
import time

import numpy as np

from common import PlannerConfig, PlanningResult, calc_path_length, nearest_path_index, resample_path
from guided_utils import build_astar_reference, finalize_path_result


class DWAPlanner:
    name = "DWA"

    def __init__(
        self,
        config: PlannerConfig = None,
        horizon: float = 1.6,
        dt: float = 0.25,
        max_speed: float = 2.4,
        max_yaw_rate: float = 1.1,
    ):
        self.config = config or PlannerConfig()
        self.horizon = horizon
        self.dt = dt
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate

    def _rollout(self, x, y, yaw, v, omega):
        steps = max(2, int(self.horizon / self.dt))
        traj = []
        cx, cy, cyaw = x, y, yaw
        for _ in range(steps):
            cx += v * self.dt * math.cos(cyaw)
            cy += v * self.dt * math.sin(cyaw)
            cyaw += omega * self.dt
            traj.append((cx, cy, cyaw))
        return traj

    def plan(self, sx, sy, gx, gy, ox, oy, scenario=None):
        t0 = time.perf_counter()
        guide, obs_map, _, _ = build_astar_reference(
            self.config,
            sx,
            sy,
            gx,
            gy,
            ox,
            oy,
            control_points=10,
            spacing=0.8,
        )
        if not guide.success:
            return finalize_path_result(self.name, t0, [], [], obs_map, fallback_result=guide)

        guide_x, guide_y = resample_path(guide.path_x, guide.path_y, spacing=0.7)
        yaw = math.atan2(guide_y[1] - guide_y[0], guide_x[1] - guide_x[0]) if len(guide_x) > 1 else 0.0
        x, y = sx, sy
        trajectory_x = [x]
        trajectory_y = [y]
        explored = 0
        obstacle_points = np.column_stack([np.asarray(ox), np.asarray(oy)])

        speed_candidates = np.linspace(0.8, self.max_speed, 4)
        yaw_rate_candidates = np.linspace(-self.max_yaw_rate, self.max_yaw_rate, 7)

        for _ in range(150):
            if math.hypot(x - gx, y - gy) <= 1.4 and obs_map.is_line_collision_free(x, y, gx, gy):
                trajectory_x.append(gx)
                trajectory_y.append(gy)
                return finalize_path_result(
                    self.name,
                    t0,
                    trajectory_x,
                    trajectory_y,
                    obs_map,
                    fallback_result=guide,
                    explored_nodes=explored,
                )

            idx = nearest_path_index(guide_x, guide_y, x, y)
            target_idx = min(len(guide_x) - 1, idx + 7)
            tx, ty = guide_x[target_idx], guide_y[target_idx]

            best_score = -float("inf")
            best_rollout = None
            for v in speed_candidates:
                for omega in yaw_rate_candidates:
                    rollout = self._rollout(x, y, yaw, float(v), float(omega))
                    explored += 1
                    collision = False
                    clearance = float("inf")
                    for px, py, _ in rollout:
                        if not obs_map.is_collision_free(px, py):
                            collision = True
                            break
                        d = float(np.min(np.hypot(obstacle_points[:, 0] - px, obstacle_points[:, 1] - py)))
                        clearance = min(clearance, d)
                    if collision:
                        continue

                    end_x, end_y, end_yaw = rollout[-1]
                    goal_score = -math.hypot(end_x - tx, end_y - ty)
                    heading_score = math.cos(math.atan2(ty - end_y, tx - end_x) - end_yaw)
                    speed_score = 0.4 * (v / max(self.max_speed, 1e-6))
                    clearance_score = 0.2 * min(clearance, 8.0)
                    score = goal_score + 1.2 * heading_score + speed_score + clearance_score
                    if score > best_score:
                        best_score = score
                        best_rollout = rollout

            if best_rollout is None:
                break

            x, y, yaw = best_rollout[0]
            trajectory_x.append(x)
            trajectory_y.append(y)

        return finalize_path_result(
            self.name,
            t0,
            trajectory_x,
            trajectory_y,
            obs_map,
            fallback_result=guide,
            explored_nodes=explored,
        )
