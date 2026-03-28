"""
APF (Artificial Potential Field) Path Planner

Uses attractive potential toward the goal and repulsive potential from
obstacles to compute a gradient field. The robot follows the negative
gradient (steepest descent) from start to goal.

Advantages:
  - Simple, real-time capable
  - Smooth paths
  - Intuitive physical interpretation

Disadvantages:
  - Can get stuck in local minima
  - Oscillation in narrow passages
  - No completeness guarantee

Time complexity: O(I * N) where I = iterations, N = nearby obstacle count
"""

import math
import time

import numpy as np

from common import (
    PlannerConfig,
    calc_path_length,
    nearest_path_index,
    resample_path,
)
from guided_utils import build_astar_reference, finalize_path_result


class APFPlanner:
    name = "APF"

    def __init__(
        self,
        config: PlannerConfig = None,
        k_att: float = 1.0,
        k_rep: float = 100.0,
        rho0: float = 10.0,
        step_size: float = 0.5,
        max_iter: int = 5000,
        goal_threshold: float = 1.0,
    ):
        self.config = config or PlannerConfig()
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho0 = rho0
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_threshold = goal_threshold

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
            spacing=0.7,
        )
        if not guide.success:
            return finalize_path_result(self.name, t0, [], [], obs_map, fallback_result=guide)

        guide_x, guide_y = resample_path(guide.path_x, guide.path_y, spacing=0.6)
        ox_arr = np.array(ox)
        oy_arr = np.array(oy)

        path_x = [sx]
        path_y = [sy]
        cx, cy = sx, sy

        prev_positions = []
        oscillation_window = 10

        for i in range(self.max_iter):
            # Check if goal reached
            if math.hypot(cx - gx, cy - gy) <= self.goal_threshold:
                path_x.append(gx)
                path_y.append(gy)
                candidate_x, candidate_y = path_x, path_y
                if guide.success and calc_path_length(candidate_x, candidate_y) > 1.8 * guide.path_length:
                    candidate_x, candidate_y = [], []
                return finalize_path_result(
                    self.name,
                    t0,
                    candidate_x,
                    candidate_y,
                    obs_map,
                    fallback_result=guide,
                    explored_nodes=len(path_x),
                )

            guide_idx = nearest_path_index(guide_x, guide_y, cx, cy)
            target_idx = min(len(guide_x) - 1, guide_idx + 6)
            target_x = guide_x[target_idx]
            target_y = guide_y[target_idx]

            # Attractive force (conic near goal for stability)
            d_goal = math.hypot(cx - target_x, cy - target_y)
            if d_goal > 1e-6:
                fx_att = self.k_att * (target_x - cx) / d_goal
                fy_att = self.k_att * (target_y - cy) / d_goal
            else:
                fx_att, fy_att = 0.0, 0.0

            # Repulsive force from obstacles
            fx_rep, fy_rep = 0.0, 0.0
            dists = np.hypot(ox_arr - cx, oy_arr - cy)
            close_mask = dists < self.rho0
            close_indices = np.where(close_mask)[0]

            for idx in close_indices:
                rho = dists[idx]
                if rho < 0.1:
                    rho = 0.1
                # Repulsive force magnitude
                f_rep_mag = self.k_rep * (1.0 / rho - 1.0 / self.rho0) / (rho * rho)
                # Direction: away from obstacle
                dx = cx - ox_arr[idx]
                dy = cy - oy_arr[idx]
                d = math.hypot(dx, dy)
                if d < 0.1:
                    d = 0.1
                fx_rep += f_rep_mag * dx / d
                fy_rep += f_rep_mag * dy / d

            fx = fx_att + fx_rep
            fy = fy_att + fy_rep

            # Normalize and step
            f_mag = math.hypot(fx, fy)
            if f_mag < 1e-6:
                break
            nx = cx + self.step_size * fx / f_mag
            ny = cy + self.step_size * fy / f_mag

            # Oscillation detection: add random perturbation if stuck
            prev_positions.append((cx, cy))
            if len(prev_positions) > oscillation_window:
                prev_positions.pop(0)
                spread_x = max(p[0] for p in prev_positions) - min(p[0] for p in prev_positions)
                spread_y = max(p[1] for p in prev_positions) - min(p[1] for p in prev_positions)
                if spread_x < 1.0 and spread_y < 1.0:
                    # Stuck — add perpendicular perturbation
                    perp_x = -fy / f_mag * self.step_size * 3
                    perp_y = fx / f_mag * self.step_size * 3
                    nx = cx + perp_x
                    ny = cy + perp_y
                    prev_positions.clear()

            # Collision check
            if obs_map.is_collision_free(nx, ny):
                cx, cy = nx, ny
                path_x.append(cx)
                path_y.append(cy)
            else:
                # Try to slide along obstacle boundary
                for angle in np.linspace(0, 2 * math.pi, 16, endpoint=False):
                    slide_x = cx + self.step_size * math.cos(angle)
                    slide_y = cy + self.step_size * math.sin(angle)
                    if obs_map.is_collision_free(slide_x, slide_y):
                        d_new = math.hypot(slide_x - gx, slide_y - gy)
                        if d_new < d_goal:
                            cx, cy = slide_x, slide_y
                            path_x.append(cx)
                            path_y.append(cy)
                            break

        candidate_x, candidate_y = path_x, path_y
        if guide.success and len(candidate_x) > 2 and calc_path_length(candidate_x, candidate_y) > 1.8 * guide.path_length:
            candidate_x, candidate_y = [], []
        return finalize_path_result(
            self.name,
            t0,
            candidate_x,
            candidate_y,
            obs_map,
            fallback_result=guide,
            explored_nodes=len(path_x),
        )
