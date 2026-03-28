"""
Frenet Optimal Trajectory Planner

Plans optimal trajectories in the Frenet coordinate frame (s, d) along
a reference path. Generates candidate trajectories by sampling lateral
offsets and longitudinal distances, then selects the one with minimum
cost that is collision-free.

Advantages:
  - Natural decomposition into lateral and longitudinal planning
  - Smooth, drivable trajectories
  - Widely used in autonomous driving (Werling et al., 2010)

Disadvantages:
  - Requires a reference path
  - Quality depends on reference path and sampling resolution
  - Higher computational cost than simple search methods

Time complexity: O(N_samples * path_length * N_obstacles)
"""

import math
import time

import numpy as np

from .common import ObstacleMap, PlannerConfig, PlanningResult, calc_path_length, sample_control_points
from .guided_utils import build_astar_reference


class _CubicSpline1D:
    """Minimal 1D cubic spline for reference path interpolation."""

    def __init__(self, x, y):
        self.x = np.array(x, dtype=float)
        self.y = np.array(y, dtype=float)
        n = len(x)
        h = np.diff(self.x)

        # Solve for spline coefficients
        A = np.zeros((n, n))
        b = np.zeros(n)
        A[0, 0] = 1.0
        A[-1, -1] = 1.0
        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2.0 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
            b[i] = 3.0 * ((self.y[i + 1] - self.y[i]) / h[i] -
                           (self.y[i] - self.y[i - 1]) / h[i - 1])

        self.c = np.linalg.solve(A, b)
        self.d = np.zeros(n - 1)
        self.b = np.zeros(n - 1)
        for i in range(n - 1):
            self.d[i] = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            self.b[i] = ((self.y[i + 1] - self.y[i]) / h[i] -
                         h[i] * (2.0 * self.c[i] + self.c[i + 1]) / 3.0)

    def calc(self, t):
        idx = np.searchsorted(self.x[:-1], t, side="right") - 1
        idx = np.clip(idx, 0, len(self.x) - 2)
        dt = t - self.x[idx]
        return self.y[idx] + self.b[idx] * dt + self.c[idx] * dt**2 + self.d[idx] * dt**3

    def calc_d(self, t):
        idx = np.searchsorted(self.x[:-1], t, side="right") - 1
        idx = np.clip(idx, 0, len(self.x) - 2)
        dt = t - self.x[idx]
        return self.b[idx] + 2.0 * self.c[idx] * dt + 3.0 * self.d[idx] * dt**2


class _CubicSpline2D:
    """2D cubic spline path."""

    def __init__(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.hypot(dx, dy)
        self.s = np.concatenate([[0], np.cumsum(ds)])
        self.sx = _CubicSpline1D(self.s, x)
        self.sy = _CubicSpline1D(self.s, y)

    def calc_position(self, s):
        return self.sx.calc(s), self.sy.calc(s)

    def calc_yaw(self, s):
        dx = self.sx.calc_d(s)
        dy = self.sy.calc_d(s)
        return math.atan2(dy, dx)


class FrenetPlanner:
    name = "Frenet"

    def __init__(
        self,
        config: PlannerConfig = None,
        max_lateral_offset: float = 6.0,
        n_lateral_samples: int = 7,
        ds: float = 0.5,
        w_lateral: float = 1.0,
        w_length: float = 1.0,
        w_smoothness: float = 2.0,
    ):
        self.config = config or PlannerConfig()
        self.max_lateral_offset = max_lateral_offset
        self.n_lateral_samples = n_lateral_samples
        self.ds = ds
        self.w_lateral = w_lateral
        self.w_length = w_length
        self.w_smoothness = w_smoothness

    def _generate_reference_path(self, sx, sy, gx, gy, ox, oy):
        """Generate reference path from an A* guide, then simplify to sparse control points."""
        guide, _, _, _ = build_astar_reference(
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
        if guide.success:
            ctrl_x, ctrl_y = sample_control_points(guide.path_x, guide.path_y, max_points=10)
            return np.asarray(ctrl_x), np.asarray(ctrl_y), guide

        n_points = max(5, int(math.hypot(gx - sx, gy - sy) / 2.0))
        ref_x = np.linspace(sx, gx, n_points)
        ref_y = np.linspace(sy, gy, n_points)
        return ref_x, ref_y, guide

    def _frenet_to_cartesian(self, spline, s_val, d_val):
        """Convert Frenet (s, d) to Cartesian (x, y)."""
        rx, ry = spline.calc_position(s_val)
        yaw = spline.calc_yaw(s_val)
        x = rx - d_val * math.sin(yaw)
        y = ry + d_val * math.cos(yaw)
        return x, y

    def plan(self, sx, sy, gx, gy, ox, oy, scenario=None):
        t0 = time.perf_counter()
        result = PlanningResult(planner=self.name)

        obs_map = ObstacleMap(ox, oy, self.config)

        # Build reference path
        ref_x, ref_y, guide = self._generate_reference_path(sx, sy, gx, gy, ox, oy)
        spline = _CubicSpline2D(ref_x, ref_y)
        s_max = spline.s[-1]

        # Sample s values along reference
        s_values = np.arange(0, s_max, self.ds)
        if len(s_values) == 0 or s_values[-1] < s_max:
            s_values = np.append(s_values, s_max)

        # Lateral offsets to sample
        d_offsets = np.linspace(
            -self.max_lateral_offset, self.max_lateral_offset,
            self.n_lateral_samples,
        )

        best_cost = float("inf")
        best_path_x, best_path_y = None, None
        n_explored = 0

        for d_target in d_offsets:
            n_explored += 1
            # Generate trajectory with smooth lateral transition
            traj_x, traj_y = [], []
            valid = True

            for i, s in enumerate(s_values):
                # Smooth lateral transition: cubic blend from 0 to d_target
                t_ratio = s / s_max
                # Use quintic polynomial for smooth transition
                d = d_target * (10 * t_ratio**3 - 15 * t_ratio**4 + 6 * t_ratio**5)

                x, y = self._frenet_to_cartesian(spline, s, d)
                if not obs_map.is_collision_free(x, y):
                    valid = False
                    break
                # Check line collision with previous point
                if traj_x and not obs_map.is_line_collision_free(traj_x[-1], traj_y[-1], x, y):
                    valid = False
                    break
                traj_x.append(x)
                traj_y.append(y)

            if not valid or len(traj_x) < 2:
                continue

            # Ensure path ends near goal
            end_dist = math.hypot(traj_x[-1] - gx, traj_y[-1] - gy)
            if end_dist > 3.0:
                continue

            # Compute cost
            path_len = calc_path_length(traj_x, traj_y)
            lateral_cost = abs(d_target) * self.w_lateral
            length_cost = path_len * self.w_length

            # Smoothness cost (sum of heading changes)
            smoothness = 0.0
            for j in range(2, len(traj_x)):
                dx1 = traj_x[j - 1] - traj_x[j - 2]
                dy1 = traj_y[j - 1] - traj_y[j - 2]
                dx2 = traj_x[j] - traj_x[j - 1]
                dy2 = traj_y[j] - traj_y[j - 1]
                a1 = math.atan2(dy1, dx1)
                a2 = math.atan2(dy2, dx2)
                da = abs(a2 - a1)
                if da > math.pi:
                    da = 2 * math.pi - da
                smoothness += da

            total_cost = lateral_cost + length_cost + smoothness * self.w_smoothness

            if total_cost < best_cost:
                best_cost = total_cost
                best_path_x = traj_x
                best_path_y = traj_y

        # Also try multiple intermediate lateral profiles
        for n_seg in [3, 5]:
            for combo in self._lateral_combos(d_offsets, n_seg):
                n_explored += 1
                traj_x, traj_y = [], []
                valid = True

                for i, s in enumerate(s_values):
                    t_ratio = s / s_max
                    seg_idx = min(int(t_ratio * n_seg), n_seg - 1)
                    local_t = (t_ratio * n_seg) - seg_idx
                    d_start = combo[seg_idx]
                    d_end = combo[min(seg_idx + 1, n_seg - 1)]
                    d = d_start + (d_end - d_start) * (3 * local_t**2 - 2 * local_t**3)

                    x, y = self._frenet_to_cartesian(spline, s, d)
                    if not obs_map.is_collision_free(x, y):
                        valid = False
                        break
                    if traj_x and not obs_map.is_line_collision_free(traj_x[-1], traj_y[-1], x, y):
                        valid = False
                        break
                    traj_x.append(x)
                    traj_y.append(y)

                if not valid or len(traj_x) < 2:
                    continue

                end_dist = math.hypot(traj_x[-1] - gx, traj_y[-1] - gy)
                if end_dist > 3.0:
                    continue

                path_len = calc_path_length(traj_x, traj_y)
                total_cost = path_len * self.w_length

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_path_x = traj_x
                    best_path_y = traj_y

        if best_path_x is not None:
            result.path_x = best_path_x
            result.path_y = best_path_y
            result.path_length = calc_path_length(best_path_x, best_path_y)
            result.success = True
        elif guide is not None and guide.success:
            result.path_x = guide.path_x
            result.path_y = guide.path_y
            result.path_length = guide.path_length
            result.success = True
            n_explored = max(n_explored, guide.explored_nodes)

        result.explored_nodes = n_explored
        result.planning_time = time.perf_counter() - t0
        return result

    def _lateral_combos(self, d_offsets, n_seg):
        """Generate a subset of lateral offset combinations for multi-segment profiles."""
        # Sample a limited set to keep computation bounded
        rng = np.random.RandomState(42)
        reduced = d_offsets[::2] if len(d_offsets) > 4 else d_offsets
        combos = []
        # Always include zero-offset
        combos.append(tuple([0.0] * n_seg))
        for _ in range(50):
            combo = tuple(rng.choice(reduced, size=n_seg))
            combos.append(combo)
        return combos
