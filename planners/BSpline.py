"""
B-Spline Planner

Generates a smooth cubic B-spline path from sparse A* guide points.
"""

import time

from .common import PlannerConfig, resample_path, uniform_bspline_path
from .guided_utils import build_astar_reference, finalize_path_result


class BSplinePlanner:
    name = "B-Spline"

    def __init__(self, config: PlannerConfig = None, control_points: int = 16):
        self.config = config or PlannerConfig()
        self.control_points = control_points

    def plan(self, sx, sy, gx, gy, ox, oy, scenario=None):
        t0 = time.perf_counter()
        guide, obs_map, ctrl_x, ctrl_y = build_astar_reference(
            self.config,
            sx,
            sy,
            gx,
            gy,
            ox,
            oy,
            control_points=self.control_points,
            spacing=0.6,
        )
        if not guide.success:
            return finalize_path_result(self.name, t0, [], [], obs_map, fallback_result=guide)

        smooth_x, smooth_y = uniform_bspline_path(ctrl_x, ctrl_y, samples_per_seg=22)
        smooth_x, smooth_y = resample_path(smooth_x, smooth_y, spacing=0.5)
        return finalize_path_result(
            self.name,
            t0,
            smooth_x,
            smooth_y,
            obs_map,
            fallback_result=guide,
            explored_nodes=max(len(ctrl_x), guide.explored_nodes),
        )
