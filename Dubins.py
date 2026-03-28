"""
Dubins-style Curvature-Constrained Planner

Builds a curvature-constrained path by rounding A* guide path corners with
minimum-turn-radius arc fillets. This approximates the role of Dubins curves
in a 2D obstacle-map benchmark.
"""

import time

from common import PlannerConfig, fillet_path, resample_path
from guided_utils import build_astar_reference, finalize_path_result


class DubinsPlanner:
    name = "Dubins"

    def __init__(self, config: PlannerConfig = None, min_turn_radius: float = 1.4):
        self.config = config or PlannerConfig()
        self.min_turn_radius = min_turn_radius

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
            control_points=14,
            spacing=0.6,
        )
        if not guide.success:
            return finalize_path_result(self.name, t0, [], [], obs_map, fallback_result=guide)

        rounded_x, rounded_y = fillet_path(ctrl_x, ctrl_y, radius=self.min_turn_radius, samples_per_corner=12)
        rounded_x, rounded_y = resample_path(rounded_x, rounded_y, spacing=0.4)
        return finalize_path_result(
            self.name,
            t0,
            rounded_x,
            rounded_y,
            obs_map,
            fallback_result=guide,
            explored_nodes=max(len(ctrl_x), guide.explored_nodes),
        )
