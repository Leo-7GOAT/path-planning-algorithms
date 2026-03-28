"""
Bezier Curve Planner

Uses an A* guide path to extract sparse anchor points, then converts them into
piecewise cubic Bezier segments for a smooth, drivable path.
"""

import time

import numpy as np

from .common import PlannerConfig, resample_path
from .guided_utils import build_astar_reference, finalize_path_result


def _piecewise_bezier(points, samples_per_seg=20):
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points[:, 0].tolist(), points[:, 1].tolist()

    padded = np.vstack([points[0], points, points[-1]])
    curve = [points[0]]
    for i in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
        c1 = p1 + (p2 - p0) / 6.0
        c2 = p2 - (p3 - p1) / 6.0
        ts = np.linspace(0.0, 1.0, samples_per_seg, endpoint=False)
        for t in ts:
            omt = 1.0 - t
            point = (
                (omt**3) * p1
                + 3.0 * (omt**2) * t * c1
                + 3.0 * omt * (t**2) * c2
                + (t**3) * p2
            )
            curve.append(point)
    curve.append(points[-1])
    curve = np.asarray(curve)
    return curve[:, 0].tolist(), curve[:, 1].tolist()


class BezierPlanner:
    name = "Bezier"

    def __init__(self, config: PlannerConfig = None, control_points: int = 14):
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

        smooth_x, smooth_y = _piecewise_bezier(list(zip(ctrl_x, ctrl_y)), samples_per_seg=20)
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
