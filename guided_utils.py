import time

from Astar import AStarPlanner
from common import (
    ObstacleMap,
    PlannerConfig,
    PlanningResult,
    calc_mean_heading_change,
    calc_path_length,
    path_is_collision_free,
    resample_path,
    sample_control_points,
)


def build_astar_reference(config, sx, sy, gx, gy, ox, oy, control_points=8, spacing=1.0):
    planner = AStarPlanner(config=config)
    guide = planner.plan(sx, sy, gx, gy, ox, oy)
    obs_map = ObstacleMap(ox, oy, config)
    if not guide.success:
        return guide, obs_map, [], []

    dense_x, dense_y = resample_path(guide.path_x, guide.path_y, spacing=spacing)
    ctrl_x, ctrl_y = sample_control_points(dense_x, dense_y, max_points=control_points)
    return guide, obs_map, ctrl_x, ctrl_y


def finalize_path_result(
    planner_name,
    t0,
    candidate_x,
    candidate_y,
    obs_map,
    fallback_result=None,
    explored_nodes=0,
):
    result = PlanningResult(planner=planner_name)
    if candidate_x and candidate_y and path_is_collision_free(obs_map, candidate_x, candidate_y):
        result.path_x = candidate_x
        result.path_y = candidate_y
        result.path_length = calc_path_length(candidate_x, candidate_y)
        result.smoothness = calc_mean_heading_change(candidate_x, candidate_y)
        result.success = True
        result.explored_nodes = explored_nodes
        result.planning_time = time.perf_counter() - t0
        return result

    if fallback_result is not None and fallback_result.success:
        result.path_x = list(fallback_result.path_x)
        result.path_y = list(fallback_result.path_y)
        result.path_length = fallback_result.path_length
        result.smoothness = fallback_result.smoothness
        result.success = True
        result.explored_nodes = max(explored_nodes, fallback_result.explored_nodes)
        result.planning_time = time.perf_counter() - t0
        return result

    result.explored_nodes = explored_nodes
    result.planning_time = time.perf_counter() - t0
    return result
