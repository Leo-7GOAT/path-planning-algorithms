import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import numpy as np


@dataclass
class PlannerConfig:
    resolution: float = 1.0
    robot_radius: float = 2.0
    margin: float = 5.0


@dataclass
class PlanningResult:
    planner: str
    path_x: List[float] = field(default_factory=list)
    path_y: List[float] = field(default_factory=list)
    path_length: float = 0.0
    smoothness: float = 0.0
    planning_time: float = 0.0
    explored_nodes: int = 0
    success: bool = False

    def to_dict(self):
        return {
            "Planner": self.planner,
            "Success": "Yes" if self.success else "No",
            "Path Length (m)": f"{self.path_length:.2f}",
            "Mean Heading Change (rad)": f"{self.smoothness:.3f}",
            "Planning Time (ms)": f"{self.planning_time * 1000:.2f}",
            "Explored Nodes": str(self.explored_nodes),
        }


class ObstacleMap:
    """Grid-based obstacle map for 2D planning."""

    def __init__(self, obstacles_x, obstacles_y, config: PlannerConfig):
        self.config = config
        self.resolution = config.resolution
        self.robot_radius = config.robot_radius

        self.min_x = round(min(obstacles_x) - config.margin)
        self.min_y = round(min(obstacles_y) - config.margin)
        self.max_x = round(max(obstacles_x) + config.margin)
        self.max_y = round(max(obstacles_y) + config.margin)

        self.x_width = round((self.max_x - self.min_x) / self.resolution) + 1
        self.y_width = round((self.max_y - self.min_y) / self.resolution) + 1

        self.obstacle_map = np.zeros((self.x_width, self.y_width), dtype=bool)
        self._build_map(obstacles_x, obstacles_y)

    def _build_map(self, ox, oy):
        for ix in range(self.x_width):
            x = self.calc_world_x(ix)
            for iy in range(self.y_width):
                y = self.calc_world_y(iy)
                for obx, oby in zip(ox, oy):
                    d = math.hypot(obx - x, oby - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    def calc_grid_x(self, x):
        return round((x - self.min_x) / self.resolution)

    def calc_grid_y(self, y):
        return round((y - self.min_y) / self.resolution)

    def calc_world_x(self, ix):
        return ix * self.resolution + self.min_x

    def calc_world_y(self, iy):
        return iy * self.resolution + self.min_y

    def is_valid(self, ix, iy):
        if ix < 0 or ix >= self.x_width:
            return False
        if iy < 0 or iy >= self.y_width:
            return False
        return not self.obstacle_map[ix][iy]

    def is_collision_free(self, x, y):
        """Check if a continuous position is collision-free."""
        ix = self.calc_grid_x(x)
        iy = self.calc_grid_y(y)
        return self.is_valid(ix, iy)

    def is_line_collision_free(self, x1, y1, x2, y2, step=0.5):
        """Check if a line segment is collision-free."""
        d = math.hypot(x2 - x1, y2 - y1)
        if d < 1e-6:
            return self.is_collision_free(x1, y1)
        n_steps = max(2, int(d / step))
        for i in range(n_steps + 1):
            t = i / n_steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.is_collision_free(x, y):
                return False
        return True


def calc_path_length(path_x, path_y):
    length = 0.0
    for i in range(1, len(path_x)):
        length += math.hypot(path_x[i] - path_x[i - 1], path_y[i] - path_y[i - 1])
    return length


def calc_mean_heading_change(path_x, path_y):
    if len(path_x) < 3:
        return 0.0
    headings = compute_path_headings(path_x, path_y)
    deltas = []
    for i in range(1, len(headings)):
        delta = headings[i] - headings[i - 1]
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        deltas.append(abs(delta))
    return float(np.mean(deltas)) if deltas else 0.0


def path_is_collision_free(obs_map, path_x, path_y):
    if len(path_x) != len(path_y) or len(path_x) < 2:
        return False
    if not obs_map.is_collision_free(path_x[0], path_y[0]):
        return False
    for i in range(1, len(path_x)):
        if not obs_map.is_line_collision_free(path_x[i - 1], path_y[i - 1], path_x[i], path_y[i]):
            return False
    return True


def resample_path(path_x, path_y, spacing=1.0):
    if len(path_x) < 2:
        return list(path_x), list(path_y)

    distances = [0.0]
    for i in range(1, len(path_x)):
        distances.append(
            distances[-1] + math.hypot(path_x[i] - path_x[i - 1], path_y[i] - path_y[i - 1])
        )

    total = distances[-1]
    if total < 1e-6:
        return [path_x[0], path_x[-1]], [path_y[0], path_y[-1]]

    samples = np.arange(0.0, total, max(spacing, 1e-3))
    if len(samples) == 0 or samples[-1] < total:
        samples = np.append(samples, total)

    resampled_x, resampled_y = [], []
    seg = 0
    for target in samples:
        while seg < len(distances) - 2 and distances[seg + 1] < target:
            seg += 1
        left = distances[seg]
        right = distances[seg + 1]
        if right - left < 1e-9:
            ratio = 0.0
        else:
            ratio = (target - left) / (right - left)
        resampled_x.append(path_x[seg] + ratio * (path_x[seg + 1] - path_x[seg]))
        resampled_y.append(path_y[seg] + ratio * (path_y[seg + 1] - path_y[seg]))
    return resampled_x, resampled_y


def sample_control_points(path_x, path_y, max_points=8):
    if len(path_x) <= max_points:
        return list(path_x), list(path_y)

    total = calc_path_length(path_x, path_y)
    if total < 1e-6:
        return [path_x[0], path_x[-1]], [path_y[0], path_y[-1]]

    target_distances = np.linspace(0.0, total, max_points)
    prefix = [0.0]
    for i in range(1, len(path_x)):
        prefix.append(prefix[-1] + math.hypot(path_x[i] - path_x[i - 1], path_y[i] - path_y[i - 1]))

    sampled_x, sampled_y = [], []
    seg = 0
    for target in target_distances:
        while seg < len(prefix) - 2 and prefix[seg + 1] < target:
            seg += 1
        left = prefix[seg]
        right = prefix[seg + 1]
        ratio = 0.0 if right - left < 1e-9 else (target - left) / (right - left)
        sampled_x.append(path_x[seg] + ratio * (path_x[seg + 1] - path_x[seg]))
        sampled_y.append(path_y[seg] + ratio * (path_y[seg + 1] - path_y[seg]))
    return sampled_x, sampled_y


def chaikin_smooth_path(path_x, path_y, iterations=2):
    px = list(path_x)
    py = list(path_y)
    for _ in range(iterations):
        if len(px) < 3:
            break
        new_x = [px[0]]
        new_y = [py[0]]
        for i in range(len(px) - 1):
            qx = 0.75 * px[i] + 0.25 * px[i + 1]
            qy = 0.75 * py[i] + 0.25 * py[i + 1]
            rx = 0.25 * px[i] + 0.75 * px[i + 1]
            ry = 0.25 * py[i] + 0.75 * py[i + 1]
            new_x.extend([qx, rx])
            new_y.extend([qy, ry])
        new_x.append(px[-1])
        new_y.append(py[-1])
        px, py = new_x, new_y
    return px, py


def catmull_rom_chain(path_x, path_y, samples_per_seg=18):
    points = np.column_stack([path_x, path_y]).astype(float)
    if len(points) < 2:
        return list(path_x), list(path_y)
    if len(points) == 2:
        return resample_path(path_x, path_y, spacing=max(calc_path_length(path_x, path_y) / 20.0, 0.5))

    padded = np.vstack([points[0], points, points[-1]])
    curve = []
    for i in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
        ts = np.linspace(0.0, 1.0, samples_per_seg, endpoint=False)
        for t in ts:
            t2 = t * t
            t3 = t2 * t
            point = 0.5 * (
                (2.0 * p1)
                + (-p0 + p2) * t
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
            )
            curve.append(point)
    curve.append(points[-1])
    curve = np.asarray(curve)
    return curve[:, 0].tolist(), curve[:, 1].tolist()


def uniform_bspline_path(path_x, path_y, samples_per_seg=20):
    control = np.column_stack([path_x, path_y]).astype(float)
    if len(control) < 4:
        return catmull_rom_chain(path_x, path_y, samples_per_seg=max(12, samples_per_seg // 2))

    curve = [control[0]]
    basis = np.array(
        [
            [-1.0, 3.0, -3.0, 1.0],
            [3.0, -6.0, 3.0, 0.0],
            [-3.0, 0.0, 3.0, 0.0],
            [1.0, 4.0, 1.0, 0.0],
        ],
        dtype=float,
    ) / 6.0

    for i in range(len(control) - 3):
        segment = control[i : i + 4]
        ts = np.linspace(0.0, 1.0, samples_per_seg, endpoint=False)
        for t in ts:
            coeff = np.array([t**3, t**2, t, 1.0], dtype=float) @ basis
            point = coeff @ segment
            curve.append(point)
    curve.append(control[-1])
    curve = np.asarray(curve)
    return curve[:, 0].tolist(), curve[:, 1].tolist()


def fillet_path(path_x, path_y, radius=2.5, samples_per_corner=10):
    if len(path_x) < 3:
        return list(path_x), list(path_y)

    fx = [path_x[0]]
    fy = [path_y[0]]
    for i in range(1, len(path_x) - 1):
        p_prev = np.array([path_x[i - 1], path_y[i - 1]], dtype=float)
        p_curr = np.array([path_x[i], path_y[i]], dtype=float)
        p_next = np.array([path_x[i + 1], path_y[i + 1]], dtype=float)

        v1 = p_curr - p_prev
        v2 = p_next - p_curr
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        if len1 < 1e-6 or len2 < 1e-6:
            continue

        d1 = v1 / len1
        d2 = v2 / len2
        turn = np.clip(np.dot(-d1, d2), -1.0, 1.0)
        angle = math.acos(turn)
        if angle < math.radians(12.0) or abs(math.pi - angle) < math.radians(12.0):
            fx.append(p_curr[0])
            fy.append(p_curr[1])
            continue

        tangent = min(radius / math.tan(angle / 2.0), 0.45 * len1, 0.45 * len2)
        start = p_curr - d1 * tangent
        end = p_curr + d2 * tangent
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        normal1 = np.array([-d1[1], d1[0]]) if cross > 0.0 else np.array([d1[1], -d1[0]])
        normal2 = np.array([-d2[1], d2[0]]) if cross > 0.0 else np.array([d2[1], -d2[0]])
        center1 = start + normal1 * radius
        center2 = end + normal2 * radius
        center = 0.5 * (center1 + center2)

        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        end_angle = math.atan2(end[1] - center[1], end[0] - center[0])

        if cross > 0.0 and end_angle < start_angle:
            end_angle += 2.0 * math.pi
        elif cross < 0.0 and end_angle > start_angle:
            end_angle -= 2.0 * math.pi

        fx.append(start[0])
        fy.append(start[1])
        for theta in np.linspace(start_angle, end_angle, samples_per_corner, endpoint=False)[1:]:
            fx.append(center[0] + radius * math.cos(theta))
            fy.append(center[1] + radius * math.sin(theta))
        fx.append(end[0])
        fy.append(end[1])

    fx.append(path_x[-1])
    fy.append(path_y[-1])
    return fx, fy


def compute_path_headings(path_x, path_y):
    if len(path_x) < 2:
        return [0.0]
    headings = []
    for i in range(len(path_x) - 1):
        headings.append(math.atan2(path_y[i + 1] - path_y[i], path_x[i + 1] - path_x[i]))
    headings.append(headings[-1])
    return headings


def nearest_path_index(path_x, path_y, x, y):
    distances = [(path_x[i] - x) ** 2 + (path_y[i] - y) ** 2 for i in range(len(path_x))]
    return int(np.argmin(distances))


def build_scenario_obstacles():
    """Build three planning scenarios with different obstacle layouts."""
    scenarios = {}

    # Scenario 1: Simple maze-like corridor
    ox, oy = [], []
    for i in range(0, 60):
        ox.append(i); oy.append(0.0)
    for i in range(0, 60):
        ox.append(i); oy.append(60.0)
    for i in range(0, 61):
        ox.append(0.0); oy.append(i)
    for i in range(0, 61):
        ox.append(60.0); oy.append(i)
    for i in range(0, 40):
        ox.append(20.0); oy.append(i)
    for i in range(20, 61):
        ox.append(40.0); oy.append(i)
    scenarios["corridor"] = {
        "ox": ox, "oy": oy,
        "sx": 5.0, "sy": 5.0, "gx": 55.0, "gy": 55.0,
        "start_yaw": 0.0,
        "goal_yaw": math.pi / 2.0,
    }

    # Scenario 2: Scattered obstacles
    ox, oy = [], []
    for i in range(-10, 61):
        ox.append(i); oy.append(-10.0)
    for i in range(-10, 61):
        ox.append(i); oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0); oy.append(i)
    for i in range(-10, 61):
        ox.append(60.0); oy.append(i)
    rng = np.random.RandomState(42)
    for _ in range(40):
        cx, cy = rng.uniform(8, 48), rng.uniform(8, 48)
        if math.hypot(cx, cy) < 8 or math.hypot(cx - 50, cy - 50) < 8:
            continue
        r = rng.uniform(1, 2)
        for angle in np.linspace(0, 2 * math.pi, int(r * 8)):
            ox.append(cx + r * math.cos(angle))
            oy.append(cy + r * math.sin(angle))
    scenarios["scattered"] = {
        "ox": ox, "oy": oy,
        "sx": 0.0, "sy": 0.0, "gx": 50.0, "gy": 50.0,
        "start_yaw": math.pi / 4.0,
        "goal_yaw": math.pi / 4.0,
    }

    # Scenario 3: Narrow passage
    ox, oy = [], []
    for i in range(0, 51):
        ox.append(i); oy.append(0.0)
    for i in range(0, 51):
        ox.append(i); oy.append(40.0)
    for i in range(0, 41):
        ox.append(0.0); oy.append(i)
    for i in range(0, 41):
        ox.append(50.0); oy.append(i)
    for i in range(0, 17):
        ox.append(25.0); oy.append(i)
    for i in range(23, 41):
        ox.append(25.0); oy.append(i)
    scenarios["narrow_passage"] = {
        "ox": ox, "oy": oy,
        "sx": 5.0, "sy": 20.0, "gx": 45.0, "gy": 20.0,
        "start_yaw": 0.0,
        "goal_yaw": 0.0,
    }

    return scenarios


def format_table(results):
    rows = [r.to_dict() for r in results]
    if not rows:
        return ""
    headers = list(rows[0].keys())
    widths = [max(len(h), *(len(str(row[h])) for row in rows)) for h in headers]
    lines = [" | ".join(h.ljust(w) for h, w in zip(headers, widths))]
    lines.append("-+-".join("-" * w for w in widths))
    for row in rows:
        lines.append(" | ".join(str(row[h]).ljust(w) for h, w in zip(headers, widths)))
    return "\n".join(lines)
