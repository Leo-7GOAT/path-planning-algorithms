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
    planning_time: float = 0.0
    explored_nodes: int = 0
    success: bool = False

    def to_dict(self):
        return {
            "Planner": self.planner,
            "Success": "Yes" if self.success else "No",
            "Path Length (m)": f"{self.path_length:.2f}",
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
