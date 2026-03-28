"""
Path Planner Comparison Benchmark

Runs Dijkstra, A*, RRT, RRT*, and PRM across multiple obstacle scenarios,
collects metrics (path length, planning time, explored nodes), and produces
comparison visualizations and animations.

Usage:
    python Compare_planner.py
    python Compare_planner.py --show
    python Compare_planner.py --animate
    python Compare_planner.py --show-animation
"""

import argparse
import csv
import inspect
import os
import shutil
from contextlib import contextmanager, nullcontext

from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.pyplot as plt
import numpy as np

from APF import APFPlanner
from Astar import AStarPlanner
from BSpline import BSplinePlanner
from Bezier import BezierPlanner
from common import PlannerConfig, build_scenario_obstacles, calc_mean_heading_change, calc_path_length, format_table
from Dijkstra import DijkstraPlanner
from Dubins import DubinsPlanner
from DWA import DWAPlanner
from Frenet import FrenetPlanner
from HybridAstar import HybridAStarPlanner
from Lattice import LatticePlanner
from PRM import PRMPlanner
from ReedsShepp import ReedsSheppPlanner
from RRT import RRTPlanner
from RRT_Star import RRTStarPlanner

COLORS = {
    "Dijkstra": "tab:red",
    "A*": "tab:blue",
    "Hybrid A*": "tab:orange",
    "RRT": "tab:green",
    "RRT*": "tab:purple",
    "PRM": "tab:brown",
    "Bezier": "tab:pink",
    "B-Spline": "tab:gray",
    "Dubins": "goldenrod",
    "Reeds-Shepp": "olive",
    "Frenet": "teal",
    "Lattice": "darkcyan",
    "APF": "crimson",
    "DWA": "navy",
}

PLANNERS = [
    ("Dijkstra", DijkstraPlanner),
    ("A*", AStarPlanner),
    ("Hybrid A*", HybridAStarPlanner),
    ("RRT", RRTPlanner),
    ("RRT*", RRTStarPlanner),
    ("PRM", PRMPlanner),
    ("Bezier", BezierPlanner),
    ("B-Spline", BSplinePlanner),
    ("Dubins", DubinsPlanner),
    ("Reeds-Shepp", ReedsSheppPlanner),
    ("Frenet", FrenetPlanner),
    ("Lattice", LatticePlanner),
    ("APF", APFPlanner),
    ("DWA", DWAPlanner),
]


def _slugify(name):
    return name.lower().replace(" ", "_").replace("*", "_star")


def _prepare_output_dir(output_dir):
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)


@contextmanager
def _suppress_interactive_windows():
    was_interactive = plt.isinteractive()
    plt.ioff()
    try:
        yield
    finally:
        if was_interactive:
            plt.ion()


def _scenario_label(scenario_name):
    labels = {
        "corridor": "Corridor",
        "scattered": "Scattered",
        "narrow_passage": "Narrow Passage",
    }
    return labels.get(scenario_name, scenario_name)


def _draw_scene(axis, scenario_data):
    ox = scenario_data["ox"]
    oy = scenario_data["oy"]
    sx, sy = scenario_data["sx"], scenario_data["sy"]
    gx, gy = scenario_data["gx"], scenario_data["gy"]

    axis.plot(ox, oy, ".k", markersize=1.5, label="Obstacles")
    axis.plot(sx, sy, "og", markersize=8, label="Start")
    axis.plot(gx, gy, "sr", markersize=8, label="Goal")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.3)


def _run_planner(planner, sx, sy, gx, gy, ox, oy, scenario_data):
    signature = inspect.signature(planner.plan)
    kwargs = {}
    if "scenario" in signature.parameters:
        kwargs["scenario"] = scenario_data
    return planner.plan(sx, sy, gx, gy, ox, oy, **kwargs)


def plot_overlay_comparison(scenario_name, scenario_data, scenario_results, output_path, show=False):
    context = nullcontext() if show else _suppress_interactive_windows()
    with context:
        figure, axis = plt.subplots(figsize=(8.2, 6.6))
        _draw_scene(axis, scenario_data)

        for planner_name, result in scenario_results:
            color = COLORS.get(planner_name, "tab:gray")
            if result.success:
                axis.plot(result.path_x, result.path_y, color=color, linewidth=2.2, label=planner_name)
            else:
                axis.plot([], [], color=color, linewidth=2.2, label=f"{planner_name} (fail)")

        axis.set_title(f"{_scenario_label(scenario_name)} - All Planners")
        axis.legend(frameon=False, ncol=4, loc="best", fontsize=8)
        figure.tight_layout()
        figure.savefig(output_path, dpi=180, bbox_inches="tight")

        if show:
            return figure

        plt.close(figure)
        return None


def animate_overlay_comparison(
    scenario_name,
    scenario_data,
    scenario_results,
    output_path=None,
    show=False,
    fps=15,
):
    context = nullcontext() if show else _suppress_interactive_windows()
    with context:
        figure, axis = plt.subplots(figsize=(8.2, 6.6))
        max_points = max((len(result.path_x) for _, result in scenario_results if result.success), default=1)
        frame_count = max(45, max_points * 2)
        progress = np.linspace(0.0, 1.0, frame_count)

        def _draw_frame(frame_no):
            axis.clear()
            _draw_scene(axis, scenario_data)
            frac = float(progress[frame_no])

            for planner_name, result in scenario_results:
                color = COLORS.get(planner_name, "tab:gray")
                if not result.success or not result.path_x:
                    axis.plot([], [], color=color, linewidth=2.0, label=f"{planner_name} (fail)")
                    continue

                point_count = max(1, int(np.ceil(frac * len(result.path_x))))
                point_count = min(point_count, len(result.path_x))
                axis.plot(
                    result.path_x[:point_count],
                    result.path_y[:point_count],
                    color=color,
                    linewidth=2.2,
                    label=planner_name,
                )
                axis.scatter(
                    [result.path_x[point_count - 1]],
                    [result.path_y[point_count - 1]],
                    color=color,
                    s=22,
                )

            axis.set_title(
                f"{_scenario_label(scenario_name)} - All Planners Live ({int(frac * 100):d}%)"
            )
            axis.legend(frameon=False, ncol=4, loc="best", fontsize=8)

        animation = FuncAnimation(
            figure,
            _draw_frame,
            frames=frame_count,
            interval=max(40, int(1000 / fps)),
            repeat=False,
        )

        if output_path:
            animation.save(output_path, writer=PillowWriter(fps=fps))

        if show:
            return figure, animation

        plt.close(figure)
        return None, None


def run_benchmark(output_dir="outputs_planning", show=False, make_animation=False, show_animation=False):
    _prepare_output_dir(output_dir)
    scenarios = build_scenario_obstacles()
    config = PlannerConfig(resolution=1.0, robot_radius=2.0)

    all_results = []
    scenario_result_map = {}

    overlay_dir = os.path.join(output_dir, "overlays")
    animation_dir = os.path.join(output_dir, "animations")
    os.makedirs(overlay_dir, exist_ok=True)
    if make_animation:
        os.makedirs(animation_dir, exist_ok=True)

    figures = []
    animation_figures = []

    for scenario_name, scenario_data in scenarios.items():
        print(f"\n{'=' * 60}")
        print(f"Scenario: {_scenario_label(scenario_name)}")
        print(f"{'=' * 60}")

        ox = scenario_data["ox"]
        oy = scenario_data["oy"]
        sx, sy = scenario_data["sx"], scenario_data["sy"]
        gx, gy = scenario_data["gx"], scenario_data["gy"]

        scenario_results = []
        for planner_name, planner_cls in PLANNERS:
            print(f"  Running {planner_name}...", end=" ", flush=True)
            planner = planner_cls(config=config)
            result = _run_planner(planner, sx, sy, gx, gy, ox, oy, scenario_data)
            if result.success and result.path_x:
                result.path_length = calc_path_length(result.path_x, result.path_y)
                result.smoothness = calc_mean_heading_change(result.path_x, result.path_y)
            all_results.append((scenario_name, result))
            scenario_results.append((planner_name, result))

            status = "OK" if result.success else "FAIL"
            print(
                f"{status} | Length={result.path_length:.1f}m | "
                f"Time={result.planning_time * 1000:.1f}ms | Nodes={result.explored_nodes}"
            )

        scenario_result_map[scenario_name] = scenario_results
        overlay_path = os.path.join(output_dir, f"{_slugify(scenario_name)}_comparison.png")
        overlay_figure = plot_overlay_comparison(
            scenario_name,
            scenario_data,
            scenario_results,
            overlay_path,
            show=show,
        )
        if overlay_figure is not None:
            figures.append(overlay_figure)

        overlay_copy_path = os.path.join(overlay_dir, f"{_slugify(scenario_name)}_all_planners.png")
        plot_overlay_comparison(
            scenario_name,
            scenario_data,
            scenario_results,
            overlay_copy_path,
            show=False,
        )

        if make_animation or show_animation:
            animation_path = None
            if make_animation:
                animation_path = os.path.join(animation_dir, f"{_slugify(scenario_name)}_all_planners.gif")
            animation_figure, _ = animate_overlay_comparison(
                scenario_name,
                scenario_data,
                scenario_results,
                output_path=animation_path,
                show=show_animation,
            )
            if animation_figure is not None:
                animation_figures.append(animation_figure)

    _plot_trajectory_overview(scenarios, scenario_result_map, output_dir, show)
    _plot_summary(all_results, scenarios, output_dir, show)

    print(f"\n{'=' * 60}")
    print("Summary")
    print(f"{'=' * 60}")
    planning_results = [result for _, result in all_results]
    print(format_table(planning_results))

    _save_csv(all_results, output_dir)

    if show and figures:
        plt.show()
        for figure in figures:
            plt.close(figure)

    if show_animation and animation_figures:
        plt.show()
        for figure in animation_figures:
            plt.close(figure)

    print(f"\nResults saved to {output_dir}/")
    print(f"  - Static comparisons: {len(scenarios)}")
    print(f"  - Overlay snapshots: {overlay_dir}")
    if make_animation:
        print(f"  - Scenario GIFs: {animation_dir}")


def _plot_trajectory_overview(scenarios, scenario_result_map, output_dir, show):
    context = nullcontext() if show else _suppress_interactive_windows()
    with context:
        figure, axes = plt.subplots(1, len(scenarios), figsize=(18, 5.6))
        if len(scenarios) == 1:
            axes = [axes]

        for axis, (scenario_name, scenario_data) in zip(axes, scenarios.items()):
            _draw_scene(axis, scenario_data)
            for planner_name, result in scenario_result_map[scenario_name]:
                color = COLORS.get(planner_name, "tab:gray")
                if result.success:
                    axis.plot(result.path_x, result.path_y, color=color, linewidth=1.8, label=planner_name)
            axis.set_title(_scenario_label(scenario_name))
            axis.legend(frameon=False, ncol=2, fontsize=7, loc="best")

        figure.suptitle("Trajectory Overview Across Scenarios", fontsize=15, y=1.02)
        figure.tight_layout()
        figure.savefig(os.path.join(output_dir, "trajectory_overview.png"), dpi=160, bbox_inches="tight")
        if show:
            plt.show()
        plt.close(figure)


def _plot_summary(all_results, scenarios, output_dir, show):
    scenario_names = list(scenarios.keys())
    planner_names = [name for name, _ in PLANNERS]
    metrics = {
        "Path Length (m)": lambda result: result.path_length if result.success else 0.0,
        "Mean Heading Change (rad)": lambda result: result.smoothness if result.success else 0.0,
        "Planning Time (ms)": lambda result: result.planning_time * 1000.0,
        "Explored Nodes": lambda result: result.explored_nodes,
    }

    for metric_name, metric_fn in metrics.items():
        context = nullcontext() if show else _suppress_interactive_windows()
        with context:
            figure, axis = plt.subplots(figsize=(10, 6))
            x_axis = np.arange(len(scenario_names))
            width = max(0.05, 0.84 / max(len(planner_names), 1))
            offsets = np.linspace(
                -width * (len(planner_names) - 1) / 2.0,
                width * (len(planner_names) - 1) / 2.0,
                len(planner_names),
            )

            for i, planner_name in enumerate(planner_names):
                values = []
                for scenario_name in scenario_names:
                    for stored_scenario, result in all_results:
                        if stored_scenario == scenario_name and result.planner == planner_name:
                            values.append(metric_fn(result))
                            break
                axis.bar(
                    x_axis + offsets[i],
                    values,
                    width,
                    label=planner_name,
                    color=COLORS.get(planner_name, "tab:gray"),
                )

            axis.set_xlabel("Scenario")
            axis.set_ylabel(metric_name)
            axis.set_title(f"Planner Comparison - {metric_name}")
            axis.set_xticks(x_axis)
            axis.set_xticklabels([_scenario_label(name) for name in scenario_names])
            axis.legend(frameon=False, ncol=4, fontsize=8)
            axis.grid(True, axis="y", alpha=0.3)
            figure.tight_layout()

            slug = metric_name.lower().replace(" ", "_").replace("(", "").replace(")", "")
            figure.savefig(os.path.join(output_dir, f"summary_{slug}.png"), dpi=150, bbox_inches="tight")
            if show:
                plt.show()
            plt.close(figure)


def _save_csv(all_results, output_dir):
    path = os.path.join(output_dir, "results.csv")
    fieldnames = [
        "Scenario",
        "Planner",
        "Success",
        "Path Length (m)",
        "Mean Heading Change (rad)",
        "Planning Time (ms)",
        "Explored Nodes",
    ]
    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for scenario_name, result in all_results:
            writer.writerow({"Scenario": _scenario_label(scenario_name), **result.to_dict()})


def build_compare_arg_parser():
    parser = argparse.ArgumentParser(description="Path Planner Comparison Benchmark")
    parser.add_argument("--output-dir", default="outputs_planning", help="Output directory")
    parser.add_argument("--show", action="store_true", help="Display figures interactively")
    parser.add_argument("--animate", action="store_true", help="Save GIF animations")
    parser.add_argument("--show-animation", action="store_true", help="Display animations interactively")
    return parser


def main():
    args = build_compare_arg_parser().parse_args()
    run_benchmark(
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
    )


if __name__ == "__main__":
    main()
