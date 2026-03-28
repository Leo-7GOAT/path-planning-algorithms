"""
Path Planner Comparison Benchmark

Runs Dijkstra, A*, RRT, RRT*, and PRM across multiple obstacle scenarios,
collects metrics (path length, planning time, explored nodes), and produces
comparison visualizations.

Usage:
    python Compare_planner.py                    # save figures only
    python Compare_planner.py --show             # display figures interactively
    python Compare_planner.py --animate          # save GIF animations
    python Compare_planner.py --show-animation   # display animations live
"""

import argparse
import csv
import os
import shutil

import matplotlib.pyplot as plt
import numpy as np

from planners import (
    AStarPlanner,
    DijkstraPlanner,
    PRMPlanner,
    RRTPlanner,
    RRTStarPlanner,
)
from planners.common import (
    PlannerConfig,
    PlanningResult,
    build_scenario_obstacles,
    format_table,
)

COLORS = {
    "Dijkstra": "tab:red",
    "A*": "tab:blue",
    "RRT": "tab:green",
    "RRT*": "tab:purple",
    "PRM": "tab:brown",
}

PLANNERS = [
    ("Dijkstra", DijkstraPlanner),
    ("A*", AStarPlanner),
    ("RRT", RRTPlanner),
    ("RRT*", RRTStarPlanner),
    ("PRM", PRMPlanner),
]


def _slugify(name):
    return name.lower().replace(" ", "_").replace("*", "_star")


def _prepare_output_dir(output_dir):
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)


def plot_scenario_result(ax, scenario_name, scenario_data, planner_name, result):
    """Plot a single planner result on the given axes."""
    ox = scenario_data["ox"]
    oy = scenario_data["oy"]
    sx, sy = scenario_data["sx"], scenario_data["sy"]
    gx, gy = scenario_data["gx"], scenario_data["gy"]

    ax.plot(ox, oy, ".k", markersize=1, label="Obstacles")
    ax.plot(sx, sy, "og", markersize=10, label="Start")
    ax.plot(gx, gy, "sr", markersize=10, label="Goal")

    if result.success:
        color = COLORS.get(planner_name, "tab:blue")
        ax.plot(result.path_x, result.path_y, "-", color=color, linewidth=2, label="Path")

    ax.set_title(f"{planner_name} — {scenario_name}\n"
                 f"Length: {result.path_length:.1f}m | "
                 f"Time: {result.planning_time * 1000:.1f}ms | "
                 f"Nodes: {result.explored_nodes}")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)


def run_benchmark(output_dir="outputs_planning", show=False):
    _prepare_output_dir(output_dir)
    scenarios = build_scenario_obstacles()
    config = PlannerConfig(resolution=1.0, robot_radius=2.0)

    all_results = []

    for scenario_name, scenario_data in scenarios.items():
        print(f"\n{'=' * 60}")
        print(f"Scenario: {scenario_name}")
        print(f"{'=' * 60}")

        ox = scenario_data["ox"]
        oy = scenario_data["oy"]
        sx, sy = scenario_data["sx"], scenario_data["sy"]
        gx, gy = scenario_data["gx"], scenario_data["gy"]

        n_planners = len(PLANNERS)
        fig, axes = plt.subplots(1, n_planners, figsize=(6 * n_planners, 6))
        if n_planners == 1:
            axes = [axes]

        for idx, (planner_name, planner_cls) in enumerate(PLANNERS):
            print(f"  Running {planner_name}...", end=" ", flush=True)
            planner = planner_cls(config=config)
            result = planner.plan(sx, sy, gx, gy, ox, oy)
            all_results.append((scenario_name, result))

            status = "OK" if result.success else "FAIL"
            print(f"{status} | "
                  f"Length={result.path_length:.1f}m | "
                  f"Time={result.planning_time * 1000:.1f}ms | "
                  f"Nodes={result.explored_nodes}")

            plot_scenario_result(axes[idx], scenario_name, scenario_data, planner_name, result)

        fig.suptitle(f"Planner Comparison — {scenario_name}", fontsize=16, y=1.02)
        fig.tight_layout()
        fig.savefig(
            os.path.join(output_dir, f"{_slugify(scenario_name)}_comparison.png"),
            dpi=150, bbox_inches="tight",
        )
        if show:
            plt.show()
        plt.close(fig)

    # Summary bar charts
    _plot_summary(all_results, scenarios, output_dir, show)

    # Print results table
    print(f"\n{'=' * 60}")
    print("Summary")
    print(f"{'=' * 60}")
    planning_results = [r for _, r in all_results]
    print(format_table(planning_results))

    # Save CSV
    _save_csv(all_results, output_dir)

    print(f"\nResults saved to {output_dir}/")


def _plot_summary(all_results, scenarios, output_dir, show):
    """Create bar chart comparisons across all scenarios."""
    scenario_names = list(scenarios.keys())
    planner_names = [name for name, _ in PLANNERS]
    metrics = {
        "Path Length (m)": lambda r: r.path_length if r.success else 0,
        "Planning Time (ms)": lambda r: r.planning_time * 1000,
        "Explored Nodes": lambda r: r.explored_nodes,
    }

    for metric_name, metric_fn in metrics.items():
        fig, ax = plt.subplots(figsize=(10, 6))
        x = np.arange(len(scenario_names))
        width = 0.15
        offsets = np.linspace(
            -width * (len(planner_names) - 1) / 2,
            width * (len(planner_names) - 1) / 2,
            len(planner_names),
        )

        for i, pname in enumerate(planner_names):
            values = []
            for sname in scenario_names:
                for s, r in all_results:
                    if s == sname and r.planner == pname:
                        values.append(metric_fn(r))
                        break
            ax.bar(x + offsets[i], values, width, label=pname,
                   color=COLORS.get(pname, "tab:gray"))

        ax.set_xlabel("Scenario")
        ax.set_ylabel(metric_name)
        ax.set_title(f"Planner Comparison — {metric_name}")
        ax.set_xticks(x)
        ax.set_xticklabels(scenario_names)
        ax.legend()
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()

        slug = metric_name.lower().replace(" ", "_").replace("(", "").replace(")", "")
        fig.savefig(os.path.join(output_dir, f"summary_{slug}.png"), dpi=150, bbox_inches="tight")
        if show:
            plt.show()
        plt.close(fig)


def _save_csv(all_results, output_dir):
    path = os.path.join(output_dir, "results.csv")
    fieldnames = [
        "Scenario", "Planner", "Success", "Path Length (m)",
        "Planning Time (ms)", "Explored Nodes",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for scenario_name, r in all_results:
            writer.writerow({
                "Scenario": scenario_name,
                **r.to_dict(),
            })


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Path Planner Comparison Benchmark")
    parser.add_argument("--output-dir", default="outputs_planning", help="Output directory")
    parser.add_argument("--show", action="store_true", help="Display figures interactively")
    return parser


def main():
    args = build_arg_parser().parse_args()
    run_benchmark(output_dir=args.output_dir, show=args.show)


if __name__ == "__main__":
    main()
