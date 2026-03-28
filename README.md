# Path Planning Algorithms Benchmark

Comparative benchmark and visualization for classical path-planning algorithms in Python.

这是一个面向自动驾驶规划算法岗位展示的路径规划 benchmark 项目。项目统一实现并对比多种经典规划方法，在不同障碍物场景上进行量化评测，并支持多算法同图展示、场景 GIF 动画和结果图表自动导出。

## At a Glance

- `5` 个代表性规划器：`Dijkstra / A* / RRT / RRT* / PRM`
- `3` 个典型场景：`Corridor / Scattered / Narrow Passage`
- 支持统一地图、统一碰撞检测、统一指标统计
- 支持每个场景多算法同图对比和联跑 GIF
- 适合作为自动驾驶规划岗位的项目展示与面试讲解材料

## Planning Methods Landscape

路径规划方法大致可以分为这几类：

- `Search-based`：`BFS / Dijkstra / A* / JPS / D* / D* Lite`
- `Sampling-based`：`PRM / RRT / RRT* / Informed RRT*`
- `Optimization-based`：`CHOMP / STOMP / TrajOpt / MPC-based planning`
- `Kinematic / lattice-based`：`Hybrid A* / State Lattice / Frenet`
- `Reactive / local methods`：`APF / DWA / TEB`

这版 benchmark 重点实现了 5 个最适合做统一对比、又最有代表性的经典全局规划器：

- `Dijkstra`
- `A*`
- `RRT`
- `RRT*`
- `PRM`

## Visualization

### Corridor

![Corridor live demo](assets/readme/corridor_all_planners.gif)

![Corridor comparison](assets/readme/corridor_all_planners.png)

### Scattered

![Scattered live demo](assets/readme/scattered_all_planners.gif)

![Scattered comparison](assets/readme/scattered_all_planners.png)

### Narrow Passage

![Narrow Passage live demo](assets/readme/narrow_passage_all_planners.gif)

![Narrow Passage comparison](assets/readme/narrow_passage_all_planners.png)

### Metric Dashboard

**Path Length**

![Path length comparison](assets/readme/summary_path_length_m.png)

**Planning Time**

![Planning time comparison](assets/readme/summary_planning_time_ms.png)

**Explored Nodes**

![Explored nodes comparison](assets/readme/summary_explored_nodes.png)

## Planners

| Planner | Category | Description |
| --- | --- | --- |
| **Dijkstra** | Search-based | 经典图搜索算法，均匀扩展搜索，保证最短路径。 |
| **A\*** | Search-based | 在 Dijkstra 基础上加入启发式函数，引导搜索朝目标推进，通常能显著减少搜索节点数。 |
| **RRT** | Sampling-based | 快速探索随机树，通过随机采样增量扩展搜索树，优先求可行解。 |
| **RRT\*** | Sampling-based | RRT 的渐近最优扩展，通过重连机制持续优化路径质量。 |
| **PRM** | Sampling-based | 概率路线图方法，先采样建图，再进行图搜索查询，适合多次路径查询场景。 |

## Scenarios

| Scenario | Description |
| --- | --- |
| **Corridor** | 迷宫式走廊，测试绕行与长路径搜索能力。 |
| **Scattered** | 随机散布障碍物，测试复杂环境下的避障和路径质量。 |
| **Narrow Passage** | 中央仅有窄通道，测试规划器发现狭窄可通行区域的能力。 |

## Benchmark Results

| Scenario | Planner | Path Length (m) | Planning Time (ms) | Explored Nodes |
| --- | --- | ---: | ---: | ---: |
| Corridor | Dijkstra | 120.71 | 172.98 | 2627 |
| Corridor | A* | 120.71 | 199.50 | 1888 |
| Corridor | RRT | 168.45 | 187.04 | 258 |
| Corridor | RRT* | 115.83 | 769.66 | 1469 |
| Corridor | PRM | 122.01 | 361.61 | 602 |
| Scattered | Dijkstra | 80.08 | 489.91 | 3035 |
| Scattered | A* | 80.08 | 639.21 | 840 |
| Scattered | RRT | 104.00 | 636.91 | 91 |
| Scattered | RRT* | 75.49 | 1212.19 | 1409 |
| Scattered | PRM | 83.66 | 746.49 | 602 |
| Narrow Passage | Dijkstra | 40.00 | 66.47 | 1191 |
| Narrow Passage | A* | 40.00 | 66.97 | 40 |
| Narrow Passage | RRT | 48.05 | 66.88 | 33 |
| Narrow Passage | RRT* | 40.08 | 564.78 | 1334 |
| Narrow Passage | PRM | 42.37 | 221.54 | 602 |

## Experiment Analysis

### Overall

- 5 种规划器在 3 个场景上都成功找到路径，说明统一 benchmark 框架与碰撞检测逻辑稳定可用。
- `Search-based` 方法在路径最优性上表现稳定，而 `Sampling-based` 方法在连续空间中更容易产生更短或更灵活的路径。
- `Narrow Passage` 对启发式和采样能力区分最明显，`Corridor` 与 `Scattered` 更适合观察路径质量和规划时间 trade-off。

### By Planner

- **Dijkstra**：最适合作为最优性 baseline。优点是稳定可靠、一定能找到最短路径；缺点是探索节点多、效率偏低。
- **A\***：路径质量与 Dijkstra 一致，但通常探索节点更少。当前实现里在 `Narrow Passage` 上只探索 `40` 个节点，对比 Dijkstra 的 `1191`，启发式优势非常明显。
- **RRT**：很快得到可行解，探索节点极少，但路径通常更长、更锯齿化。
- **RRT\***：路径质量通常最好或接近最好，但优化代价明显更高，规划时间最长。
- **PRM**：性能居中，路径质量和规划时间都比较平衡，适合多次 query 的应用设定。

### By Scenario

- **Corridor**：`RRT*` 得到最短路径，说明采样 + 重连在连续空间长走廊中有优势；`RRT` 可行但路径明显更绕。
- **Scattered**：`RRT*` 仍然最优，`A*` 比 Dijkstra 探索节点显著更少，符合启发式搜索理论。
- **Narrow Passage**：`A*` 与 Dijkstra 路径同为最优，但搜索效率远高于 Dijkstra；`RRT` 也能快速穿过窄通道，不过路径质量偏差更大。

### Theory vs Practice

- 结果整体符合经典理论预期：
- `Dijkstra / A*` 在栅格地图上给出最优路径；
- `RRT` 更擅长快速找到可行解，而不是最好解；
- `RRT*` 通过 rewiring 提升路径质量，但时间代价更高；
- `PRM` 作为 roadmap 方法，表现通常处于中间位置。

需要注意的是：

- 这里的 wall-clock 时间是 Python 实现下的工程结果，不完全等于算法理论复杂度排名。
- `A*` 虽然通常会减少探索节点，但在 Python 层面的数据结构和障碍布局差异下，时间优势不一定在每个场景都绝对成立。

## Quick Start

### 1. Install

```bash
pip install -r requirements.txt
```

### 2. Run the benchmark

```bash
python Compare_planner.py
```

静默运行并输出结果到 `outputs_planning/`。

如果你想弹出静态图：

```bash
python Compare_planner.py --show
```

如果你想导出每个场景的多算法同图 GIF：

```bash
python Compare_planner.py --animate
```

如果你想弹出动画窗口：

```bash
python Compare_planner.py --show-animation
```

## File Guide

| File | Purpose |
| --- | --- |
| `Compare_planner.py` | 规划 benchmark 总入口，负责调度算法、输出结果、生成静态图和 GIF。 |
| `planners/` | 规划器实现目录。 |
| `planners/Dijkstra.py` | Dijkstra 最短路径规划器。 |
| `planners/Astar.py` | A* 启发式搜索规划器。 |
| `planners/RRT.py` | RRT 快速探索随机树规划器。 |
| `planners/RRT_Star.py` | RRT* 渐近最优规划器。 |
| `planners/PRM.py` | PRM 概率路线图规划器。 |
| `planners/common.py` | 公共模块：栅格地图、障碍物碰撞检测、场景构造、结果格式。 |
| `requirements.txt` | 项目依赖（numpy, matplotlib）。 |
| `assets/readme/` | README 展示图片和 GIF。 |
| `outputs_planning/` | benchmark 运行输出目录。 |

## Generated Outputs

运行 `python Compare_planner.py` 后自动生成：

- `outputs_planning/corridor_comparison.png`
- `outputs_planning/scattered_comparison.png`
- `outputs_planning/narrow_passage_comparison.png`
- `outputs_planning/overlays/`
- `outputs_planning/results.csv`
- `outputs_planning/summary_path_length_m.png`
- `outputs_planning/summary_planning_time_ms.png`
- `outputs_planning/summary_explored_nodes.png`

如果启用 `--animate`，还会生成：

- `outputs_planning/animations/corridor_all_planners.gif`
- `outputs_planning/animations/scattered_all_planners.gif`
- `outputs_planning/animations/narrow_passage_all_planners.gif`

## Resume-oriented Summary

> 搭建了一个自包含的路径规划算法 benchmark，统一实现并评测 Dijkstra、A*、RRT、RRT*、PRM 5 类经典规划算法，基于统一地图与碰撞检测框架，在走廊、散布障碍物和窄通道 3 个场景上对路径长度、规划时间和搜索效率进行量化对比，并支持多算法同图可视化与 GIF 动态展示。

## License

This project is released under the MIT License. See `LICENSE` for details.
