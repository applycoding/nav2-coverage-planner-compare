# SPDX-License-Identifier: Apache-2.0
import math
import platform
import time
from pathlib import Path

import ncp_core
import pytest

MAP_PLANNER_CASES = [
    ("open", ncp_core.PlannerId.AStar, (0.5, 0.5), (1.5, 1.0)),
    ("open", ncp_core.PlannerId.Dijkstra, (0.5, 0.5), (1.5, 1.0)),
    ("open", ncp_core.PlannerId.LazyThetaStar, (0.5, 0.5), (1.5, 1.0)),
    ("open", ncp_core.PlannerId.RrtStar, (0.5, 0.5), (1.8, 1.8)),
    ("office", ncp_core.PlannerId.AStar, (0.5, 1.05), (2.5, 1.05)),
    ("narrow_pass", ncp_core.PlannerId.AStar, (0.5, 1.05), (2.5, 1.05)),
]


def _bench_row(map_id: str, planner, start, goal) -> dict[str, float | str]:
    root = Path(__file__).resolve().parents[2] / "fixtures" / "maps" / map_id
    grid = ncp_core.load_grid(str(root / "map.pgm"), str(root / "map.yaml"))
    params = ncp_core.CostmapParams()
    params.robot_radius_m = 0.15
    params.inflation_radius_m = 0.15
    params.cost_decay = 0

    t0 = time.perf_counter()
    cmap = ncp_core.make_costmap(grid, params)
    load_ms = (time.perf_counter() - t0) * 1000.0

    req = ncp_core.PlanRequest()
    req.planner = planner
    req.start = ncp_core.WorldPoint()
    req.start.x, req.start.y = start
    req.goal = ncp_core.WorldPoint()
    req.goal.x, req.goal.y = goal
    if planner == ncp_core.PlannerId.RrtStar:
        req.rrt_seed = 42
        req.rrt_max_iterations = 3000

    t1 = time.perf_counter()
    res = ncp_core.plan_global(cmap, req)
    search_ms = (time.perf_counter() - t1) * 1000.0

    return {
        "map": map_id,
        "planner": int(planner),
        "load_ms": load_ms,
        "search_ms": search_ms,
        "length_m": res.metrics.length_m,
        "compute_ms": res.metrics.compute_ms,
    }


def test_benchmark_table(capsys) -> None:
    rows = [_bench_row(*args) for args in MAP_PLANNER_CASES]
    assert len(rows) == len(MAP_PLANNER_CASES)
    for row in rows:
        assert row["load_ms"] > 0 and math.isfinite(row["load_ms"])
        assert row["search_ms"] > 0 and math.isfinite(row["search_ms"])
        assert row["length_m"] >= 0 and math.isfinite(row["length_m"])
        assert row["compute_ms"] > 0 and math.isfinite(row["compute_ms"])

    plat = f"{platform.system()}/{platform.machine()} py{platform.python_version()}"
    print(f"\nplatform\t{plat}")
    for row in rows:
        print(
            f"{row['map']}\tplanner={row['planner']}\tload_ms={row['load_ms']:.3f}"
            f"\tsearch_ms={row['search_ms']:.3f}\tlength_m={row['length_m']:.4f}"
        )
    capsys.readouterr()
