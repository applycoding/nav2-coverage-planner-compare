# SPDX-License-Identifier: Apache-2.0
import json
import math
from pathlib import Path

import ncp_core
import pytest

PLANNER = {
    "astar": ncp_core.PlannerId.AStar,
    "dijkstra": ncp_core.PlannerId.Dijkstra,
    "rrt_star": ncp_core.PlannerId.RrtStar,
    "lazy_theta": ncp_core.PlannerId.LazyThetaStar,
}


def _params_from_cases(cases: dict) -> ncp_core.CostmapParams:
    p = ncp_core.CostmapParams()
    cp = cases.get("costmap_params", {})
    p.robot_radius_m = cp.get("robot_radius_m", 0.15)
    p.inflation_radius_m = cp.get("inflation_radius_m", p.robot_radius_m)
    p.cost_decay = cp.get("cost_decay", 0.0)
    p.w_cost = cp.get("w_cost", 1.0)
    pol = cp.get("unknown_policy", "lethal")
    p.unknown_policy = (
        ncp_core.UnknownPolicy.Free if pol == "free" else ncp_core.UnknownPolicy.Lethal
    )
    return p


def _point(d: dict) -> ncp_core.WorldPoint:
    pt = ncp_core.WorldPoint()
    pt.x = d["x"]
    pt.y = d["y"]
    return pt


def _near(a: float, b: float, eps: float = 1e-6) -> bool:
    return abs(a - b) <= eps


def _compare_result(actual: ncp_core.PlanResult, expected: dict, case: dict, cmap) -> None:
    assert actual.success == expected["success"], (actual.reason, expected.get("reason"))
    exp_reason = expected.get("reason") or ""
    if actual.success:
        assert actual.reason in ("", exp_reason)
    else:
        assert actual.reason == exp_reason

    assert _near(actual.metrics.length_m, expected["length_m"])
    assert _near(actual.metrics.objective_cost, expected["objective_cost"])

    assert len(actual.path) == len(expected["path"])
    for ap, ep in zip(actual.path, expected["path"], strict=True):
        assert _near(ap.x, ep["x"])
        assert _near(ap.y, ep["y"])

    if "rrt_vertices" in expected:
        assert actual.rrt_vertices == expected["rrt_vertices"]

    if expected.get("coverage_ratio_reachable", 0.0) > 0 or case.get("type") == "coverage":
        assert _near(
            actual.metrics.coverage_ratio_reachable or 0.0,
            expected.get("coverage_ratio_reachable", 0.0),
            1e-4,
        )
        assert _near(
            actual.metrics.coverage_ratio_total or 0.0,
            expected.get("coverage_ratio_total", 0.0),
            1e-4,
        )

    if "unreachable_free_ratio" in expected:
        assert _near(actual.unreachable_free_ratio, expected["unreachable_free_ratio"], 1e-4)

    if "clearance_m" in expected:
        exp_c = expected["clearance_m"]
        act_c = actual.metrics.clearance_m
        if exp_c is None:
            assert math.isnan(act_c)
        else:
            assert _near(act_c, exp_c, 1e-4)

    assert "compute_ms" not in expected

    if actual.success and "goal" in case:
        vr = ncp_core.validate_path(
            cmap,
            actual.path,
            _point(case["start"]),
            _point(case["goal"]),
            max(1e6, expected["length_m"] * 2.0 + 1.0),
        )
        assert vr["ok"], vr["message"]


@pytest.mark.parametrize(
    "map_id",
    [
        "hole",
        "narrow_block",
        "narrow_pass",
        "office",
        "office_large_inflation",
        "open",
        "tiny",
        "unknown_row",
        "unreachable",
    ],
)
def test_fixture_goldens(map_id: str, fixtures_root: Path) -> None:
    mdir = fixtures_root / map_id
    cases = json.loads((mdir / "cases.json").read_text(encoding="utf-8"))
    expected = json.loads((mdir / "expected.json").read_text(encoding="utf-8"))
    pmap = mdir / "map.pgm"
    ymap = mdir / "map.yaml"
    if not pmap.exists():
        pytest.skip("no pgm")

    grid = ncp_core.load_grid(str(pmap), str(ymap))

    for case in cases["cases"]:
        case_params = dict(cases.get("costmap_params", {}))
        if "unknown_policy" in case:
            case_params["unknown_policy"] = case["unknown_policy"]
        merged = {"costmap_params": case_params}
        cmap = ncp_core.make_costmap(grid, _params_from_cases(merged))
        cid = case["id"]
        exp = expected[cid]
        if case.get("type") == "coverage":
            req = ncp_core.CoverageRequest()
            req.start = _point(case["start"])
            req.linker = PLANNER[case.get("linker", "astar")]
            req.overlap_m = case.get("overlap_m", 0.0)
            req.coverage_success_threshold = case.get("coverage_success_threshold", 0.95)
            res = ncp_core.plan_coverage(cmap, req)
        else:
            req = ncp_core.PlanRequest()
            req.planner = PLANNER[case["planner"]]
            req.start = _point(case["start"])
            req.goal = _point(case["goal"])
            req.rrt_seed = case.get("rrt_seed", 1)
            req.rrt_max_iterations = case.get("rrt_max_iterations", 5000)
            req.rrt_max_time_ms = case.get("rrt_max_time_ms", 0)
            res = ncp_core.plan_global(cmap, req)
        _compare_result(res, exp, case, cmap)
