#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate PGM/YAML fixture maps and cases.json for ncp_dump_fixture."""
from __future__ import annotations

import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "maps"


def write_pgm(path: Path, w: int, h: int, occ: list[int]) -> None:
    with path.open("wb") as f:
        f.write(f"P5\n# ncp fixture\n{w} {h}\n255\n".encode())
        for v in occ:
            f.write(bytes([v]))


def write_yaml(path: Path) -> None:
    path.write_text(
        "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )


def main() -> None:
    maps: dict[str, dict] = {}

    w, h = 20, 20
    occ = [0] * (w * h)
    maps["open"] = {"w": w, "h": h, "occ": occ}

    w, h = 30, 20
    occ = [0] * (w * h)
    for y in range(6, 18):
        occ[y * w + 15] = 255
    for x in (14, 15, 16):
        occ[10 * w + x] = 0
    office_occ = occ[:]
    maps["office"] = {"w": w, "h": h, "occ": office_occ}
    maps["narrow_pass"] = {"w": w, "h": h, "occ": office_occ[:], "gap": 15}
    maps["narrow_block"] = {"w": w, "h": h, "occ": office_occ[:]}

    w, h = 25, 25
    occ = [0] * (w * h)
    for y in range(8, 17):
        for x in range(8, 17):
            occ[y * w + x] = 255
    maps["hole"] = {"w": w, "h": h, "occ": occ}

    w, h = 20, 20
    occ = [0] * (w * h)
    for y in range(h):
        occ[y * w + 10] = 255
    maps["unreachable"] = {"w": w, "h": h, "occ": occ}

    w, h = 20, 3
    occ = [0] * (w * h)
    for x in range(w):
        occ[1 * w + x] = 128
    maps["unknown_row"] = {"w": w, "h": h, "occ": occ}

    maps["tiny"] = {"w": 1, "h": 1, "occ": [0]}

    cases_data: dict[str, dict] = {
        "open": {
            "costmap_params": {
                "robot_radius_m": 0.15,
                "inflation_radius_m": 0.15,
                "cost_decay": 0,
                "w_cost": 1,
            },
            "cases": [
                {
                    "id": "astar_to_goal",
                    "planner": "astar",
                    "start": {"x": 0.5, "y": 0.5},
                    "goal": {"x": 1.5, "y": 1.0},
                },
                {
                    "id": "dijkstra_to_goal",
                    "planner": "dijkstra",
                    "start": {"x": 0.5, "y": 0.5},
                    "goal": {"x": 1.5, "y": 1.0},
                },
                {
                    "id": "lazy_theta",
                    "planner": "lazy_theta",
                    "start": {"x": 0.5, "y": 0.5},
                    "goal": {"x": 1.5, "y": 1.0},
                },
                {
                    "id": "rrt_iter",
                    "planner": "rrt_star",
                    "start": {"x": 0.5, "y": 0.5},
                    "goal": {"x": 1.8, "y": 1.8},
                    "rrt_seed": 42,
                    "rrt_max_iterations": 3000,
                },
            ],
        },
        "office": {
            "costmap_params": {
                "robot_radius_m": 0.15,
                "inflation_radius_m": 0.15,
                "cost_decay": 0,
            },
            "cases": [
                {
                    "id": "corridor_astar",
                    "planner": "astar",
                    "start": {"x": 0.5, "y": 1.05},
                    "goal": {"x": 2.5, "y": 1.05},
                }
            ],
        },
        "hole": {
            "costmap_params": {"robot_radius_m": 0.12, "inflation_radius_m": 0.12, "cost_decay": 0},
            "cases": [
                {
                    "id": "coverage_hole",
                    "type": "coverage",
                    "start": {"x": 0.5, "y": 0.5},
                    "linker": "astar",
                    "overlap_m": 0.02,
                    "coverage_success_threshold": 0.95,
                }
            ],
        },
        "narrow_pass": {
            "costmap_params": {"robot_radius_m": 0.15, "inflation_radius_m": 0.15, "cost_decay": 0},
            "cases": [
                {
                    "id": "through_gap",
                    "planner": "astar",
                    "start": {"x": 0.5, "y": 1.05},
                    "goal": {"x": 2.5, "y": 1.05},
                }
            ],
        },
        "narrow_block": {
            "costmap_params": {"robot_radius_m": 0.22, "inflation_radius_m": 0.22, "cost_decay": 0},
            "cases": [
                {
                    "id": "blocked_gap",
                    "planner": "astar",
                    "start": {"x": 0.5, "y": 1.05},
                    "goal": {"x": 2.5, "y": 1.05},
                }
            ],
        },
        "unreachable": {
            "costmap_params": {"robot_radius_m": 0.1, "inflation_radius_m": 0.1, "cost_decay": 0},
            "cases": [
                {
                    "id": "astar_unreachable",
                    "planner": "astar",
                    "start": {"x": 0.5, "y": 1.0},
                    "goal": {"x": 1.5, "y": 1.0},
                },
                {
                    "id": "goal_in_obstacle",
                    "planner": "astar",
                    "start": {"x": 0.5, "y": 1.0},
                    "goal": {"x": 1.05, "y": 1.0},
                },
                {
                    "id": "rrt_timeout",
                    "planner": "rrt_star",
                    "start": {"x": 0.5, "y": 1.0},
                    "goal": {"x": 1.5, "y": 1.0},
                    "rrt_seed": 7,
                    "rrt_max_iterations": 500,
                },
            ],
        },
        "unknown_row": {
            "costmap_params": {"robot_radius_m": 0.08, "inflation_radius_m": 0.08, "cost_decay": 0},
            "cases": [
                {
                    "id": "lethal_unknown",
                    "planner": "astar",
                    "unknown_policy": "lethal",
                    "start": {"x": 0.5, "y": 0.05},
                    "goal": {"x": 1.5, "y": 0.05},
                },
                {
                    "id": "free_unknown",
                    "planner": "astar",
                    "unknown_policy": "free",
                    "start": {"x": 0.5, "y": 0.05},
                    "goal": {"x": 1.5, "y": 0.05},
                },
            ],
        },
        "tiny": {
            "costmap_params": {"robot_radius_m": 0.05, "inflation_radius_m": 0.05, "cost_decay": 0},
            "cases": [
                {
                    "id": "start_eq_goal",
                    "planner": "astar",
                    "start": {"x": 0.05, "y": 0.05},
                    "goal": {"x": 0.05, "y": 0.05},
                }
            ],
        },
    }

    import copy

    office_large = copy.deepcopy(cases_data["office"])
    office_large["costmap_params"]["inflation_radius_m"] = 0.22
    office_large["costmap_params"]["robot_radius_m"] = 0.22
    office_large["cases"][0]["id"] = "corridor_inflate"
    maps["office_large_inflation"] = maps["office"]
    cases_data["office_large_inflation"] = office_large

    for name, spec in maps.items():
        d = ROOT / name
        d.mkdir(parents=True, exist_ok=True)
        write_pgm(d / "map.pgm", spec["w"], spec["h"], spec["occ"])
        write_yaml(d / "map.yaml")
        if name in cases_data:
            (d / "cases.json").write_text(
                json.dumps(cases_data[name], indent=2) + "\n", encoding="utf-8"
            )

    (ROOT.parent / "README.md").write_text(
        "# Fixtures\n\nGolden `expected.json` files are produced by "
        "`build/ncp_dump_fixture fixtures/maps/<id>`. Hand-derived cases live in "
        "`cpp/test`.\n",
        encoding="utf-8",
    )
    print("wrote fixtures under", ROOT)


if __name__ == "__main__":
    main()
