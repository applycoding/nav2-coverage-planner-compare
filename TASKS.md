# Build tasks — nav2-coverage-planner-compare

Execute in order. Follow PLAN.md. Deviate only if a pin or test proves a step broken, and record why. License is **Apache-2.0**, not MIT. Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

Each task has acceptance checks. A task is done only when those checks pass.

## 1. Repository skeleton and license

- Create the tree in PLAN.md (empty dirs may hold `.gitkeep`).
- Fetch `https://www.apache.org/licenses/LICENSE-2.0.txt` into `LICENSE` and confirm byte equality (including the appendix).
- Write `NOTICE` starting with `nav2-coverage-planner-compare, Copyright 2026 Arpan Bom`, then idea-credit licenses and a separate dependency-notice section (to be filled as FetchContent lands).
- README stub: portfolio demo, simulation only, Apache-2.0, author Arpan Bom, no production or real-robot claims.
- Root `CMakeLists.txt` (C++17, CMake >= 3.22, options `NCP_BUILD_PYTHON`, `NCP_BUILD_TESTS`, `NCP_BUILD_ROS` defaults as in the plan), `.gitignore`, `pyproject.toml` placeholder if needed.
- Add `SPDX-License-Identifier: Apache-2.0` on every authored file created from here on (after shebangs / XML declarations).

**Accept:** `cmp LICENSE` against a fresh curl of the official text exits 0. NOTICE and README name Apache-2.0 and Arpan Bom. `cmake -S . -B build` configures with ROS off.

## 2. FetchContent pins and third-party notices

- Add GoogleTest, pybind11, nlohmann/json, and yaml-cpp via FetchContent at the **commit SHAs** in PLAN.md. Record archive SHA-256 in CMake when first downloaded.
- Append MIT/BSD notices for those dependencies to `NOTICE`. Do not relicense them. Do not add SPDX Apache headers to their sources.

**Accept:** Configure+build fetches the pinned commits (not a moved tag). NOTICE lists each dependency and its license.

## 3. Grid loader and costmap

- Implement `GridMap` PGM/YAML load (supported subset in PLAN.md) and world/cell conversion including origin x,y,yaw.
- Implement occupancy vs `uint8` cost, unknown policy, center-space inflation (`inflation_radius_m = robot_radius_m + extra_margin_m`), optional soft decay, and the single segment-collision predicate (center vs hard lethal/inscribed; no double-counted circle).
- Reject unsupported PGM/YAML with `invalid_request`. Tool `ncp_dump_fixture` can emit `map.json`.

**Accept:** gtests for tiny maps, rotated origin, negate, unknown-as-lethal vs free, inflation opening/closing a one-cell corridor, and no corner-cut diagonal. Segment check fails when a line crosses lethal even if endpoints are free.

## 4. Shared graph search: Dijkstra and A*

- One engine, heuristic 0 vs admissible octile, 8-connected costs as in PLAN.md, deterministic heap tie-break, no corner cutting.
- Public path type is `std::vector<WorldPoint>` (meters). Start==goal → one waypoint, length 0.
- Reasons: `start_in_obstacle`, `goal_in_obstacle`, `unreachable`, `out_of_bounds`.

**Accept:** Hand-derived 10x10 (or smaller) cases with a known optimal 8-connected length. A* objective cost equals Dijkstra within `1e-6` on the same graph. Closed wall returns `unreachable` under a 2 s CTest timeout.

## 5. Lazy Theta*

- Implement deferred line-of-sight Lazy Theta* with parent repair, not a post-smoother. Same collision predicate for LOS. Credit the AAAI 2010 paper in the source comment and README.

**Accept:** On `open`, the path is collision-free and geometric length <= A*. A case where any-angle is strictly shorter than 8-connected A* (e.g. displacement (2,1) with clear LOS) is tested. Walled-off goal still returns `unreachable`.

## 6. RRT*

- Shared xorshift32 (zero seed → 1; document TS `>>>` equivalent). Iteration-only vs deadline modes as in PLAN.md. Locked parameters, lowest-index nearest tie-break, continue optimize until iteration cap in fixture mode.
- Fixture mode never uses a wall-clock cap. Deadline mode may return `timeout_incumbent` or `timeout`, never `unreachable`.

**Accept:** Same seed + map yields identical vertex counts and waypoints in two C++ runs. Closed wall with an iteration cap returns `timeout` and terminates. A gtest compares a recorded xorshift32 uint32 vector.

## 7. Metrics and independent validators

- Length (meters along segments), objective cost, `compute_ms` (reported only), clearance along segments (JSON null if no obstacles), coverage ratios (implemented fully in task 8).
- Validator library: path endpoints, every segment collision-free, length within bounds, no NaN in JSON fixtures.

**Accept:** Validators catch a path that clips a corner. `compute_ms` is absent from `expected.json` schema. Clearance uses segments, not vertices only.

## 8. Coverage planner

- Boustrophedon / lawnmower on a simple polygon or full map, start pose required, overlap validation, stripe linking with a chosen global planner, `partial_coverage` without teleporting, both coverage ratios as in PLAN.md.
- No polygon interior rings in v1.

**Accept:** `hole` fixture either links reachable stripes or reports `partial_coverage` with `unreachable_free_ratio > 0`. Invalid overlap or radius → `invalid_request`. Swept-area coverage is not just circles at vertices.

## 9. Benchmark maps and golden fixtures

- Add `open`, `office`, `hole`, `narrow_pass`, `narrow_block`, `unreachable`, `unknown_row`, `tiny` under `fixtures/maps/`.
- `ncp_dump_fixture` writes `map.json` and `expected.json`. Check them in. Keep a short note in `fixtures/README.md` that goldens are regeneration-from-C++ plus hand-derived cases.

**Accept:** Every map in the plan table exists. Regenerating goldens is a documented command. Hand-derived tiny cases are not solely produced by the dumper.

## 10. C++ test suite complete

- gtest covers all edge cases in PLAN.md and success criteria 5–12 that are C++-testable. CTest TIMEOUT set.

**Accept:** `ctest --output-on-failure` is green on Ubuntu without ROS. Criteria 5–12 have named tests.

## 11. pybind11 module and pytest

- Build `ncp_core` into `build/python/` (and `python/ncp/`). `pytest.ini` pythonpath set. Authored `ncp_core.pyi` with SPDX. pytest diffs goldens, runs validators, prints a platform bench table (load vs search timed separately).

**Accept:** `PYTHONPATH=build/python pytest -q python/tests` passes without a ROS install, on Python 3.12. Criterion 2 holds.

## 12. TypeScript planner port and vitest

- Port costmap, collision, planners, coverage, and xorshift32 to TypeScript. Consume `map.json`, not PGM. Vitest uses the same fixtures.

**Accept:** vitest matches C++ goldens (paths, reasons, lengths, RRT* iteration-only vertices, RNG vector). Criterion 3 holds.

## 13. Canvas demo, Web Worker, Pages base

- Vite 8 app, `base: '/nav2-coverage-planner-compare/'`. Canvas shows grid, inflation, path, metrics. Worker with cancellation and stale-result drop. Build script copies `fixtures/` into `web/public/fixtures`.

**Accept:** `npm ci && npm test && npm run build` succeeds on Node 26.13.0. A local preview serves the app under the base path and plans A* on `office`.

## 14. Playwright smoke

- Chromium only. Install with `--with-deps`. Preview the **built** site. Exercise A* on `office` and coverage on `hole` at the Pages base path. Explicitly write `web/test-results/demo.png` and configure CI to upload it.

**Accept:** Playwright is green locally and the screenshot exists. Criterion 4 holds.

## 15. ROS 2 Humble wrapper

- `ros2/ncp_planner` with `package.xml` (Apache-2.0), `.srv` files, `rosidl` generation, node QoS, snapshot-per-request, stamped I/O, identity quaternions, `planner.launch.py`. Install the core to a prefix; the package finds `ncp` via CMake config.
- Smoke script: transient-local `/map`, wait, `plan_global`, assert success and a non-empty path. Service-before-map returns `no_map`.

**Accept:** Colcon build on Humble succeeds. Smoke passes. Core tests still run with `NCP_BUILD_ROS=OFF` and no ROS installed.

## 16. CI and GitHub Pages

- `core.yml` (ubuntu-24.04, g++ 13, Python 3.12), `web.yml` (Node 26.13.0, Playwright), `ros2-humble.yml` (label `ci-ros`, 20 min setup/build, 3 min smoke), `pages.yml` (deploy `web/dist` from `main` only after core+web succeed).
- Pin `ros:humble-ros-base` digest on the first green run if the job uses that image.

**Accept:** Workflows exist and match PLAN.md. A dry-run or first CI on the branch shows core and web green. Pages base path is used in the deploy build.

## 17. README, credits, and honesty pass

- Finish README in the order given in PLAN.md. Credit idea sources and their licenses. Flag GPL/LGPL as idea-only. Document collision model, RRT* modes, metrics, quick start (`PYTHONPATH`, optional ROS), and Idea Hub (no SaaS in this repo).
- Footer and `package.xml` say Apache-2.0. Screenshot in README once Playwright has `demo.png`.

**Accept:** Criteria 14, 16, 17, 19, 20 can be checked off by reading the tree. No MIT license artifacts remain.

## 18. Git history hygiene (when the public repo is created)

- Every commit: `GIT_AUTHOR_NAME="Arpan Bom"` and `GIT_COMMITTER_NAME="Arpan Bom"`, with Arpan's chosen email. No co-author trailers. Do not rewrite already-pushed history without explicit approval.

**Accept:** `git log --format='%an %cn %s %b'` shows only Arpan Bom as author and committer and no co-author trailers.

---

Do not start a follow-on planner (TEB/MPC, WASM, Jazzy, Idea Hub product) in this repository.
