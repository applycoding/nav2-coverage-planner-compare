# nav2-coverage-planner-compare — v1 plan

Revised 2026-11-10 after independent version, URL, license, and algorithm checks. Outbound license is **Apache-2.0**, not MIT.

## Goal

Public portfolio demo for a Lionsbot Senior Software Engineer (Path Planning) interview. It shows C++17, Python, ROS 2 Humble, Nav2-style costmaps, boustrophedon coverage, and global planners (A*, Dijkstra, RRT*, Lazy Theta*) on occupancy grids, plus a GitHub Pages canvas demo and shared metrics: path length, objective cost, compute time, coverage percent, clearance, and success.

This is a **portfolio demo, simulation only**. Do not claim production years with these algorithms, a Nav2 plugin used in production, or a run on a real robot.

Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

## Scope

Single ground robot, circular footprint, indoor grid. No fleet, aerial stack, Gazebo, SLAM, AMCL, or mission FSM.

- Costmap from PGM/YAML (`nav_msgs` map layout) plus a numeric traversal-cost layer. Inflation produces hard lethal/inscribed center-space and a separate soft cost field.
- Coverage: simple polygon or axis-aligned bounds (no interior rings in v1). Stripe spacing is `2 * robot_radius - overlap`. Obstacles in the costmap split stripes; a selected global planner links components. Spiral is a stretch goal, not required.
- Global: A* (8-connected, cost-aware), Dijkstra on that same graph (optimality baseline), seeded RRT*, Lazy Theta* (deferred line-of-sight, parent repair). Hybrid-A*, state lattice, TEB, MPC, DWA, NavFn, and Smac are out.
- Browser demo reimplements the algorithms in TypeScript and is parity-tested against C++ fixtures. No Emscripten/WASM in v1 (a project tradeoff, not a claim that WASM is generally unreliable).
- ROS 2 wrapper is optional and is the only piece that needs Humble.

Do not duplicate existing applycoding repos: no Gazebo, slam_toolbox, Nav2 bringup, AMCL, or waypoint mission runner (`ros2-nav2-slam-sim`); no PX4, MAVSDK, GPS lawnmower, geofence, or RTH energy (`px4-drone-mission-planner`); no CBS, MAPF, fleet, or charging (`multi-robot-warehouse-planner`); no drone kinematic FSM (`ros2-drone-mission-sim`).

Do not vendor Fields2Cover, GDAL, OR-Tools, or Nav2 source. Reimplement from published ideas. Do not copy, translate, or link GPL/LGPL sources (idea-only: `ethz-asl/polygon_coverage_planning`, `nav2_amcl`).

## Architecture

**Decision:** a ROS-free C++17 library (`ncp`) with GoogleTest; a thin pybind11 module so pytest and Python benches call that library; an optional Humble `rclcpp` node; a TypeScript + Canvas 2D site on GitHub Pages, with planners running in a Web Worker, parity-tested against C++ golden JSON.

**Why:** the interview artifact is the C++ core, and algorithm CI must pass on Ubuntu without ROS. pybind11 avoids a second Python planner that would drift. The node only adapts `OccupancyGrid` plus start/goal or a polygon into `nav_msgs/Path` and `visualization_msgs/MarkerArray`. Humble is kept because it matches `ros2-nav2-slam-sim` and is supported until May 2027; it is **not** claimed as the newest ROS LTS (Jazzy is a later port). WASM is deferred so v1 does not take on an extra toolchain; fixture parity keeps the TS port honest.

Layers: `GridMap` (PGM/YAML, resolution, origin) → `Costmap` (occupancy, hard inflation, soft costs, unknown policy) → segment `Collision` (robot-center vs hard lethal/inscribed) → `GlobalPlanner` → `CoveragePlanner` → `Metrics` → adapters (pybind11, ROS, TS).

One graph-search engine implements A* and Dijkstra (octile heuristic vs zero). One segment-collision predicate is used for graph edges, Lazy Theta* line-of-sight, RRT* steer/rewire, coverage links, and independent validators.

### Costmap and collision

Occupancy and traversal cost are stored separately. Occupancy cells: `Free`, `Occupied`, `Unknown`. After inflation, each cell also has a `uint8` cost inspired by Nav2, documented as a **simplified** model, not a bit-identical `nav2_costmap_2d` drop-in:

- 0–252: traversable soft cost
- 253: inscribed (hard if `collide_inscribed`, else high soft cost)
- 254: lethal (hard)
- 255: unknown

**Collision model (pick this one, do not mix):** inflate occupied cells (and unknown cells when the policy is lethal) into **forbidden robot-center space** once, using `inflation_radius_m = robot_radius_m + extra_margin_m` (`extra_margin_m` defaults to 0). Planning checks the path **center** (points and segments) against that hard space. Do not also test a full circle against already-expanded cells (that would double-count the radius). Soft exponential decay (`cost_decay`; 0 means no soft field, only hard lethal vs free) is for cost-aware search, not a second collision radius.

Unknown cells default to lethal (`UnknownPolicy::Lethal`); `Free` is opt-in. Edge cost for 8-connected search: `step_length_m * (1 + w_cost * cell_cost / 252)` with `w_cost` default 1 and `cell_cost` the max of the two endpoint cells (or a documented midpoint). Diagonal length is `resolution * sqrt(2)`. The A* heuristic is octile, scaled by the minimum per-meter cost so it stays admissible. Corner cutting is forbidden: a diagonal is illegal if either adjacent orthogonal neighbor is hard-lethal.

`ncp_dump_fixture` also writes `map.json` (occupancy, costs, metadata) so the TypeScript port does not parse PGM/YAML.

### Paths and planners

Paths are `std::vector<WorldPoint>` in meters (`x`, `y`). Grid indices are an implementation detail, not the public path type. Start equal to goal on a free center returns **success and a single waypoint**, length 0, reason empty.

RRT* has two modes:

- **Fixture / iteration-only:** `rrt_max_time_ms` disabled (0). Same seed, map, start, goal, and `rrt_max_iterations` ⇒ identical vertex count and waypoint sequence in C++ and TS. Shared xorshift32 (`uint32_t`, zero seed becomes 1; TS uses unsigned `>>>` shifts). Parameters locked for v1: `step_size_m`, `rewire_radius_m`, `goal_bias` 0.1, `goal_connect_m`, continue-optimize until iteration cap after first goal connection (so fixtures are stable). Specify nearest-neighbor tie-break: lowest insertion index wins.
- **Deadline / demo:** time cap on, no golden path equality. A valid incumbent at timeout is `success` with reason `timeout_incumbent`. Exhausted budget with no path is `timeout` (not a proof of unreachability). Only A* / Dijkstra / Lazy Theta* may return `unreachable`.

Lazy Theta* is the deferred-LOS algorithm (validate parent visibility when expanding, repair if needed), not a post-hoc smoother. Credit Nash, Koenig, Likhachev, “Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D” (AAAI 2010), https://idm-lab.org/bib/abstracts/papers/aaai10b.pdf. Any-angle segment cost uses the Euclidean length times the same cost weight, sampled along the segment. Do not claim general shortest-path optimality for Lazy Theta*, and do not require its geometric length to equal 8-connected A*.

### Coverage

`CoverageRequest` includes a **start pose**, region (simple polygon or empty = full free map), `linker` planner, `overlap_m`, RRT* seed/caps if the linker is RRT*, and `coverage_success_threshold` (default 0.95).

v1 regions are simple, non-self-intersecting polygons **without interior rings**. Holes come from costmap obstacles, not polygon holes. Validate `robot_radius_m > 0` and `0 <= overlap_m < 2 * robot_radius_m` (default overlap 0). Stripe axis is the longer side of the region's axis-aligned bounding box unless an optional `stripe_angle` is set (default 0 = rows along x). Connectors must stay in admissible center space; they may leave the polygon only if the footprint is still collision-free, but v1 keeps connectors inside the inflated-inward region when possible. A failed connector returns `partial_coverage` plus metadata (`covered_ratio`, `unreachable_free_ratio`) and only the physically connected path prefix (no fake jump). Disconnected regions are not concatenated into one traversable route.

Coverage percent is the area of the **continuous swept circle** (radius = `robot_radius_m`) along path segments, intersected with free cells in the region, divided by free cells in the region that are reachable from the start under the current costmap. Report both `coverage_ratio_reachable` (success uses this vs threshold) and `coverage_ratio_total` (includes unreachable free cells so a closed-off room cannot vanish). Unknown-as-lethal cells are not free.

## Pinned stack

Checked 2026-11-10 against PyPI, npm, GitHub releases, and the Node/Python support schedules. Pins are exact; CI does not float `latest`. FetchContent uses the **commit SHA** below (tags can be moved). Record SHA-256 of the downloaded archives at first build and keep them in CMake.

| Component | Pin | Source |
| --- | --- | --- |
| Language | C++17, no GNU extensions | — |
| Core CI | `ubuntu-24.04`, g++ 13, Python 3.12 | GitHub Actions |
| Humble CI | `ros:humble-ros-base` (pin the image digest at first green run), g++ 11, system Python 3.10 only for the wrapper | Humble is supported until May 2027; 3.10 itself is EOL upstream as of 2026-10-01 (https://endoflife.date/api/python.json) and is used only because the Humble image ships it |
| CMake | minimum 3.22 (Ubuntu 22.04). Core job may use the runner's newer CMake. Current stable seen: 4.4.5 | https://github.com/Kitware/CMake/releases/tag/v4.4.5 |
| GoogleTest | v1.18.0, commit `063de7e9578f82b369302001269680b4b1553359` | https://github.com/google/googletest/releases/tag/v1.18.0 |
| pybind11 | v3.1.0, commit `97bf890db679505a14dfe547a5e77bb2bd05dc90` (CMake `Python_*` / FindPython, see upgrade guide) | https://pypi.org/project/pybind11/ / https://github.com/pybind/pybind11/releases/tag/v3.1.0 |
| nlohmann/json | v3.12.0, commit `55f93686c01528224f448c19128836e7df245f72` (MIT; list in NOTICE as a dependency) | https://github.com/nlohmann/json/releases/tag/v3.12.0 |
| yaml-cpp | yaml-cpp-0.9.0, commit `56e3bb550c91fd7005566f19c079cb7a503223cf` (MIT; list in NOTICE) | https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.9.0 |
| pytest | 9.2.0, lock transitive deps in `python/requirements-dev.txt` (requires Python >=3.10) | https://pypi.org/project/pytest/ |
| Node | 26.13.0 Active LTS (Lithium). Node 20 ended 2026-04-30 | https://github.com/nodejs/Release `schedule.json` |
| TypeScript | 7.0.2 | https://registry.npmjs.org/typescript/latest |
| Vite | 8.3.7 | https://github.com/vitejs/vite/releases/tag/v8.3.7 and npm `latest` |
| Vitest | 5.1.0 (peer: Vite `^6.4.0 \|\| ^7.0.0 \|\| ^8.0.0`, Node `^22.12.0 \|\| ^24.0.0 \|\| >=26.0.0`) | https://registry.npmjs.org/vitest/5.1.0 |
| Playwright | `@playwright/test` 1.65.0 (depends on `playwright` 1.65.0). Chromium only | https://registry.npmjs.org/@playwright/test/latest |
| ROS | Humble only. Apt packages: `ros-humble-rclcpp`, `ros-humble-nav-msgs`, `ros-humble-geometry-msgs`, `ros-humble-visualization-msgs`, `ros-humble-rosidl-default-generators`, `ros-humble-rosidl-default-runtime`, `ros-humble-ament-cmake`. Jazzy is a later port, not built in v1 | https://docs.ros.org/en/humble/index.html |

Python 3.10 is **not** the core test interpreter. Core pytest runs on 3.12.

## File layout

```text
LICENSE  NOTICE  README.md  CMakeLists.txt  pyproject.toml
cpp/include/ncp/  cpp/src/  cpp/test/  cpp/tools/ncp_dump_fixture.cpp
python/ncp/__init__.py  python/ncp/ncp_core.pyi  python/tests/  python/requirements-dev.txt
ros2/ncp_planner/  (package.xml, CMakeLists.txt, planner_node.cpp, srv/, launch/planner.launch.py)
web/  (Vite app, src/planners, src/worker, vitest, playwright.config.ts)
fixtures/maps/<id>/{map.pgm,map.yaml,map.json,cases.json,expected.json}
.github/workflows/{core,web,ros2-humble,pages}.yml
```

Namespace is `ncp`. `NCP_BUILD_PYTHON` and `NCP_BUILD_TESTS` default ON. `NCP_BUILD_ROS` defaults OFF so the core never links `rclcpp`. The pybind11 module is written to `build/python/` (and copied next to `python/ncp/`). `python/pytest.ini` sets `pythonpath` to that location. `pyproject.toml` may add an optional editable install later; the documented quick start does not require it.

## Interfaces

C++ (`namespace ncp`):

```text
struct WorldPoint { double x, y; };          // meters
enum class Occupancy : uint8_t { Free, Occupied, Unknown };
enum class UnknownPolicy { Lethal, Free };
enum class PlannerId { AStar, Dijkstra, RrtStar, LazyThetaStar };
enum class RrtMode { IterationOnly, Deadline };
struct CostmapParams {
  double inflation_radius_m;   // robot_radius_m + extra_margin_m
  double robot_radius_m;
  double extra_margin_m;       // default 0
  double cost_decay;           // 0 = hard only
  double w_cost;               // default 1
  bool collide_inscribed;      // default true for hard inscribed
  UnknownPolicy unknown_policy;
};
struct PlanRequest {
  PlannerId planner; WorldPoint start, goal;
  uint32_t rrt_seed; int rrt_max_iterations; int rrt_max_time_ms; // 0 time = iteration-only
  RrtMode rrt_mode;
};
struct CoverageRequest {
  WorldPoint start;
  std::vector<WorldPoint> region_polygon; // empty => full map; simple, no holes
  PlannerId linker; double overlap_m; double stripe_angle;
  uint32_t rrt_seed; int rrt_max_iterations; int rrt_max_time_ms;
  double coverage_success_threshold; // default 0.95
};
struct Metrics {
  double length_m;             // geometric path length
  double objective_cost;       // graph / segment objective
  double compute_ms;           // wall clock of the plan call; never golden-compared
  double coverage_ratio_reachable;
  double coverage_ratio_total;
  double clearance_m;          // min footprint-center-to-occupied distance along segments; NaN in C++ if no obstacles, JSON null
  bool success;
  std::string reason;
};
struct PlanResult {
  bool success; std::string reason;
  std::vector<WorldPoint> path;
  Metrics metrics;
  int rrt_vertices;            // 0 if not RRT*
  double unreachable_free_ratio; // coverage only
};
class GlobalPlanner {
  virtual ~GlobalPlanner() = default;
  virtual PlanResult plan(const Costmap&, const PlanRequest&) = 0;
};
```

Stable `reason` strings: `start_in_obstacle`, `goal_in_obstacle`, `unreachable`, `timeout`, `timeout_incumbent`, `partial_coverage`, `empty_map`, `invalid_request`, `out_of_bounds`, `no_map`. Defaults are initialized on every request struct.

Python module `ncp_core` mirrors those structs. Authored stubs `ncp_core.pyi` carry an SPDX header. pytest reads `fixtures/` and compares to `expected.json` written by `ncp_dump_fixture`. Compared fields: path waypoints, reasons, lengths, objective costs, RRT* vertex counts, coverage ratios. **Never** `compute_ms`.

ROS 2 package `ncp_planner` uses `ament_cmake` and `rosidl_default_generators` with `srv/PlanGlobal.srv` and `srv/PlanCoverage.srv` (stamped pose or stamped polygon, planner id, overlap, seed, threshold; response: `nav_msgs/Path`, success, reason, metrics). Node `ncp_planner_node` subscribes to `/map` (`nav_msgs/OccupancyGrid`) with **reliable + transient_local** QoS, keeps a snapshot per request, and returns `no_map` if called before a map arrives. Parameters match `CostmapParams` plus default `planner_id`. It publishes `/plan` (`nav_msgs/Path` with header frame_id from the map and identity quaternions) and `/markers`. No lifecycle, no behavior tree, no `nav2_core` plugin in v1. The installed C++ core is found via CMake package config + `CMAKE_PREFIX_PATH`.

TypeScript:

```ts
export interface Planner {
  id: "astar" | "dijkstra" | "rrt_star" | "lazy_theta";
  plan(map: Costmap, req: PlanRequest): PlanResult;
}
export function planCoverage(map: Costmap, req: CoverageRequest): PlanResult;
```

Canvas 2D draws grid, inflation, path, and the metric readout. Planning runs in a Web Worker with cancellation and a map-size cap; out-of-order results after a new request are dropped. Vite `base` is `/nav2-coverage-planner-compare/` for GitHub Pages. Fixture JSON is copied into `web/public/fixtures` by the web build script (single source: `fixtures/`, not a second editable copy).

## Edge cases

- Walled-off goal on A* / Dijkstra / Lazy Theta* → `unreachable`, returns, no retry loop. RRT* on the same map with a budget and no path → `timeout`, not `unreachable`.
- Start in inflated lethal → `start_in_obstacle` before search. Goal in obstacle or inflated lethal → `goal_in_obstacle` before search. Out of map → `out_of_bounds`. Degenerate radius or overlap → `invalid_request`.
- `narrow_pass` gap just wider than `2 * inflation_radius_m` (center-space) succeeds; `narrow_block` just narrower fails. Document the exact cell widths in `cases.json`.
- Same `office` PGM at two inflation radii: the larger radius makes the short corridor illegal.
- Coverage with an interior **costmap** hole: linker joins stripe components, or `partial_coverage` with `unreachable_free_ratio` set. Success is **not** forced to a total ratio below 0.95 (a small closed room can be <5%).
- Zero-cell or invalid PGM/YAML → `empty_map` or `invalid_request`. `1x1` free cell, start==goal → success, one waypoint, length 0.
- RRT* iteration-only: same seed + map ⇒ identical vertices and path in C++ and TS. Length ε is `1e-6` m. A shared xorshift32 vector is asserted in both languages.
- `unknown_row`: default lethal blocks the path; `UnknownPolicy::Free` allows it. Footprint/segment checks treat unknown under the center the same way.
- Diagonal corner cutting is rejected. A rotated map origin and a negated PGM are loaded correctly or rejected with `invalid_request` if a feature is unsupported (document supported PGM: P5, maxval 255, `#` comments; YAML keys `image`, `resolution`, `origin` [x,y,yaw], `negate`, `occupied_thresh`, `free_thresh`).
- Service-before-map in ROS returns `no_map`. A later map replacement is snapshotted per call.

## Test plan

Each fixture directory has `map.pgm`, `map.yaml`, generated `map.json`, `cases.json`, and checked-in `expected.json` produced by `ncp_dump_fixture`. Golden files are **regression** data. Independent validators (collision-free segments, start/goal, admissible length bounds) and a few **hand-derived** tiny-grid cases (known optimal 8-connected length) must also pass so a bad dumper cannot bless a bad planner. CTest `TIMEOUT` (2 s for unit cases, larger for benches) is the hang watchdog.

| Id | Lock |
| --- | --- |
| `open` | A* objective cost equals Dijkstra on the same graph within `1e-6`. Lazy Theta* is valid and its geometric length is <= A* length on this map |
| `office` | Inflation changes the corridor |
| `hole` | Interior obstacle, link or partial coverage with reported unreachable ratio |
| `narrow_pass`, `narrow_block` | Gap versus center-space inflation diameter |
| `unreachable` | Walled room: grid search `unreachable`; RRT* iteration cap `timeout` |
| `unknown_row` | Lethal default vs free policy |
| `tiny` | 1x1 and start==goal; plus an empty-grid case |

- gtest in `cpp/test` covers loaders, inflation, collision, planners, coverage, edge cases, and meter conversion. Those headers do not include ROS.
- pytest loads the extension, diffs `expected.json` (no `compute_ms`), runs validators, and prints a bench table that includes the platform string. Time inflation/load and search separately. Report RRT* over several seeds in deadline mode as a distribution, not a golden path.
- vitest runs the TS planners on the same fixtures, including RRT* iteration-only vertex counts and the shared RNG vector.
- Playwright (`web.yml`): `npx playwright install --with-deps chromium`, `npm run build`, `vite preview --port 4173`, load `office` at the Pages base path, run A*, assert metrics text shows a length and success, run coverage on `hole`, explicitly write and upload `web/test-results/demo.png`. Chromium only.

Workflows on push and pull_request:

- `core.yml` (`ubuntu-24.04`): cmake, ctest, pytest on Python 3.12. No ROS apt packages.
- `web.yml`: `npm ci`, `tsc --noEmit`, vitest, Playwright as above. Node 26.13.0.
- `ros2-humble.yml`: `pull_request` types include `labeled` / `synchronize` / `opened`. Runs on push to `main` and on PRs labeled `ci-ros`. Colcon-build the wrapper against the installed core. Smoke: publish `fixtures/maps/open` as a latched (reliable, transient_local) `OccupancyGrid`, wait until the node has the map, call `plan_global`, assert `success` and a non-empty `Path`. Job budget: setup/build up to 20 minutes; smoke itself capped at 3 minutes. A red ROS job does not waive a red core job.
- `pages.yml`: deploy `web/dist` from `main` only after core and web succeed. `actions/deploy-pages` with the required permissions. Vite base `/nav2-coverage-planner-compare/`.

## Numbered success criteria

1. `cmake -S . -B build && cmake --build build && ctest --test-dir build` passes on `ubuntu-24.04` / g++ 13 / Python 3.12 with no ROS installed.
2. pytest matches `expected.json` through the pybind11 module without comparing `compute_ms`, and independent validators plus hand-derived tiny cases pass.
3. vitest matches the same fixtures, including RRT* iteration-only vertex counts and the shared xorshift32 vector.
4. Playwright builds the site, loads `office` at the Pages base path, runs A* and coverage, and uploads `demo.png`.
5. On `open`, A* and Dijkstra share the same **objective cost** (and geometric length when the objective is purely geometric) within `1e-6`. Lazy Theta* returns a collision-free path whose geometric length is less than or equal to A* on that map; it is not required to equal A* length in general.
6. Unreachable (grid search), start-in-obstacle, and goal-in-obstacle return the documented reason and finish inside a 2 s CTest timeout.
7. `narrow_pass` succeeds and `narrow_block` fails at the documented radius and inflation in `cases.json`.
8. A larger inflation on `office` removes the short corridor, as asserted in `cases.json`.
9. `hole` either links all reachable stripe components or returns `partial_coverage` with `unreachable_free_ratio > 0`. Success uses `coverage_ratio_reachable` versus the threshold, not a forced total ratio below 0.95.
10. Unknown cells are lethal unless `UnknownPolicy::Free`, locked by `unknown_row`.
11. `tiny` start==goal is success with one waypoint and length 0. An empty grid returns `empty_map`.
12. Length and clearance are meters using YAML resolution. Clearance is computed along segments. The bench names OS and compiler and does not golden-compare `compute_ms`.
13. The Humble job colcon-builds the wrapper (generated interfaces included) and the service smoke returns a path after a transient-local map. Core tests do not need that job.
14. README says portfolio demo, simulation only, Apache-2.0, and does not claim production or a real robot.
15. `LICENSE` is byte-identical to `https://www.apache.org/licenses/LICENSE-2.0.txt`. `NOTICE` and SPDX headers match the License section.
16. The tree has no Gazebo, SLAM, AMCL, PX4, MAVSDK, MAPF, fleet, or charging dependency.
17. No vendored Nav2, Fields2Cover, GDAL, or OR-Tools, and no GPL/LGPL code copied or translated.
18. Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.
19. The Pages site is static Vite + Canvas 2D + Web Worker, not WASM, with `base` `/nav2-coverage-planner-compare/`.
20. Jazzy appears only as a future port. Humble is described with its support horizon, not as “the current industry LTS.”

## Out of scope

Hybrid-A*, state lattice, TEB, MPC, DWA, NavFn, Smac, behavior trees, recoveries, velocity smoothing, sensor simulation, dynamic obstacles, multi-robot, outdoor GPS, required spiral coverage, polygon-with-holes cellular decomposition, `nav2_core` plugin registration, Emscripten/WASM, Windows CI, Jazzy CI, and live robot bags. A hosted SaaS, accounts, or billing (Idea Hub one-pager only, later).

## Documentation and honesty

README order: (1) portfolio demo, simulation only, single robot, not a production Nav2 plugin and not run on a robot; (2) planner comparison and a Pages screenshot; (3) quick start for C++ tests, Python bench (including how `ncp_core` is found), and the web server, with ROS marked optional; (4) 8-connected costs, inflation/collision model, stripe rule, RRT* modes; (5) metric definitions and the bench platform column; (6) Apache-2.0, `NOTICE`, and idea credits, stating this code was written here and is not a copy; (7) author Arpan Bom.

Idea sources, not code sources: PythonRobotics (MIT), Nav2 costmap / NavFn / Smac 2D / Theta* packages (Apache-2.0 and/or BSD-3-Clause; do not treat all of Nav2 as one license), Fields2Cover (BSD-3-Clause), opennav_coverage (Apache-2.0), nobleo/full_coverage_path_planner (Apache-2.0; this is a ROS 1 / catkin plugin, used only as a coverage-pattern idea, not a Humble API template), ngraph.path (MIT), javascript-astar (MIT), rrt-algorithms (MIT).

Idea-only, do not derive code: ethz-asl/polygon_coverage_planning (SPDX GPL-3.0; README states GPL-3.0-or-later because of CGAL, which is also non-commercial for some exact geometric components); nav2_amcl (LGPL-2.1-or-later).

Current docs URLs (the old `docs.nav2.org/configuration/packages/configuring-*.html` paths 404; the site now redirects to `/rolling/`):

- https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/
- https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/
- https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/
- https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/

**Idea Hub:** keep this a portfolio demo. If the viewer and coverage tool later look productizable, send a one-pager to the Idea Hub bot. Do not add accounts, billing, hosted maps, or a service API here.

## License

Apache-2.0, not MIT.

- `LICENSE` is the verbatim file at `https://www.apache.org/licenses/LICENSE-2.0.txt`, fetched when the repo is created, not rewritten. Verify byte equality with the official text (including the appendix).
- `NOTICE` opens with `nav2-coverage-planner-compare, Copyright 2026 Arpan Bom`. List idea-credit projects and their licenses separately from **dependency** notices (GoogleTest, pybind11 BSD-style, nlohmann/json MIT, yaml-cpp MIT, and any FetchContent sources shipped in archives). State that no third-party planner source is vendored. Do not relicense dependencies.
- Authored C++, Python (including `.pyi`), TypeScript, CMake, launch, shell, and GitHub workflow files start with `SPDX-License-Identifier: Apache-2.0` in a valid comment, placed after any shebang or XML declaration. Lockfiles, generated stubs, and PGM fixtures are exempt. Never relabel third-party files with this project's license.
- ROS `package.xml`, `pyproject.toml`, README, and the site footer all say Apache-2.0.

## Build and CI

```text
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DNCP_BUILD_PYTHON=ON -DNCP_BUILD_TESTS=ON
cmake --build build -j && ctest --test-dir build --output-on-failure
PYTHONPATH=build/python pytest -q python/tests
cd web && npm ci && npm test && npm run build
```

The Humble job installs the core to a prefix (`NCP_BUILD_PYTHON=OFF`, `NCP_BUILD_TESTS=OFF` is allowed there), then colcon-builds `ros2/ncp_planner` with the prefix on `CMAKE_PREFIX_PATH`. Playwright uses a completed Vite build and `vite preview --port 4173`. Algorithm correctness is gtest + pytest + vitest, plus independent path validators.

## Risks

- TS can drift from C++. `expected.json` from `ncp_dump_fixture` is regression data; vitest fails closed; hand-derived cases and validators catch a bad oracle. Planner changes land in one PR for both sides.
- Language RNGs differ. Both ports use the same xorshift32, checked against a shared vector of uint32 outputs. Deadline-mode RRT* is never golden-compared across runtimes.
- "Nav2-style" can be read as bit-identical Nav2 costs. README calls the model simplified occupancy + soft costs + center-space inflation, not a costmap drop-in.
- ROS smoke can flake. One latched map, QoS documented, wait-for-map, one service call, three-minute smoke cap, no simulator.
- The Pages demo cannot call the C++ binary. That is accepted; parity tests cover the TS port. Vite `base` must be tested or asset paths break on project Pages.
- License contamination if Nav2 or GPL/LGPL coverage code is pasted in. No git submodules of those repos; NOTICE states the rule; review rejects copied sources.
- FetchContent and `ros:humble-ros-base` are not fully reproducible until SHAs/digests are recorded at first green CI.

## Verification already done (do not re-litigate)

- Existing applycoding repos were read; this design does not duplicate them.
- Eight primary references are MIT, BSD-3-Clause, and/or Apache-2.0 (compatible with Apache-2.0 outbound as idea sources). GPL/LGPL items flagged idea-only.
- Old Nav2 config URLs 404; rolling URLs above return 200. `https://www.apache.org/licenses/LICENSE-2.0.txt` returns 200. DOI `10.1109/LRA.2023.3248439` redirects to IEEE (IEEE itself may return 202 to unauthenticated clients).
- A 10x10 Python sanity check: A* found a path around a wall and matched Dijkstra length (20.899495); a closed wall returned `unreachable` in 0.0002 s; RRT* with seed 42 was deterministic (length 18.8067, 103 nodes) and a closed wall terminated in 0.09 s with no path.
