# Critical review of PLAN.md

Review date: **2026-11-10**. Scope: the proposed design, dependency pins, APIs, tests, simpler alternatives, and licensing. This is a plan review, not evidence that an implementation builds or passes tests. PLAN.md is left unchanged.

**Recommendation: revise before implementation.** The ROS-free core and explicit simulation-only claims are sound. The largest problems are contradictory determinism and optimality promises, undefined collision/cost semantics, and a coverage interface that cannot express the promised behavior. All seven explicitly pinned library/test-tool versions are behind the current releases checked below.

## 1. Dependency and platform currency

These are stable releases observed in primary package/release sources on the review date, not instructions to float dependencies to `latest`. Select a compatible set, pin it, commit its lockfiles, and validate it together. Some docs.ros.org pages returned anti-bot access denials; those links are provided as interface documentation pointers, not claimed as successfully retrieved evidence. Version conclusions use the accessible primary release/registry sources linked below.

| Component | PLAN.md | Current observed release/status | Assessment and action |
| --- | --- | --- | --- |
| GoogleTest | v1.14.0 | [v1.18.0, GitHub](https://github.com/google/googletest/releases/tag/v1.18.0) | Outdated. Current release requires C++17, matching this project. Verify compiler and CMake requirements before replacing the pin. |
| pybind11 | v2.13.6 | [3.1.0, PyPI](https://pypi.org/project/pybind11/), released 2026-08-06 | Outdated major. Check the [upgrade guide](https://pybind11.readthedocs.io/en/stable/upgrade.html): v3 changes ABI compatibility and defaults to modern FindPython. Rebuild the extension; use `Python_*` CMake variables. |
| pytest | 8.3.4 | [9.2.0, PyPI](https://pypi.org/project/pytest/), released 2026-10-27 | Outdated. Confirm interpreter requirements and plugin compatibility when choosing the replacement. |
| TypeScript | 5.6.3 | [7.0.2, npm registry](https://registry.npmjs.org/typescript/latest) | Outdated major. Run type checking and review compiler/config/API changes; a version bump alone is insufficient. |
| Vite | 5.4.11 | [8.3.7, GitHub](https://github.com/vitejs/vite/releases/tag/v8.3.7) | Outdated major. npm registry access for Vite failed during review; GitHub is the verified release source here. Review the [migration guide](https://vite.dev/guide/migration) and align Node and Vitest. |
| Vitest | 2.1.8 | [5.1.0, npm registry](https://registry.npmjs.org/vitest/latest) | Outdated major. Registry metadata requires Node `^22.12.0 \|\| ^24.0.0 \|\| >=26.0.0` and Vite `^6.4.0 \|\| ^7.0.0 \|\| ^8.0.0`; do not combine this upgrade with the old Node/Vite pins. |
| Playwright | 1.49.1 | [@playwright/test 1.65.0, npm registry](https://registry.npmjs.org/@playwright/test/latest) | Outdated. Specify that Playwright Test is the runner; its metadata depends on `playwright` 1.65.0 exactly. Keep package, browser installation, and any container version aligned. |
| Node | 20 | EOL; [release schedule](https://github.com/nodejs/Release) lists 24 as Maintenance LTS and 26 as Active LTS | Replace Node 20. Choose an exact supported patch of 24 or 26, then test the web dependency set. A major-only declaration is not an exact pin. |
| Python | 3.10 | [EOL since 2026-10-01](https://devguide.python.org/versions/) | Keep only as an explicit Humble/Ubuntu compatibility lane if needed. Use a supported interpreter for the main ROS-free Python lane. Separate upstream Python EOL from any distribution vendor maintenance. |
| CMake | >=3.22 | [4.4.5 stable, GitHub](https://github.com/Kitware/CMake/releases/tag/v4.4.5) | This is a minimum, not a stale exact pin. Retain 3.22 compatibility if required by Ubuntu 22.04, and verify against a modern CMake version. |
| C++ / GCC / Ubuntu | C++17 / g++ 11 / 22.04 | Deliberate language/platform baseline | C++17 is a requirement, not a package pin. Keep the Humble baseline if useful; add a supported modern core lane rather than implying GCC 11 is current. |
| ROS | Humble, `ros:humble-ros-base` | Humble supported to May 2027; Jazzy to May 2029; Lyrical to May 2031 ([ROS-maintained support table](https://github.com/ros/meta-ros/blob/master/README.md)) | Humble is supported but near EOL, not the current LTS. Keep it for stated compatibility; replace “common industry LTS” with the actual reason and support horizon. Jazzy can remain a later port. |

The claim “immutable tags” is incorrect: Git refs can move. Use a full commit SHA or archive plus SHA-256 for FetchContent, as supported by [CMake FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html). `ros:humble-ros-base` is also a mutable image tag, and unversioned apt packages change. State the reproducibility level honestly; pin image digests and record resolved packages when exact replay matters. Exact top-level pytest pins do not lock transitive Python dependencies.

## 2. High-priority algorithm/design corrections

### P1 — RRT* exact replay contradicts the wall-clock cap

“Same seed, map, start, goal, and caps ⇒ identical vertex count and cell sequence” cannot hold when a 200 ms deadline determines the last iteration. C++ and JavaScript, browser load, and CI hardware will complete different iteration counts. `compute_ms` also cannot be golden-equal.

Use iteration-only termination for parity fixtures, with the time cap explicitly disabled. Keep a separate runtime/deadline mode that promises bounded work and valid results rather than an identical path. Specify whether a valid incumbent at timeout is success, and distinguish exhausted budget from proven unreachability: finite RRT* failure does not prove no path exists.

Shared xorshift32 is necessary but insufficient. Define zero-seed handling, unsigned shifts (`>>>` in TS), overflow, integer-to-sample conversion, nearest-neighbor ties, duplicate samples, traversal order, and floating-point comparisons. Specify steering distance, goal bias, goal connection tolerance, neighborhood/rewiring rule, descendant cost updates, cycle prevention, and whether optimization continues after first success. These are substantive RRT* choices, illustrated by the [OMPL RRT* API](https://ompl.kavrakilab.org/core/classompl_1_1geometric_1_1SORRTstar.html); it is a reference, not a required dependency.

### P1 — Optimality criteria compare different graphs/objectives

Criterion 5 requires A*, Dijkstra, and Lazy Theta* to share optimal length. On an unobstructed grid with displacement (2,1), an 8-connected path has length `1 + sqrt(2)` cells; a collision-free direct any-angle segment has length `sqrt(5)`. Equality holds only for specially aligned endpoints and would not demonstrate general correctness.

Compare A* with Dijkstra **objective cost on the same graph**, including the same diagonal/corner and cost rules. Cost-aware optimal paths need not have equal geometric lengths when several equal-cost paths exist. Test Lazy Theta* for valid endpoints, collision-free segments, and analytical length on a selected empty-map case. Do not advertise general shortest-path optimality for Theta-family search; see the [original Lazy Theta* paper](https://idm-lab.org/bib/abstracts/papers/aaai10b.pdf).

The description “line-of-sight shortcuts” also does not define Lazy Theta*. The algorithm defers visibility validation and repairs invalid assumed parents. Implement its specified search behavior, or call a simpler implementation “A* with line-of-sight smoothing.” Define segment cost integration if inflation costs affect any-angle planning; otherwise disclose that its objective differs from cost-aware A*.

### P1 — Inflation, occupancy, and footprint collision are conflated

The four-value `Cell` enum cannot represent an exponential soft-cost field by itself. Define separate occupancy and traversal-cost storage, numeric ranges, the edge-cost formula, and a positive base movement cost. In Nav2, inscribed collision costs and exponentially decaying costs are distinct; consult the [Humble inflation implementation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp), not only rolling documentation.

Checking a full robot circle against obstacles already expanded for the robot radius can count that radius twice. Conversely, ignoring inscribed cells can allow invalid center paths if center-space inflation was intended as the collision model. Choose explicitly:

- Check the physical footprint against raw occupied cell areas, with a separate soft-cost field; or
- Expand obstacles into forbidden robot-center space once, then check point/segment centers against that space.

Treat additional hard safety margin separately from soft inflation. `2*radius + inflation` is not a universal corridor-width threshold; a hard margin applied to both walls contributes twice. Specify boundary contact, cell-square geometry, and grid discretization. Test diagonal corner cutting, circle tangency, map-edge overhang, and unknown cells under the whole footprint. Every long edge needs a swept-circle/expanded-grid segment check; valid endpoints do not establish a valid motion.

### P1 — Coverage contract cannot express the promised task

`CoverageRequest` has no start pose/cell. Define the initial location, stripe order, connector start, and whether a return-to-start is required. A single `Polygon` has no explicit interior-ring representation; distinguish a region hole from an obstacle hole, and either support rings or explicitly reject region holes in v1.

Clarify whether connectors must stay inside the polygon. Clip the admissible center space appropriately if the full footprint must remain inside it. Reject self-intersections and degenerate polygons, and handle disconnected regions without concatenating separated subpaths into an apparent traversable route. On a failed connector, return only physically connected motion plus unreachable-region metadata, or explicitly return multiple paths.

Validate `robot_radius > 0`, finite values, and `0 <= overlap < 2*radius`. Otherwise the stripe loop can stall or reverse. State how subcell stripe spacing, stripe endpoints, narrow slivers, and boundary residual strips are handled. Separate tool/sweep radius from collision radius if coverage represents a cleaning tool.

Define coverage as the union of the **continuous swept path**, not circles around only listed vertices; sparse any-angle paths otherwise undercount. Define the denominator for inaccessible free cells, unknown-as-free cells, zero target area, and inflation-only cells. Report total-target coverage and optionally reachable-target coverage so disconnected rooms cannot disappear from the metric. An unreachable component can be smaller than 5%; therefore criterion 9's forced ratio below 0.95 does not follow from unreachable coverage. Define whether success means complete reachability or meeting a threshold.

## 3. Missing contracts and edge cases

| Area | Required additions to the plan |
| --- | --- |
| Grid loading | Define supported PGM variants (P2/P5, comments, maxval/bit depth), YAML parser dependency or deliberately restricted schema, relative image paths, `negate`, threshold inclusivity, and supported map modes. Reject unsupported formats explicitly. |
| Coordinates | Specify row-major occupancy versus image y direction, cell centers, meters/cells conversion, origin translation and yaw, negative world coordinates, and boundary rounding. Test a nonzero rotated origin and asymmetric image. [Humble MapMetaData](https://docs.ros.org/en/humble/p/nav_msgs/msg/MapMetaData.html) includes an origin pose; [OccupancyGrid](https://docs.ros.org/en/humble/p/nav_msgs/msg/OccupancyGrid.html) defines its data layout. |
| Invalid input | Validate dimensions/data length, overflow and allocation limits, positive finite resolution, finite radii/decay, bounds for start/goal, invalid planner IDs, negative/zero budgets, and thresholds outside [0,1]. Add `invalid_request`, `out_of_bounds`, and `no_map` or equally explicit errors; existing reasons are insufficient. |
| Search | Specify admissible heuristic (e.g. octile for the matching 8-neighbor graph), cost quantization, diagonal rules, deterministic heap ties, stale queue handling, and maximum work/memory. Integer cell costs alone do not remove floating-point diagonal costs. |
| Results | Start==goal should return one valid path point, not an empty successful route. Define failed/partial paths and a virtual destructor for the C++ planner interface. Give all request fields explicit initialized defaults. Avoid duplicated status/reason fields drifting between `PlanResult` and `Metrics`. |
| Metrics | Add objective cost independently of geometric length. Define clearance from footprint to obstacle area, not merely path-vertex center distance; evaluate segments and specify unknown/boundary policy. Use `null` or another documented JSON representation for undefined/obstacle-free clearance; JSON cannot represent infinity/NaN. Global-planner coverage should be explicitly inapplicable or defined. |
| Benchmarking | Time the same stage in each comparison; separate loading/inflation from search and end-to-end latency. Use repeated runs and multiple RRT* seeds, report distributions and hardware/build settings, and distinguish fixed-iteration from fixed-time comparisons. Do not claim TS/C++ speed parity from golden paths. |

## 4. ROS, build, browser, and CI gaps

**ROS interface generation is missing.** Custom services require actual `.srv` definitions, `rosidl_default_generators`, runtime exports, `ament_cmake`, and an explicit link to generated typesupport. Specify how the installed core exports `ncp::ncp` and its package config, how the wrapper finds it, and how `CMAKE_PREFIX_PATH`/ROS setup files are set. `NCP_BUILD_ROS=ON` alone does not establish a colcon package or discover an installed library. Spell apt names fully (`ros-humble-nav-msgs`, etc.) and include launch/runtime dependencies. The [Humble custom-interface tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) covers the generation workflow.

**“Latched” needs a ROS 2 QoS contract.** Set reliable/transient-local map publication and compatible subscription, keep the publisher alive, and wait for map ingestion before calling the service. Test service-before-map and a later map replacement. Snapshot a map/version for each request to avoid mutation during planning. Use stamped poses/polygons and either validate the map frame or explicitly add transforms; plain `geometry_msgs/Polygon` carries no header. Return valid path headers and quaternion orientations. See [ROS 2 QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html).

**The Python quick start cannot yet import the module.** Define extension output/install placement and a supported install command or explicit `PYTHONPATH`. `cmake --build` followed by pytest does not automatically install `ncp_core`. Disable Python/tests when installing just the ROS core if those artifacts are unnecessary. Document Python development headers and the YAML/JSON parser choices and licenses.

**Golden fixtures are regression data, not an independent oracle.** A C++ bug can generate the expected JSON and pass all three layers. Add hand-derived cases, independent path validity/cost checks, and exhaustive tiny-grid comparisons. Exclude timing from golden comparisons. Include parser/costmap parity, not just search on precomputed grids. The two-second gtest requirement needs an external CTest timeout or equivalent watchdog; an assertion after a hung call cannot enforce it.

**Web CI needs one consistent command contract.** Reconcile `npm test`, Vitest, type checking, build, and Playwright across sections. Install Chromium and OS dependencies (`npx playwright install --with-deps chromium`), start preview from a completed build, wait for readiness, and explicitly capture/upload `demo.png`; Playwright does not create that named screenshot automatically. See [Playwright CI setup](https://playwright.dev/docs/ci-intro).

For repository Pages, configure Vite `base: '/nav2-coverage-planner-compare/'` and build fixture fetch URLs with the base URL. Test the subpath, not just localhost root. Define fixture synchronization before dev/build and avoid two editable copies. Include Pages artifact upload, deployment permissions/environment, and restrict deployment to successful main builds. See [Vite Pages deployment](https://vite.dev/guide/static-deploy.html#github-pages).

Run planners in a Web Worker with a work/map-size cap and cancellation so the canvas remains responsive. Define out-of-order response handling after map/parameter edits. Enable `pull_request` label-change events for the proposed `ci-ros` trigger, and make optional ROS checks compatible with branch protection. A three-minute whole-job cap can be unrealistic for cold dependency installation; distinguish smoke timeout from setup/build timeout.

## 5. Apache-2.0 and reference-license requirements

**Outbound license is Apache-2.0, never MIT.** PLAN.md already says this correctly; preserve it throughout README, package metadata, ROS `package.xml`, site footer, and release artifacts.

1. `LICENSE` must be the verbatim [official Apache-2.0 text](https://www.apache.org/licenses/LICENSE-2.0.txt), including its appendix. Do not substitute a short declaration, rewrite it, or insert project-specific text into the official file. Verify byte equality with the fetched source at repository creation.
2. `NOTICE` must credit **Arpan Bom**, beginning with `nav2-coverage-planner-compare, Copyright 2026 Arpan Bom` as planned. Distinguish optional idea acknowledgments from notices legally required for actual included third-party material. Apache section 4 governs redistribution and retained notices; a “no vendored source” claim must remain accurate if packaging FetchContent sources or other dependencies in release archives.
3. Authored source files need `SPDX-License-Identifier: Apache-2.0` in a syntactically valid comment. Cover C++, Python, TypeScript, CMake, launch files and authored shell/workflow/config files where comments are supported. Preserve required shebang/XML declaration placement. Authored `.pyi` files should receive headers; only actually generated artifacts qualify for the plan's exemption. Never relabel third-party files with the project's license.
4. **GPL/LGPL references are idea-only:** [ethz-asl/polygon_coverage_planning](https://github.com/ethz-asl/polygon_coverage_planning) identifies GPL v3 or later and also documents a noncommercial solver restriction; [Humble nav2_amcl](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/package.xml) declares LGPL-2.1-or-later. Do not copy, translate, vendor, or link their code into this project under the stated project policy. Prefer papers for algorithm descriptions. Correct the ETH reference from an unqualified `GPL-3.0` to its documented “GPL-3.0-or-later” characterization.
5. Do not assign one license to all of Nav2. Track the exact package/file and revision consulted. The plan correctly identifies [nobleo's manifest](https://github.com/nobleo/full_coverage_path_planner/blob/master/package.xml) as Apache 2.0, but it is a catkin/ROS 1 plugin, not a Humble API template. Permissive dependencies retain their own notices; an Apache outbound project does not relicense them. Record dependency/license provenance separately from algorithm idea credits.

These are concrete project requirements and provenance checks; no existing implementation or distribution was audited here.

## 6. Simpler alternatives and recommended sequence

- Implement A* and Dijkstra using one graph-search engine with a selectable heuristic. This makes the baseline comparison directly meaningful and reduces duplicate logic.
- Begin with grid-row boustrophedon-style sweeps plus A* connectors, not a claim of complete polygonal cellular decomposition. Restrict v1 to validated simple regions and obstacle masks; document incomplete boundary coverage rather than introducing a heavy polygon/optimization stack.
- Choose one collision representation and reuse one segment predicate for graph edges, smoothing, RRT*, coverage links, and independent validators.
- If true Lazy Theta* is too large for v1, ship A* plus a clearly named visibility smoother. Keep RRT* as a later milestone with fixed-iteration fixtures. Do not silently rename either algorithm.
- Keep Python as a thin binding as proposed. For the browser, either accept the maintenance cost of a limited TS port or use a fixture-replay viewer first. Replay must be labeled and cannot claim arbitrary live planning. WASM is a reasonable future way to share the core; the plan's assertion that it fails more often is unsupported and should be presented as a project tradeoff, not a general fact.
- Complete the ROS-free core, metric definitions, analytic tests, Apache license/NOTICE/headers, and reproducible quick start first; then add live web planning and the optional Humble wrapper. Keep the existing portfolio/simulation-only claims explicit.

Before coding, rewrite criteria 5 and 9, separate deterministic and deadline RRT* modes, define hard versus soft inflation and coverage semantics, and replace the obsolete web/Python platform assumptions. Those decisions prevent the largest implementation and test rewrites.
