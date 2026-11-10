# References

Permissively licensed GitHub projects used as **references only**. Do not copy source. Credit any conceptually derived idea in the README. Licenses were checked on 2026-11-10 with `gh api repos/OWNER/REPO --jq .license.spdx_id`, and with the raw LICENSE or package.xml when GitHub reported `NOASSERTION`.

**Outbound license for this demo repo is Apache-2.0**, not MIT. A LICENSE file with the official Apache-2.0 text, a NOTICE file crediting Arpan Bom, and SPDX-License-Identifier: Apache-2.0 headers are required at build time.

This demo must not duplicate existing applycoding repos:

| Repo | What it already is | How this demo differs |
| --- | --- | --- |
| [applycoding/ros2-nav2-slam-sim](https://github.com/applycoding/ros2-nav2-slam-sim) | ROS 2 Humble indoor robot with Gazebo Classic, slam_toolbox, Nav2, and a YAML waypoint mission runner | No Gazebo, no SLAM, no AMCL, no mission FSM. This repo compares planners on static Nav2-style costmaps. |
| [applycoding/px4-drone-mission-planner](https://github.com/applycoding/px4-drone-mission-planner) | Aerial lawnmower over a GPS polygon, MAVSDK upload, geofence and RTH energy checks | Coverage here is indoor occupancy-grid / costmap, not a GPS survey, and has no PX4 or MAVSDK. |
| [applycoding/multi-robot-warehouse-planner](https://github.com/applycoding/multi-robot-warehouse-planner) | Multi-agent CBS warehouse fleet with allocation and batteries | Single-robot planner comparison only. No MAPF, no fleet, no charging. |
| [applycoding/ros2-drone-mission-sim](https://github.com/applycoding/ros2-drone-mission-sim) | Drone waypoint FSM, geofence, battery, kinematic simulator | Ground robot path / coverage planning on costmaps, not a flight mission stack. |

## Selected references (8)

### 1. AtsushiSakai/PythonRobotics

- URL: https://github.com/AtsushiSakai/PythonRobotics
- License: **MIT** (LICENSE file starts "The MIT License (MIT)", copyright 2016–now Atsushi Sakai and contributors). `gh api --jq .license.spdx_id` returns `NOASSERTION`; use the file, not the API label.
- LICENSE: https://github.com/AtsushiSakai/PythonRobotics/blob/master/LICENSE
- Textbook: https://atsushisakai.github.io/PythonRobotics/index.html
- Borrow: textbook-style illustrations of grid A\*, Dijkstra, RRT\*, and grid-based coverage path planning; search-tree and path animations that make algorithm differences obvious. Implement our own code; do not lift the Python samples.

### 2. ros-navigation/navigation2 (NavFn, Smac 2D, Theta\*)

- URL: https://github.com/ros-navigation/navigation2
- License: **mixed**. Repo LICENSE says unmarked files are Apache-2.0; some packages also list BSD-3-Clause or LGPL-2.1-or-later. API SPDX is `NOASSERTION`. Packages we would study (all Apache-2.0 and/or BSD-3-Clause, compatible with our Apache-2.0 outbound license):
  - `nav2_navfn_planner`: Apache-2.0 and BSD-3-Clause (`package.xml`)
  - `nav2_smac_planner` (including SmacPlanner2D): Apache-2.0
  - `nav2_theta_star_planner` (Lazy Theta\* P): Apache-2.0
  - `nav2_costmap_2d`: BSD-3-Clause and Apache-2.0
- **LGPL, idea-only:** `nav2_amcl` is LGPL-2.1-or-later (`package.xml` checked 2026-11-10). Do not copy or derive code from AMCL or any other LGPL Nav2 package. This demo has no localization anyway. Treat any LGPL Nav2 package as idea-only.
- Docs (checked 2026-11-10): the old paths `https://docs.nav2.org/configuration/packages/configuring-navfn.html` (and smac-planner / thetastar) now 404. `https://docs.nav2.org/` redirects to `/rolling/`. Current pages:
  - NavFn: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/ (also under `/jazzy/`; no `/humble/` tree on this site)
  - Smac (overview): https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/
  - Smac 2D: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/
  - Theta*: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/
  - Costmap 2D: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/
  - Inflation: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/
  - Humble package page: https://docs.ros.org/en/humble/p/nav2_navfn_planner/
- Borrow: occupancy-grid cost semantics (free, inscribed, lethal, unknown); inflation radius and cost decay; NavFn-style Dijkstra / A\* on a weighted costmap; Lazy Theta\* line-of-sight shortcuts that remove grid zigzag; Smac 2D eight-connected cost-aware A\*. We implement simplified teaching versions, not a Nav2 plugin fork. Hybrid-A\* and State Lattice are out of scope for v1.

### 3. Fields2Cover/Fields2Cover

- URL: https://github.com/Fields2Cover/Fields2Cover
- License: **BSD-3-Clause** (`gh api` SPDX: BSD-3-Clause)
- Docs: https://fields2cover.github.io
- Paper: https://doi.org/10.1109/LRA.2023.3248439
- Borrow: the idea of a modular coverage pipeline (swath generation, route, path) and metrics for coverage completeness vs path length; boustrophedon / snake / spiral patterns and obstacle-aware cell decomposition as *design* references. Do not vendor Fields2Cover, GDAL, GEOS or OR-tools into this demo. Our coverage planner is a simpler grid lawnmower plus stripe-linking through a global planner.

### 4. open-navigation/opennav_coverage

- URL: https://github.com/open-navigation/opennav_coverage
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0)
- Borrow: how a coverage planner is exposed as a Nav2-style task / action server (polygon in, path and debug markers out) and how coverage sits beside global planners in a ROS 2 stack. Our ROS 2 wrapper should feel familiar to a Nav2 user without depending on Fields2Cover.

### 5. nobleo/full_coverage_path_planner

- URL: https://github.com/nobleo/full_coverage_path_planner
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0)
- Borrow: grid coverage that treats obstacles inside the region as holes, then connects coverage stripes with a point-to-point planner; spiral vs boustrophedon as a toggle. Useful for the "coverage % after linking" metric and for narrow-passage stripe connection.

### 6. anvaka/ngraph.path and anvaka/ngraph.path.demo

- URLs: https://github.com/anvaka/ngraph.path , https://github.com/anvaka/ngraph.path.demo
- License: **MIT** for both (`gh api` SPDX: MIT)
- Demo: https://anvaka.github.io/ngraph.path.demo/ (confirm at build time)
- Borrow: browser UX for path search at interactive speed: start/goal dragging, algorithm switching, and a clear animation of exploration vs the final path. Rebuild a Canvas 2D UI for occupancy grids and coverage swaths; do not use the ngraph graph model (our maps are grids, not general graphs).

### 7. bgrins/javascript-astar

- URL: https://github.com/bgrins/javascript-astar
- License: **MIT** (`gh api` SPDX: MIT)
- Demo: https://briangrinstead.com/files/astar/ (confirm at build time)
- Borrow: a small, readable browser A\* with a binary heap and an 8-connected grid, plus the idea of a static GitHub Pages / static-file demo with no backend. Our TypeScript A\* should be written from the algorithm, then checked against C++ golden fixtures, not copied from this repo.

### 8. motion-planning/rrt-algorithms

- URL: https://github.com/motion-planning/rrt-algorithms
- License: **MIT** (`gh api` SPDX: MIT)
- Borrow: RRT\* structure (nearest, steer, rewire, goal sampling), a seeded RNG so runs are deterministic in tests, and a clear iteration / time cap with a success flag when the goal is not reached. We implement a 2D occupancy-grid RRT\* (plus optional informed sampling later). Do not copy the n-dimensional Python package.

## Looked at, not used as primary references

| Project | License | Why not primary |
| --- | --- | --- |
| [qiao/PathFinding.js](https://github.com/qiao/PathFinding.js) | **No license** (no LICENSE file, `package.json` has no `license` field, GitHub license API 404) | Cannot rely on an unlicensed project. |
| [clementmihailescu/Pathfinding-Visualizer](https://github.com/clementmihailescu/Pathfinding-Visualizer) | none listed | Same risk. |
| [ethz-asl/polygon_coverage_planning](https://github.com/ethz-asl/polygon_coverage_planning) | GPL-3.0 (GitHub SPDX). README: GPL-3.0-or-later because of CGAL, and CGAL's exact geometric operations are free only for non-commercial use. | **Idea-only.** Incompatible with Apache-2.0 if code were derived. Do not copy, translate, vendor, or link. |
| [ompl/ompl](https://github.com/ompl/ompl) | BSD-3-Clause (LICENSE file; API SPDX `NOASSERTION`) | Excellent RRT\* / Informed RRT\* reference, but heavy. Optional later reading only. |
| [zhm-real/PathPlanning](https://github.com/zhm-real/PathPlanning) | MIT | Good animation gallery; overlaps PythonRobotics. Kept as a backup visual reference. |
| [AbdallahHemdan/Pathfinding-Visualizer](https://github.com/AbdallahHemdan/Pathfinding-Visualizer) | MIT | Solid Vanilla JS UX; overlaps ngraph.path.demo and javascript-astar. Backup UI reference. |

## Credit rule

README must list these projects, their licenses, and a one-line "what we took as an idea". Any file that is conceptually derived gets a short comment or docs note pointing at the source paper or repo. No vendored third-party planner source.


## Compatibility with our Apache-2.0 outbound license

We do not vendor third-party planner source. Even so, every primary reference was checked so that later attribution or a short derived comment would stay legal under Apache-2.0.

| Inbound license | Compatible with Apache-2.0 outbound? | Our use |
| --- | --- | --- |
| Apache-2.0 | Yes | opennav_coverage, nobleo/full_coverage_path_planner, Nav2 packages listed above |
| MIT | Yes, keep copyright and permission notice | PythonRobotics, ngraph.path, ngraph.path.demo, javascript-astar, rrt-algorithms, zhm-real/PathPlanning, AbdallahHemdan/Pathfinding-Visualizer |
| BSD-3-Clause | Yes, keep copyright, conditions, and disclaimer | Fields2Cover, OMPL, some Nav2 packages (NavFn, costmap_2d) |
| LGPL-2.1-or-later | **No** for copied or derived source (cannot relicense as Apache-2.0). Dynamic linking of an unmodified LGPL library would be a different, unused path. | **Idea-only.** Confirmed: `nav2_amcl`. Do not read those sources to implement our planners. |
| GPL-3.0 / GPL-3.0-or-later | **No** for copied or derived source. | **Idea-only.** ethz-asl/polygon_coverage_planning (SPDX GPL-3.0; README says GPL-3.0-or-later due to CGAL, with a non-commercial restriction on that dependency). Already excluded as a primary reference. |
| No license / unlicensed | Cannot rely on it. | qiao/PathFinding.js, clementmihailescu/Pathfinding-Visualizer. Do not use. |

All eight primary references are MIT, BSD-3-Clause, and/or Apache-2.0. No primary reference is GPL or LGPL.
