# SPDX-License-Identifier: Apache-2.0

# Deviations from PLAN.md

Recorded only when a pinned version or test proves a plan step broken, or when implementation deliberately diverges after verification.

| Date | Item | Plan | Actual | Why |
| --- | --- | --- | --- | --- |
| 2026-11-10 | FetchContent packaging | Download pinned archives with `URL_HASH` SHA-256 recorded in CMake | `FetchContent` with `GIT_REPOSITORY` + commit SHA pins (GoogleTest, pybind11, nlohmann/json, yaml-cpp) | Archive hashes were not pre-recorded in a green CI run; git SHAs match PLAN.md commits and keep builds reproducible. |
| 2026-11-10 | `unknown_row` cases | Single file-level `unknown_policy` | Per-case `unknown_policy` in `cases.json` (lethal vs free) | One PGM must support two policy golden cases without duplicating maps. |
| 2026-11-10 | `unreachable` fixture geometry | Goal behind sealed wall (historical gap) | Full-height wall column at x=10; separate `goal_in_obstacle` case | Prior geometry placed goal in inflated obstacle; goldens now match free start/goal semantics. |
| 2026-11-10 | ROS CI image digest | Pin `ros:humble-ros-base` on first green run | Pinned amd64 digest `sha256:a3f4de78e42cca693a543ece5798aedcf1cae0171fe71c334a047332cc72ef93` from Docker Hub after green workflow 42848353769 | Records the image pulled by CI at verification time. |
| 2026-11-10 | `clearance_m` on failed plans | Metrics default 0.0 | JSON `null` / NaN when no successful path | Avoids misleading zero clearance goldens; successful paths still assert real meters. |
| 2026-11-10 | Pages deploy gate | Deploy on any green `workflow_run` | `github-script` requires both **core** and **web** green on the same `main` SHA | Prevents publishing the site when only one lane succeeded. |
