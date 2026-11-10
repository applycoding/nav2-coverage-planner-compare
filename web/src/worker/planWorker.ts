// SPDX-License-Identifier: Apache-2.0
import { Costmap, planCoverage, planGlobal, type MapJson, type PlanRequest } from "../planners";

const PLANNERS: PlanRequest["planner"][] = ["astar", "dijkstra", "lazy_theta", "rrt_star"];

export interface WorkerPlanMsg {
  kind: "plan" | "compare";
  id: number;
  map: MapJson;
  request?: PlanRequest;
  start?: { x: number; y: number };
  goal?: { x: number; y: number };
  coverage?: boolean;
  coverageOverlap?: number;
}

let activeId = 0;

self.onmessage = (ev: MessageEvent<WorkerPlanMsg>) => {
  const msg = ev.data;
  if (msg.kind !== "plan" && msg.kind !== "compare") return;
  activeId = msg.id;
  const maxCells = 400 * 400;
  if (msg.map.width * msg.map.height > maxCells) {
    self.postMessage({ id: msg.id, error: "map_too_large" });
    return;
  }
  const cmap = Costmap.fromMapJson(msg.map);

  if (msg.kind === "compare" && msg.start && msg.goal) {
    const rows = PLANNERS.map((planner) => {
      const t0 = performance.now();
      const r = planGlobal(cmap, {
        planner,
        start: msg.start!,
        goal: msg.goal!,
        rrt_seed: 42,
        rrt_max_iterations: 3000,
        rrt_max_time_ms: 0,
      });
      return {
        planner,
        success: r.success,
        reason: r.reason,
        length_m: r.metrics.length_m,
        compute_ms: performance.now() - t0,
      };
    });
    if (msg.id !== activeId) return;
    self.postMessage({ id: msg.id, compare: rows });
    return;
  }

  if (!msg.request) return;

  const result =
    msg.coverage
      ? planCoverage(cmap, {
          start: msg.request.start,
          region_polygon: [],
          linker: msg.request.planner,
          overlap_m: msg.coverageOverlap ?? 0,
          stripe_angle: 0,
          rrt_seed: msg.request.rrt_seed,
          rrt_max_iterations: msg.request.rrt_max_iterations,
          rrt_max_time_ms: msg.request.rrt_max_time_ms,
          coverage_success_threshold: 0.95,
        })
      : planGlobal(cmap, msg.request);

  if (msg.id !== activeId) return;
  self.postMessage({ id: msg.id, result });
};
