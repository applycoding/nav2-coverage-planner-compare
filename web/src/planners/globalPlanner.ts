// SPDX-License-Identifier: Apache-2.0
import type { Costmap } from "./costmap";
import { planGraphSearch } from "./graphSearch";
import { planLazyThetaStar } from "./lazyThetaStar";
import { planRrtStar } from "./rrtStar";
import { pathClearanceM } from "./metrics";
import type { PlanRequest, PlanResult } from "./types";

function finalize(map: Costmap, r: PlanResult): PlanResult {
  if (r.success && r.path.length > 0) {
    const { clearance, hasObstacles } = pathClearanceM(map, r.path);
    r.metrics.clearance_m = hasObstacles ? clearance : Number.NaN;
  }
  return r;
}

export function planGlobal(map: Costmap, req: PlanRequest): PlanResult {
  switch (req.planner) {
    case "astar":
      return finalize(map, planGraphSearch(map, req, true));
    case "dijkstra":
      return finalize(map, planGraphSearch(map, req, false));
    case "lazy_theta":
      return finalize(map, planLazyThetaStar(map, req));
    case "rrt_star":
      return finalize(map, planRrtStar(map, req));
    default:
      return {
        success: false,
        reason: "invalid_request",
        path: [],
        metrics: {
          length_m: 0,
          objective_cost: 0,
          compute_ms: 0,
          coverage_ratio_reachable: 0,
          coverage_ratio_total: 0,
          clearance_m: NaN,
          success: false,
          reason: "invalid_request",
        },
        rrt_vertices: 0,
        unreachable_free_ratio: 0,
      };
  }
}
