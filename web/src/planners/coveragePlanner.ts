// SPDX-License-Identifier: Apache-2.0
import { pointCenterFree } from "./collision";
import type { Costmap } from "./costmap";
import { planGlobal } from "./globalPlanner";
import { pathClearanceM, pathLengthM, pathObjectiveCost } from "./metrics";
import type { CoverageRequest, PlanRequest, PlanResult, WorldPoint } from "./types";

function pointInPolygon(p: WorldPoint, poly: WorldPoint[]): boolean {
  if (poly.length < 3) return true;
  let inside = false;
  let j = poly.length - 1;
  for (let i = 0; i < poly.length; j = i++) {
    const intersect =
      (poly[i]!.y > p.y) !== (poly[j]!.y > p.y) &&
      p.x <
        ((poly[j]!.x - poly[i]!.x) * (p.y - poly[i]!.y)) / (poly[j]!.y - poly[i]!.y + 1e-18) +
          poly[i]!.x;
    if (intersect) inside = !inside;
  }
  return inside;
}

function stripeWaypoints(map: Costmap, yRow: number, x0: number, x1: number, stripeStep: number) {
  const pts: WorldPoint[] = [];
  let a = x0;
  let b = x1;
  if (a > b) [a, b] = [b, a];
  for (let x = a; x <= b + 1e-9; x += stripeStep) {
    const p = { x, y: yRow };
    if (!pointCenterFree(map, p)) continue;
    pts.push(p);
  }
  return pts;
}

export function planCoverage(map: Costmap, req: CoverageRequest): PlanResult {
  const t0 = performance.now();
  const result: PlanResult = {
    success: false,
    reason: "",
    path: [],
    metrics: {
      length_m: 0,
      objective_cost: 0,
      compute_ms: 0,
      coverage_ratio_reachable: 0,
      coverage_ratio_total: 0,
      clearance_m: NaN,
      success: false,
      reason: "",
    },
    rrt_vertices: 0,
    unreachable_free_ratio: 0,
  };

  const robotR = map.params.robot_radius_m;
  if (robotR <= 0 || req.overlap_m < 0 || req.overlap_m >= 2 * robotR) {
    result.reason = "invalid_request";
    return result;
  }
  if (!pointCenterFree(map, req.start)) {
    result.reason = "start_in_obstacle";
    return result;
  }

  const stripeSpacing = 2 * robotR - req.overlap_m;
  const res = map.resolution;
  const meta = map.mapMeta();
  const xmin = meta.origin_x;
  const ymin = meta.origin_y;
  const xmax = xmin + map.width * res;
  const ymax = ymin + map.height * res;

  const rowYs: number[] = [];
  for (let y = ymin + robotR; y <= ymax - robotR; y += stripeSpacing) rowYs.push(y);

  const components: WorldPoint[][] = [];
  let reverse = false;
  for (const y of rowYs) {
    let seg = stripeWaypoints(map, y, xmin + robotR, xmax - robotR, res);
    if (req.region_polygon.length >= 3) {
      seg = seg.filter((p) => pointInPolygon(p, req.region_polygon));
    }
    if (seg.length === 0) continue;
    if (reverse) seg.reverse();
    reverse = !reverse;
    components.push(seg);
  }

  const fullPath: WorldPoint[] = [req.start];
  let cursor = req.start;
  let partial = false;

  for (const comp of components) {
    if (comp.length === 0) continue;
    const entry = comp[0]!;
    const linkReq: PlanRequest = {
      planner: req.linker,
      start: cursor,
      goal: entry,
      rrt_seed: req.rrt_seed,
      rrt_max_iterations: req.rrt_max_iterations,
      rrt_max_time_ms: req.rrt_max_time_ms,
    };
    const link = planGlobal(map, linkReq);
    if (!link.success) {
      partial = true;
      break;
    }
    for (let i = 1; i < link.path.length; i++) fullPath.push(link.path[i]!);
    for (const p of comp) {
      if (
        fullPath.length > 0 &&
        Math.hypot(p.x - fullPath[fullPath.length - 1]!.x, p.y - fullPath[fullPath.length - 1]!.y) <
          1e-6
      ) {
        continue;
      }
      fullPath.push(p);
    }
    cursor = comp[comp.length - 1]!;
  }

  type Cell = string;
  const covered = new Set<Cell>();
  const freeInRegion = new Set<Cell>();
  const reachableFree = new Set<Cell>();

  for (let y = 0; y < map.height; y++) {
    for (let x = 0; x < map.width; x++) {
      if (map.isHardBlocked(x, y)) continue;
      const cp = map.cellCenter(x, y);
      if (req.region_polygon.length >= 3 && !pointInPolygon(cp, req.region_polygon)) continue;
      freeInRegion.add(`${x},${y}`);
    }
  }

  for (const cell of freeInRegion) {
    const [xs, ys] = cell.split(",");
    const x = Number(xs);
    const y = Number(ys);
    const reachReq: PlanRequest = {
      planner: "dijkstra",
      start: req.start,
      goal: map.cellCenter(x, y),
      rrt_seed: 1,
      rrt_max_iterations: 5000,
      rrt_max_time_ms: 0,
    };
    const r = planGlobal(map, reachReq);
    if (r.success) reachableFree.add(cell);
  }

  const brush = Math.max(1, Math.ceil(robotR / res));
  for (let i = 1; i < fullPath.length; i++) {
    const a = fullPath[i - 1]!;
    const b = fullPath[i]!;
    const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / (res * 0.5)));
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      const p = { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) };
      const c = map.worldToCell(p);
      if (!c) continue;
      for (let dy = -brush; dy <= brush; dy++) {
        for (let dx = -brush; dx <= brush; dx++) {
          if (dx * dx + dy * dy > brush * brush) continue;
          const nx = c.x + dx;
          const ny = c.y + dy;
          if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) continue;
          if (map.isHardBlocked(nx, ny)) continue;
          covered.add(`${nx},${ny}`);
        }
      }
    }
  }

  let covReach = 0;
  let covTotal = 0;
  if (reachableFree.size > 0) {
    let hit = 0;
    for (const c of reachableFree) if (covered.has(c)) hit++;
    covReach = hit / reachableFree.size;
  }
  if (freeInRegion.size > 0) {
    let hitT = 0;
    let unreach = 0;
    for (const c of freeInRegion) {
      if (covered.has(c)) hitT++;
      if (!reachableFree.has(c)) unreach++;
    }
    covTotal = hitT / freeInRegion.size;
    result.unreachable_free_ratio = unreach / freeInRegion.size;
  }

  result.path = fullPath;
  result.metrics.length_m = pathLengthM(fullPath);
  result.metrics.objective_cost = pathObjectiveCost(map, fullPath);
  result.metrics.coverage_ratio_reachable = covReach;
  result.metrics.coverage_ratio_total = covTotal;

  result.success = covReach >= req.coverage_success_threshold && !partial;
  result.metrics.success = result.success;
  if (partial || (!result.success && covReach < req.coverage_success_threshold)) {
    result.reason = "partial_coverage";
    result.success = covReach >= req.coverage_success_threshold;
    result.metrics.success = result.success;
  }

  if (fullPath.length > 0) {
    const { clearance, hasObstacles } = pathClearanceM(map, fullPath);
    result.metrics.clearance_m = hasObstacles ? clearance : Number.NaN;
  }

  result.metrics.compute_ms = performance.now() - t0;
  return result;
}
