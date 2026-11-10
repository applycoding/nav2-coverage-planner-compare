// SPDX-License-Identifier: Apache-2.0
// Lazy Theta*: deferred line-of-sight (Nash, Koenig, Likhachev, AAAI 2010).
import { kCostInscribed } from "./constants";
import { lineOfSight } from "./collision";
import type { Costmap } from "./costmap";
import { pathLengthM, pathObjectiveCost } from "./metrics";
import type { PlanRequest, PlanResult } from "./types";

function segmentTraversalCost(map: Costmap, a: { x: number; y: number }, b: { x: number; y: number }): number {
  const len = Math.hypot(b.x - a.x, b.y - a.y);
  const ca = map.worldToCell(a);
  const cb = map.worldToCell(b);
  const cmax = Math.max(ca ? map.cost(ca.x, ca.y) : 255, cb ? map.cost(cb.x, cb.y) : 255);
  if (cmax >= kCostInscribed) return Infinity;
  return len * (1 + (map.params.w_cost * cmax) / 252);
}

export function planLazyThetaStar(map: Costmap, req: PlanRequest): PlanResult {
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

  const sc = map.worldToCell(req.start);
  const gc = map.worldToCell(req.goal);
  if (!sc) {
    result.reason = "out_of_bounds";
    return result;
  }
  if (!gc) {
    result.reason = "out_of_bounds";
    return result;
  }
  const { x: sx, y: sy } = sc;
  const { x: gx, y: gy } = gc;
  if (map.isHardBlocked(sx, sy)) {
    result.reason = "start_in_obstacle";
    return result;
  }
  if (map.isHardBlocked(gx, gy)) {
    result.reason = "goal_in_obstacle";
    return result;
  }
  if (sx === gx && sy === gy) {
    result.success = true;
    result.path = [req.start];
    result.metrics.length_m = 0;
    result.metrics.objective_cost = 0;
    result.metrics.success = true;
    result.metrics.compute_ms = performance.now() - t0;
    return result;
  }

  const w = map.width;
  const h = map.height;
  const n = w * h;
  const start = sy * w + sx;
  const goal = gy * w + gx;
  const gscore = new Float64Array(n).fill(Infinity);
  const parent = new Int32Array(n).fill(-1);
  const closed = new Uint8Array(n);
  gscore[start] = 0;
  parent[start] = start;

  const res = map.resolution;
  const minCostPerM = 1;

  const cellPoint = (idx: number) => map.cellCenter(idx % w, Math.floor(idx / w));

  const heuristic = (idx: number) => {
    const x = idx % w;
    const y = Math.floor(idx / w);
    const adx = Math.abs(gx - x);
    const ady = Math.abs(gy - y);
    const m = Math.min(adx, ady);
    const M = Math.max(adx, ady);
    return minCostPerM * res * (M - m + m * Math.SQRT2);
  };

  const open: { f: number; idx: number }[] = [{ f: heuristic(start), idx: start }];
  const dirs = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
    [1, 1],
    [1, -1],
    [-1, 1],
    [-1, -1],
  ];

  while (open.length > 0) {
    open.sort((a, b) => (a.f !== b.f ? a.f - b.f : a.idx - b.idx));
    const s = open.shift()!.idx;
    if (closed[s]) continue;
    closed[s] = 1;

    const par = parent[s]!;
    if (s !== start) {
      const ppar = parent[par]!;
      if (lineOfSight(map, cellPoint(ppar), cellPoint(s))) {
        const ng = gscore[ppar]! + segmentTraversalCost(map, cellPoint(ppar), cellPoint(s));
        if (ng < gscore[s]!) {
          gscore[s] = ng;
          parent[s] = ppar;
        }
      }
    }

    if (s === goal) break;

    const cx = s % w;
    const cy = Math.floor(s / w);
    for (const d of dirs) {
      const nx = cx + d[0]!;
      const ny = cy + d[1]!;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (map.isHardBlocked(nx, ny)) continue;
      const nidx = ny * w + nx;
      let nc = gscore[s]! + segmentTraversalCost(map, cellPoint(s), cellPoint(nidx));
      if (lineOfSight(map, cellPoint(parent[s]!), cellPoint(nidx))) {
        const np =
          gscore[parent[s]!]! + segmentTraversalCost(map, cellPoint(parent[s]!), cellPoint(nidx));
        nc = Math.min(nc, np);
        if (nc < gscore[nidx]!) parent[nidx] = parent[s]!;
      }
      if (nc < gscore[nidx]!) {
        gscore[nidx] = nc;
        if (parent[nidx] !== parent[s]) parent[nidx] = s;
        open.push({ f: nc + heuristic(nidx), idx: nidx });
      }
    }
  }

  if (!Number.isFinite(gscore[goal]!)) {
    result.reason = "unreachable";
    result.metrics.compute_ms = performance.now() - t0;
    return result;
  }

  const path = [];
  let cur = goal;
  while (cur !== parent[cur]!) {
    path.push(cellPoint(cur));
    cur = parent[cur]!;
  }
  path.push(cellPoint(start));
  path.reverse();
  path[0] = req.start;
  path[path.length - 1] = req.goal;

  result.success = true;
  result.path = path;
  result.metrics.length_m = pathLengthM(path);
  result.metrics.objective_cost = pathObjectiveCost(map, path);
  result.metrics.success = true;
  result.metrics.compute_ms = performance.now() - t0;
  return result;
}
