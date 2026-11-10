// SPDX-License-Identifier: Apache-2.0
import { kCostInscribed } from "./constants";
import type { Costmap } from "./costmap";
import { pathLengthM } from "./metrics";
import type { PlanRequest, PlanResult } from "./types";

function octileHeuristic(dx: number, dy: number, res: number, minCostPerM: number): number {
  const adx = Math.abs(dx);
  const ady = Math.abs(dy);
  const m = Math.min(adx, ady);
  const M = Math.max(adx, ady);
  return minCostPerM * res * (M - m + m * Math.SQRT2);
}

function diagonalLegal(map: Costmap, x: number, y: number, nx: number, ny: number): boolean {
  if (map.isHardBlocked(nx, y)) return false;
  if (map.isHardBlocked(x, ny)) return false;
  return true;
}

function stepCost(map: Costmap, x0: number, y0: number, x1: number, y1: number): number {
  const res = map.resolution;
  const diag = x0 !== x1 && y0 !== y1;
  const len = diag ? res * Math.SQRT2 : res;
  const c0 = map.cost(x0, y0);
  const c1 = map.cost(x1, y1);
  if (c0 >= kCostInscribed || c1 >= kCostInscribed) return Infinity;
  const cmax = Math.max(c0, c1);
  return len * (1 + (map.params.w_cost * cmax) / 252);
}

export function planGraphSearch(map: Costmap, req: PlanRequest, useAstar: boolean): PlanResult {
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

  const minCostPerM = 1;
  const res = map.resolution;

  const heuristic = (idx: number) => {
    if (!useAstar) return 0;
    const x = idx % w;
    const y = Math.floor(idx / w);
    return octileHeuristic(gx - x, gy - y, res, minCostPerM);
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
    const { idx: cur } = open.shift()!;
    if (closed[cur]) continue;
    closed[cur] = 1;
    if (cur === goal) break;

    const cx = cur % w;
    const cy = Math.floor(cur / w);
    for (const d of dirs) {
      const nx = cx + d[0]!;
      const ny = cy + d[1]!;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (map.isHardBlocked(nx, ny)) continue;
      const diag = d[0] !== 0 && d[1] !== 0;
      if (diag && !diagonalLegal(map, cx, cy, nx, ny)) continue;
      const ec = stepCost(map, cx, cy, nx, ny);
      if (!Number.isFinite(ec)) continue;
      const nidx = ny * w + nx;
      const tentative = gscore[cur]! + ec;
      if (tentative < gscore[nidx]!) {
        gscore[nidx] = tentative;
        parent[nidx] = cur;
        open.push({ f: tentative + heuristic(nidx), idx: nidx });
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
  while (cur >= 0) {
    path.push(map.cellCenter(cur % w, Math.floor(cur / w)));
    cur = parent[cur]!;
  }
  path.reverse();
  path[0] = req.start;
  path[path.length - 1] = req.goal;

  result.success = true;
  result.path = path;
  result.metrics.length_m = pathLengthM(path);
  result.metrics.objective_cost = gscore[goal]!;
  result.metrics.success = true;
  result.metrics.compute_ms = performance.now() - t0;
  return result;
}
