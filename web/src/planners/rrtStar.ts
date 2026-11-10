// SPDX-License-Identifier: Apache-2.0
import { segmentCenterFree, pointCenterFree } from "./collision";
import type { Costmap } from "./costmap";
import { pathLengthM, pathObjectiveCost } from "./metrics";
import { XorShift32 } from "./rng";
import type { PlanRequest, PlanResult } from "./types";

interface RrtNode {
  p: { x: number; y: number };
  parent: number;
  cost: number;
}

export function planRrtStar(map: Costmap, req: PlanRequest): PlanResult {
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

  if (!pointCenterFree(map, req.start)) {
    result.reason = "start_in_obstacle";
    return result;
  }
  if (!pointCenterFree(map, req.goal)) {
    result.reason = "goal_in_obstacle";
    return result;
  }
  const sc = map.worldToCell(req.start);
  const gc = map.worldToCell(req.goal);
  if (!sc || !gc) {
    result.reason = "out_of_bounds";
    return result;
  }
  if (sc.x === gc.x && sc.y === gc.y) {
    result.success = true;
    result.path = [req.start];
    result.metrics.length_m = 0;
    result.metrics.success = true;
    result.rrt_vertices = 1;
    result.metrics.compute_ms = performance.now() - t0;
    return result;
  }

  const stepSize = 0.15;
  const rewireRadius = 0.4;
  const goalBias = 0.1;
  const goalConnect = stepSize;
  const deadline = req.rrt_max_time_ms > 0;
  const deadlineAt = t0 + req.rrt_max_time_ms;

  const rng = new XorShift32(req.rrt_seed);
  const nodes: RrtNode[] = [{ p: req.start, parent: -1, cost: 0 }];

  const meta = map.mapMeta();
  const xmin = meta.origin_x;
  const ymin = meta.origin_y;
  const xmax = xmin + map.width * map.resolution;
  const ymax = ymin + map.height * map.resolution;

  const samplePoint = () => {
    if (rng.uniform01() < goalBias) return req.goal;
    return {
      x: xmin + rng.uniform01() * (xmax - xmin),
      y: ymin + rng.uniform01() * (ymax - ymin),
    };
  };

  const nearest = (q: { x: number; y: number }) => {
    let best = 0;
    let bestD = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d = Math.hypot(nodes[i]!.p.x - q.x, nodes[i]!.p.y - q.y);
      if (d < bestD - 1e-12) {
        bestD = d;
        best = i;
      }
    }
    return best;
  };

  const steer = (from: { x: number; y: number }, to: { x: number; y: number }) => {
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const len = Math.hypot(dx, dy);
    if (len <= stepSize) return to;
    return { x: from.x + (stepSize * dx) / len, y: from.y + (stepSize * dy) / len };
  };

  let goalIdx = -1;
  const maxIter = req.rrt_max_iterations;

  for (let iter = 0; iter < maxIter; iter++) {
    if (deadline && performance.now() >= deadlineAt) break;

    const target = samplePoint();
    const ni = nearest(target);
    const neu = steer(nodes[ni]!.p, target);
    if (!segmentCenterFree(map, nodes[ni]!.p, neu)) continue;

    let newCost = nodes[ni]!.cost + Math.hypot(neu.x - nodes[ni]!.p.x, neu.y - nodes[ni]!.p.y);
    let parent = ni;
    for (let j = 0; j < nodes.length; j++) {
      const d = Math.hypot(nodes[j]!.p.x - neu.x, nodes[j]!.p.y - neu.y);
      if (d > rewireRadius) continue;
      if (!segmentCenterFree(map, nodes[j]!.p, neu)) continue;
      const c = nodes[j]!.cost + d;
      if (c + 1e-9 < newCost) {
        newCost = c;
        parent = j;
      }
    }

    const newIdx = nodes.length;
    nodes.push({ p: neu, parent, cost: newCost });

    for (let j = 0; j < nodes.length; j++) {
      if (j === newIdx) continue;
      const d = Math.hypot(nodes[j]!.p.x - neu.x, nodes[j]!.p.y - neu.y);
      if (d > rewireRadius) continue;
      if (!segmentCenterFree(map, neu, nodes[j]!.p)) continue;
      const c = newCost + d;
      if (c + 1e-9 < nodes[j]!.cost) {
        nodes[j]!.parent = newIdx;
        nodes[j]!.cost = c;
      }
    }

    const dg = Math.hypot(neu.x - req.goal.x, neu.y - req.goal.y);
    if (dg <= goalConnect && segmentCenterFree(map, neu, req.goal)) {
      goalIdx = newIdx;
    }
  }

  result.rrt_vertices = nodes.length;

  if (goalIdx < 0) {
    result.reason = deadline && nodes.length > 1 ? "timeout_incumbent" : "timeout";
    result.metrics.compute_ms = performance.now() - t0;
    return result;
  }

  const path: { x: number; y: number }[] = [req.goal];
  let cur = goalIdx;
  while (cur >= 0) {
    path.push(nodes[cur]!.p);
    cur = nodes[cur]!.parent;
  }
  path.reverse();
  path[path.length - 1] = req.goal;

  result.success = true;
  result.path = path;
  result.metrics.length_m = pathLengthM(path);
  result.metrics.objective_cost = pathObjectiveCost(map, path);
  result.metrics.success = true;
  result.metrics.compute_ms = performance.now() - t0;
  return result;
}
