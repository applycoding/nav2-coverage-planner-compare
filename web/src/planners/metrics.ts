// SPDX-License-Identifier: Apache-2.0
import { kCostInscribed } from "./constants";
import type { Costmap } from "./costmap";
import type { WorldPoint } from "./types";

export function pathLengthM(path: WorldPoint[]): number {
  let len = 0;
  for (let i = 1; i < path.length; i++) {
    len += Math.hypot(path[i]!.x - path[i - 1]!.x, path[i]!.y - path[i - 1]!.y);
  }
  return len;
}

export function edgeObjective(map: Costmap, a: WorldPoint, b: WorldPoint): number {
  const len = Math.hypot(b.x - a.x, b.y - a.y);
  const ca = map.worldToCell(a);
  const cb = map.worldToCell(b);
  const c0 = ca ? map.cost(ca.x, ca.y) : 255;
  const c1 = cb ? map.cost(cb.x, cb.y) : 255;
  const cmax = Math.max(c0, c1);
  const wc = map.params.w_cost;
  if (cmax >= 253) return len * 1e6;
  return len * (1 + (wc * cmax) / 252);
}

export function pathObjectiveCost(map: Costmap, path: WorldPoint[]): number {
  let cost = 0;
  for (let i = 1; i < path.length; i++) {
    cost += edgeObjective(map, path[i - 1]!, path[i]!);
  }
  return cost;
}

export function pathClearanceM(map: Costmap, path: WorldPoint[]): { clearance: number; hasObstacles: boolean } {
  let minClear = Infinity;
  let hasObstacles = false;
  const res = map.resolution;
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1]!;
    const b = path[i]!;
    const segLen = Math.hypot(b.x - a.x, b.y - a.y);
    const steps = Math.max(1, Math.ceil(segLen / (res * 0.5)));
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      const px = a.x + t * (b.x - a.x);
      const py = a.y + t * (b.y - a.y);
      for (let y = 0; y < map.height; y++) {
        for (let x = 0; x < map.width; x++) {
          if (map.occupancyAt(x, y) !== 1) continue;
          hasObstacles = true;
          const c = map.cellCenter(x, y);
          minClear = Math.min(minClear, Math.hypot(px - c.x, py - c.y));
        }
      }
    }
  }
  if (!hasObstacles) return { clearance: NaN, hasObstacles: false };
  return { clearance: minClear, hasObstacles: true };
}
