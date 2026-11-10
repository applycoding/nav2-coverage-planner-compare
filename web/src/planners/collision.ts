// SPDX-License-Identifier: Apache-2.0
import type { Costmap } from "./costmap";
import type { WorldPoint } from "./types";

export function pointCenterFree(map: Costmap, p: WorldPoint): boolean {
  const c = map.worldToCell(p);
  if (!c) return false;
  return !map.isHardBlocked(c.x, c.y);
}

export function segmentCenterFree(
  map: Costmap,
  a: WorldPoint,
  b: WorldPoint,
  sampleStepM = 0.05,
): boolean {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len < 1e-12) return pointCenterFree(map, a);
  const steps = Math.max(1, Math.ceil(len / sampleStepM));
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const p = { x: a.x + t * dx, y: a.y + t * dy };
    if (!pointCenterFree(map, p)) return false;
  }
  return true;
}

export function lineOfSight(map: Costmap, a: WorldPoint, b: WorldPoint, step = 0.05): boolean {
  return segmentCenterFree(map, a, b, step);
}
