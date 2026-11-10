// SPDX-License-Identifier: Apache-2.0
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { describe, expect, it } from "vitest";
import { Costmap, planCoverage, planGlobal, plannerFromString, xorshift32ReferenceVector } from "../src/planners";

const FIXTURES = join(dirname(fileURLToPath(import.meta.url)), "../../fixtures/maps");
const MAP_IDS = [
  "hole",
  "narrow_block",
  "narrow_pass",
  "office",
  "office_large_inflation",
  "open",
  "tiny",
  "unknown_row",
  "unreachable",
];

const XORSHIFT32_GOLDEN = [
  11355432, 2836018348, 476557059, 3648046016, 3759983556, 1441438134, 3713466840, 2431644334,
  3120216979, 1067267639, 2989017562, 3379058227, 3060050788, 1108305632, 1958019241, 1084823001,
];

function near(a: number, b: number, eps = 1e-6): boolean {
  return Math.abs(a - b) <= eps;
}

function loadMap(mapId: string, cases: Record<string, unknown>, item: Record<string, unknown>): Costmap {
  const mapJson = JSON.parse(readFileSync(join(FIXTURES, mapId, "map.json"), "utf-8"));
  const cp = { ...(cases.costmap_params as Record<string, unknown>) };
  if (item.unknown_policy) cp.unknown_policy = item.unknown_policy;
  return Costmap.fromMapJson(mapJson, cp);
}

describe("fixture goldens", () => {
  for (const mapId of MAP_IDS) {
    it(`matches ${mapId}`, () => {
      const cases = JSON.parse(readFileSync(join(FIXTURES, mapId, "cases.json"), "utf-8"));
      const expected = JSON.parse(readFileSync(join(FIXTURES, mapId, "expected.json"), "utf-8"));

      for (const item of cases.cases as Record<string, unknown>[]) {
        const id = item.id as string;
        const exp = expected[id];
        const cmap = loadMap(mapId, cases, item);
        let res;
        if (item.type === "coverage") {
          res = planCoverage(cmap, {
            start: item.start as { x: number; y: number },
            region_polygon: [],
            linker: plannerFromString((item.linker as string) ?? "astar"),
            overlap_m: (item.overlap_m as number) ?? 0,
            stripe_angle: 0,
            rrt_seed: 1,
            rrt_max_iterations: 5000,
            rrt_max_time_ms: 0,
            coverage_success_threshold: (item.coverage_success_threshold as number) ?? 0.95,
          });
        } else {
          res = planGlobal(cmap, {
            planner: plannerFromString(item.planner as string),
            start: item.start as { x: number; y: number },
            goal: item.goal as { x: number; y: number },
            rrt_seed: (item.rrt_seed as number) ?? 1,
            rrt_max_iterations: (item.rrt_max_iterations as number) ?? 5000,
            rrt_max_time_ms: (item.rrt_max_time_ms as number) ?? 0,
          });
        }

        expect(res.success).toBe(exp.success);
        if (!res.success) expect(res.reason).toBe(exp.reason);
        expect(near(res.metrics.length_m, exp.length_m)).toBe(true);
        expect(near(res.metrics.objective_cost, exp.objective_cost)).toBe(true);
        expect(res.path.length).toBe(exp.path.length);
        for (let i = 0; i < res.path.length; i++) {
          expect(near(res.path[i]!.x, exp.path[i].x)).toBe(true);
          expect(near(res.path[i]!.y, exp.path[i].y)).toBe(true);
        }
        if (exp.rrt_vertices != null) expect(res.rrt_vertices).toBe(exp.rrt_vertices);
        if ("clearance_m" in exp) {
          if (exp.clearance_m == null) expect(Number.isNaN(res.metrics.clearance_m)).toBe(true);
          else expect(near(res.metrics.clearance_m, exp.clearance_m as number, 1e-4)).toBe(true);
        }
        expect(exp.compute_ms).toBeUndefined();
      }
    });
  }
});

describe("xorshift32", () => {
  it("matches reference vector", () => {
    const v = xorshift32ReferenceVector(42, XORSHIFT32_GOLDEN.length);
    expect(v).toEqual(XORSHIFT32_GOLDEN);
  });
});
