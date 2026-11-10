// SPDX-License-Identifier: Apache-2.0
export interface WorldPoint {
  x: number;
  y: number;
}

export type PlannerId = "astar" | "dijkstra" | "rrt_star" | "lazy_theta";

export type UnknownPolicy = "lethal" | "free";

export interface CostmapParams {
  inflation_radius_m: number;
  robot_radius_m: number;
  extra_margin_m: number;
  cost_decay: number;
  w_cost: number;
  collide_inscribed: boolean;
  unknown_policy: UnknownPolicy;
}

export interface PlanRequest {
  planner: PlannerId;
  start: WorldPoint;
  goal: WorldPoint;
  rrt_seed: number;
  rrt_max_iterations: number;
  rrt_max_time_ms: number;
}

export interface CoverageRequest {
  start: WorldPoint;
  region_polygon: WorldPoint[];
  linker: PlannerId;
  overlap_m: number;
  stripe_angle: number;
  rrt_seed: number;
  rrt_max_iterations: number;
  rrt_max_time_ms: number;
  coverage_success_threshold: number;
}

export interface Metrics {
  length_m: number;
  objective_cost: number;
  compute_ms: number;
  coverage_ratio_reachable: number;
  coverage_ratio_total: number;
  clearance_m: number;
  success: boolean;
  reason: string;
}

export interface PlanResult {
  success: boolean;
  reason: string;
  path: WorldPoint[];
  metrics: Metrics;
  rrt_vertices: number;
  unreachable_free_ratio: number;
}

export interface MapJson {
  width: number;
  height: number;
  resolution: number;
  origin: [number, number, number];
  costs: number[];
  occupancy: number[];
  params: {
    robot_radius_m: number;
    inflation_radius_m: number;
    w_cost: number;
    cost_decay: number;
    unknown_policy: UnknownPolicy;
  };
}

export const defaultCostmapParams = (): CostmapParams => ({
  inflation_radius_m: 0.2,
  robot_radius_m: 0.1,
  extra_margin_m: 0,
  cost_decay: 5,
  w_cost: 1,
  collide_inscribed: true,
  unknown_policy: "lethal",
});

export function plannerFromString(s: string): PlannerId {
  if (s === "dijkstra") return "dijkstra";
  if (s === "rrt_star") return "rrt_star";
  if (s === "lazy_theta") return "lazy_theta";
  return "astar";
}
