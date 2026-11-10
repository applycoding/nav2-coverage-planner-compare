// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ncp {

struct WorldPoint {
  double x = 0.0;
  double y = 0.0;
};

enum class Occupancy : uint8_t { Free = 0, Occupied = 1, Unknown = 2 };

enum class UnknownPolicy { Lethal, Free };

enum class PlannerId { AStar, Dijkstra, RrtStar, LazyThetaStar };

enum class RrtMode { IterationOnly, Deadline };

struct CostmapParams {
  double inflation_radius_m = 0.2;
  double robot_radius_m = 0.1;
  double extra_margin_m = 0.0;
  double cost_decay = 5.0;
  double w_cost = 1.0;
  bool collide_inscribed = true;
  UnknownPolicy unknown_policy = UnknownPolicy::Lethal;
};

struct PlanRequest {
  PlannerId planner = PlannerId::AStar;
  WorldPoint start{};
  WorldPoint goal{};
  uint32_t rrt_seed = 1;
  int rrt_max_iterations = 5000;
  int rrt_max_time_ms = 0;
  RrtMode rrt_mode = RrtMode::IterationOnly;
};

struct CoverageRequest {
  WorldPoint start{};
  std::vector<WorldPoint> region_polygon;
  PlannerId linker = PlannerId::AStar;
  double overlap_m = 0.0;
  double stripe_angle = 0.0;
  uint32_t rrt_seed = 1;
  int rrt_max_iterations = 5000;
  int rrt_max_time_ms = 0;
  double coverage_success_threshold = 0.95;
};

struct Metrics {
  double length_m = 0.0;
  double objective_cost = 0.0;
  double compute_ms = 0.0;
  double coverage_ratio_reachable = 0.0;
  double coverage_ratio_total = 0.0;
  double clearance_m = std::numeric_limits<double>::quiet_NaN();
  bool success = false;
  std::string reason;
};

struct PlanResult {
  bool success = false;
  std::string reason;
  std::vector<WorldPoint> path;
  Metrics metrics;
  int rrt_vertices = 0;
  double unreachable_free_ratio = 0.0;
};

const char* planner_id_to_string(PlannerId id);
PlannerId planner_id_from_string(const std::string& s);

}  // namespace ncp
