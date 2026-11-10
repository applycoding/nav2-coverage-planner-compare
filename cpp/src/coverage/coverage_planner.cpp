// SPDX-License-Identifier: Apache-2.0
#include "ncp/coverage/coverage_planner.hpp"

#include "ncp/collision.hpp"
#include "ncp/metrics.hpp"
#include "ncp/planners/global_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <utility>
#include <vector>

namespace ncp {

static bool point_in_polygon(const WorldPoint& p, const std::vector<WorldPoint>& poly) {
  if (poly.size() < 3) return true;
  bool inside = false;
  size_t j = poly.size() - 1;
  for (size_t i = 0; i < poly.size(); j = i++) {
    const bool intersect =
        ((poly[i].y > p.y) != (poly[j].y > p.y)) &&
        (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y + 1e-18) +
                 poly[i].x);
    if (intersect) inside = !inside;
  }
  return inside;
}

static std::vector<WorldPoint> stripe_waypoints(const Costmap& map, double y_row, double x0,
                                                 double x1, double stripe_step) {
  std::vector<WorldPoint> pts;
  if (x0 > x1) std::swap(x0, x1);
  for (double x = x0; x <= x1 + 1e-9; x += stripe_step) {
    WorldPoint p{x, y_row};
    if (!point_center_free(map, p)) continue;
    pts.push_back(p);
  }
  return pts;
}

PlanResult plan_coverage(const Costmap& map, const CoverageRequest& req) {
  PlanResult result;
  const auto t0 = std::chrono::steady_clock::now();

  const double robot_r = map.params().robot_radius_m;
  if (robot_r <= 0.0 || req.overlap_m < 0.0 || req.overlap_m >= 2.0 * robot_r) {
    result.reason = "invalid_request";
    return result;
  }
  if (!point_center_free(map, req.start)) {
    result.reason = "start_in_obstacle";
    return result;
  }

  const double stripe_spacing = 2.0 * robot_r - req.overlap_m;
  const double res = map.resolution();
  const double xmin = map.map_meta().origin_x;
  const double ymin = map.map_meta().origin_y;
  const double xmax = xmin + map.width() * res;
  const double ymax = ymin + map.height() * res;

  std::vector<double> row_ys;
  for (double y = ymin + robot_r; y <= ymax - robot_r; y += stripe_spacing) {
    row_ys.push_back(y);
  }

  std::vector<std::vector<WorldPoint>> components;
  bool reverse = false;
  for (double y : row_ys) {
    std::vector<WorldPoint> seg = stripe_waypoints(map, y, xmin + robot_r, xmax - robot_r, res);
    if (req.region_polygon.size() >= 3) {
      std::vector<WorldPoint> filtered;
      for (const auto& p : seg) {
        if (point_in_polygon(p, req.region_polygon)) filtered.push_back(p);
      }
      seg = std::move(filtered);
    }
    if (seg.empty()) continue;
    if (reverse) std::reverse(seg.begin(), seg.end());
    reverse = !reverse;
    components.push_back(std::move(seg));
  }

  std::vector<WorldPoint> full_path;
  full_path.push_back(req.start);
  WorldPoint cursor = req.start;
  bool partial = false;

  PlanRequest link_req;
  link_req.planner = req.linker;
  link_req.rrt_seed = req.rrt_seed;
  link_req.rrt_max_iterations = req.rrt_max_iterations;
  link_req.rrt_max_time_ms = req.rrt_max_time_ms;
  link_req.rrt_mode =
      req.rrt_max_time_ms > 0 ? RrtMode::Deadline : RrtMode::IterationOnly;

  for (const auto& comp : components) {
    if (comp.empty()) continue;
    const WorldPoint& entry = comp.front();
    link_req.start = cursor;
    link_req.goal = entry;
    const PlanResult link = plan_global(map, link_req);
    if (!link.success) {
      partial = true;
      break;
    }
    for (size_t i = 1; i < link.path.size(); ++i) full_path.push_back(link.path[i]);
    for (const auto& p : comp) {
      if (!full_path.empty() &&
          std::hypot(p.x - full_path.back().x, p.y - full_path.back().y) < 1e-6) {
        continue;
      }
      full_path.push_back(p);
    }
    cursor = comp.back();
  }

  using Cell = std::pair<int, int>;
  std::set<Cell> covered;
  std::set<Cell> free_in_region;
  std::set<Cell> reachable_free;

  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.is_hard_blocked(x, y)) continue;
      const WorldPoint cp = map.cell_center(x, y);
      if (req.region_polygon.size() >= 3 && !point_in_polygon(cp, req.region_polygon)) continue;
      free_in_region.insert({x, y});
    }
  }

  PlanRequest reach_req;
  reach_req.planner = PlannerId::Dijkstra;
  reach_req.start = req.start;
  for (const auto& cell : free_in_region) {
    reach_req.goal = map.cell_center(cell.first, cell.second);
    const PlanResult r = plan_global(map, reach_req);
    if (r.success) reachable_free.insert(cell);
  }

  const int brush = std::max(1, static_cast<int>(std::ceil(robot_r / res)));
  for (size_t i = 1; i < full_path.size(); ++i) {
    const WorldPoint& a = full_path[i - 1];
    const WorldPoint& b = full_path[i];
    const int steps =
        std::max(1, static_cast<int>(std::hypot(b.x - a.x, b.y - a.y) / (res * 0.5)));
    for (int s = 0; s <= steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      WorldPoint p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
      int cx = 0, cy = 0;
      if (!map.world_to_cell(p, cx, cy)) continue;
      for (int dy = -brush; dy <= brush; ++dy) {
        for (int dx = -brush; dx <= brush; ++dx) {
          if (dx * dx + dy * dy > brush * brush) continue;
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= map.width() || ny >= map.height()) continue;
          if (map.is_hard_blocked(nx, ny)) continue;
          covered.insert({nx, ny});
        }
      }
    }
  }

  double cov_reach = 0.0;
  double cov_total = 0.0;
  if (!reachable_free.empty()) {
    size_t hit_r = 0;
    for (const auto& c : reachable_free) {
      if (covered.count(c)) hit_r++;
    }
    cov_reach = static_cast<double>(hit_r) / static_cast<double>(reachable_free.size());
  }
  if (!free_in_region.empty()) {
    size_t hit_t = 0;
    size_t unreach = 0;
    for (const auto& c : free_in_region) {
      if (covered.count(c)) hit_t++;
      if (!reachable_free.count(c)) unreach++;
    }
    cov_total = static_cast<double>(hit_t) / static_cast<double>(free_in_region.size());
    result.unreachable_free_ratio = static_cast<double>(unreach) / free_in_region.size();
  }

  result.path = full_path;
  result.metrics.length_m = path_length_m(full_path);
  result.metrics.objective_cost = path_objective_cost(map, full_path);
  result.metrics.coverage_ratio_reachable = cov_reach;
  result.metrics.coverage_ratio_total = cov_total;
  if (!full_path.empty()) {
    bool has_obs = false;
    result.metrics.clearance_m = path_clearance_m(map, full_path, has_obs);
  } else {
    result.metrics.clearance_m = std::numeric_limits<double>::quiet_NaN();
  }

  result.success = cov_reach >= req.coverage_success_threshold && !partial;
  result.metrics.success = result.success;
  if (partial || (!result.success && cov_reach < req.coverage_success_threshold)) {
    result.reason = "partial_coverage";
    result.success = cov_reach >= req.coverage_success_threshold;
    result.metrics.success = result.success;
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.metrics.compute_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace ncp
