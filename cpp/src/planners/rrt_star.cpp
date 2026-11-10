// SPDX-License-Identifier: Apache-2.0
#include "ncp/planners/rrt_star.hpp"

#include "ncp/collision.hpp"
#include "ncp/metrics.hpp"
#include "ncp/rng.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace ncp {

struct RrtNode {
  WorldPoint p;
  int parent = -1;
  double cost = 0.0;
};

PlanResult plan_rrt_star(const Costmap& map, const PlanRequest& req) {
  PlanResult result;
  const auto t0 = std::chrono::steady_clock::now();

  if (!point_center_free(map, req.start)) {
    result.reason = "start_in_obstacle";
    return result;
  }
  if (!point_center_free(map, req.goal)) {
    result.reason = "goal_in_obstacle";
    return result;
  }
  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!map.world_to_cell(req.start, sx, sy) || !map.world_to_cell(req.goal, gx, gy)) {
    result.reason = "out_of_bounds";
    return result;
  }
  if (sx == gx && sy == gy) {
    result.success = true;
    result.path = {req.start};
    result.metrics.length_m = 0.0;
    result.metrics.success = true;
    result.rrt_vertices = 1;
    const auto t1 = std::chrono::steady_clock::now();
    result.metrics.compute_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
  }

  const double step_size = 0.15;
  const double rewire_radius = 0.4;
  const double goal_bias = 0.1;
  const double goal_connect = step_size;
  const bool deadline =
      req.rrt_mode == RrtMode::Deadline && req.rrt_max_time_ms > 0;
  const auto deadline_tp =
      t0 + std::chrono::milliseconds(deadline ? req.rrt_max_time_ms : 0);

  XorShift32 rng(req.rrt_seed);
  std::vector<RrtNode> nodes;
  nodes.push_back({req.start, -1, 0.0});

  const double xmin = map.map_meta().origin_x;
  const double ymin = map.map_meta().origin_y;
  const double xmax = xmin + map.width() * map.resolution();
  const double ymax = ymin + map.height() * map.resolution();

  auto sample_point = [&]() -> WorldPoint {
    if (rng.uniform01() < goal_bias) return req.goal;
    WorldPoint p;
    p.x = xmin + rng.uniform01() * (xmax - xmin);
    p.y = ymin + rng.uniform01() * (ymax - ymin);
    return p;
  };

  auto nearest = [&](const WorldPoint& q) -> int {
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < nodes.size(); ++i) {
      const double d = std::hypot(nodes[i].p.x - q.x, nodes[i].p.y - q.y);
      if (d < best_d - 1e-12) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    return best;
  };

  auto steer = [&](const WorldPoint& from, const WorldPoint& to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double len = std::hypot(dx, dy);
    if (len <= step_size) return to;
    WorldPoint p;
    p.x = from.x + step_size * dx / len;
    p.y = from.y + step_size * dy / len;
    return p;
  };

  int goal_idx = -1;
  const int max_iter = req.rrt_max_iterations;

  for (int iter = 0; iter < max_iter; ++iter) {
    if (deadline && std::chrono::steady_clock::now() >= deadline_tp) break;

    const WorldPoint target = sample_point();
    const int ni = nearest(target);
    WorldPoint neu = steer(nodes[ni].p, target);
    if (!segment_center_free(map, nodes[ni].p, neu)) continue;

    double new_cost = nodes[ni].cost + std::hypot(neu.x - nodes[ni].p.x, neu.y - nodes[ni].p.y);
    int parent = ni;
    for (size_t j = 0; j < nodes.size(); ++j) {
      const double d = std::hypot(nodes[j].p.x - neu.x, nodes[j].p.y - neu.y);
      if (d > rewire_radius) continue;
      if (!segment_center_free(map, nodes[j].p, neu)) continue;
      const double c = nodes[j].cost + d;
      if (c + 1e-9 < new_cost) {
        new_cost = c;
        parent = static_cast<int>(j);
      }
    }

    RrtNode node{neu, parent, new_cost};
    const int new_idx = static_cast<int>(nodes.size());
    nodes.push_back(node);

    for (size_t j = 0; j < nodes.size(); ++j) {
      if (static_cast<int>(j) == new_idx) continue;
      const double d = std::hypot(nodes[j].p.x - neu.x, nodes[j].p.y - neu.y);
      if (d > rewire_radius) continue;
      if (!segment_center_free(map, neu, nodes[j].p)) continue;
      const double c = new_cost + d;
      if (c + 1e-9 < nodes[j].cost) {
        nodes[j].parent = new_idx;
        nodes[j].cost = c;
      }
    }

  const double dg = std::hypot(neu.x - req.goal.x, neu.y - req.goal.y);
    if (dg <= goal_connect && segment_center_free(map, neu, req.goal)) {
      goal_idx = new_idx;
    }
  }

  result.rrt_vertices = static_cast<int>(nodes.size());

  if (goal_idx < 0) {
    if (deadline && result.rrt_vertices > 1) {
      result.reason = "timeout_incumbent";
    } else {
      result.reason = deadline ? "timeout" : "timeout";
    }
    const auto t1 = std::chrono::steady_clock::now();
    result.metrics.compute_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
  }

  std::vector<WorldPoint> path;
  int cur = goal_idx;
  path.push_back(req.goal);
  while (cur >= 0) {
    path.push_back(nodes[cur].p);
    cur = nodes[cur].parent;
  }
  std::reverse(path.begin(), path.end());
  path.back() = req.goal;

  result.success = true;
  result.path = path;
  result.metrics.length_m = path_length_m(path);
  result.metrics.objective_cost = path_objective_cost(map, path);
  result.metrics.success = true;
  const auto t1 = std::chrono::steady_clock::now();
  result.metrics.compute_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace ncp
