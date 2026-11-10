// SPDX-License-Identifier: Apache-2.0
#include "ncp/metrics.hpp"

#include <cmath>
#include <limits>

namespace ncp {

double path_length_m(const std::vector<WorldPoint>& path) {
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

double edge_objective(const Costmap& map, const WorldPoint& a, const WorldPoint& b) {
  const double res = map.resolution();
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  map.world_to_cell(a, x0, y0);
  map.world_to_cell(b, x1, y1);
  const uint8_t c0 = map.cost(x0, y0);
  const uint8_t c1 = map.cost(x1, y1);
  const uint8_t cmax = std::max(c0, c1);
  const double wc = map.params().w_cost;
  if (cmax >= 253) return len * 1e6;
  return len * (1.0 + wc * static_cast<double>(cmax) / 252.0);
}

double path_objective_cost(const Costmap& map, const std::vector<WorldPoint>& path) {
  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    cost += edge_objective(map, path[i - 1], path[i]);
  }
  return cost;
}

double path_clearance_m(const Costmap& map, const std::vector<WorldPoint>& path,
                        bool& has_obstacles) {
  has_obstacles = false;
  double min_clear = std::numeric_limits<double>::infinity();
  const double res = map.resolution();
  for (size_t i = 1; i < path.size(); ++i) {
    const WorldPoint& a = path[i - 1];
    const WorldPoint& b = path[i];
    const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / (res * 0.5))));
    for (int s = 0; s <= steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      WorldPoint p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
      int cx = 0, cy = 0;
      if (!map.world_to_cell(p, cx, cy)) continue;
      for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
          if (map.occupancy(x, y) != Occupancy::Occupied) continue;
          has_obstacles = true;
          WorldPoint oc = map.cell_center(x, y);
          min_clear = std::min(min_clear, std::hypot(p.x - oc.x, p.y - oc.y));
        }
      }
    }
  }
  if (!has_obstacles) return std::numeric_limits<double>::quiet_NaN();
  return min_clear;
}

}  // namespace ncp
