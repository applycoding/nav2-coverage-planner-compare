// SPDX-License-Identifier: Apache-2.0
#include "ncp/collision.hpp"

#include <cmath>

namespace ncp {

bool point_center_free(const Costmap& map, const WorldPoint& p) {
  int x = 0;
  int y = 0;
  if (!map.world_to_cell(p, x, y)) return false;
  return !map.is_hard_blocked(x, y);
}

bool segment_center_free(const Costmap& map, const WorldPoint& a, const WorldPoint& b,
                         double sample_step_m) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-12) return point_center_free(map, a);
  const int steps = std::max(1, static_cast<int>(std::ceil(len / sample_step_m)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    WorldPoint p{a.x + t * dx, a.y + t * dy};
    if (!point_center_free(map, p)) return false;
  }
  return true;
}

bool line_of_sight(const Costmap& map, const WorldPoint& a, const WorldPoint& b,
                   double sample_step_m) {
  return segment_center_free(map, a, b, sample_step_m);
}

}  // namespace ncp
