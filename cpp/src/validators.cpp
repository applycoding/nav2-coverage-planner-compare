// SPDX-License-Identifier: Apache-2.0
#include "ncp/validators.hpp"

#include "ncp/collision.hpp"
#include "ncp/metrics.hpp"

#include <cmath>

namespace ncp {

ValidationResult validate_path(const Costmap& map, const std::vector<WorldPoint>& path,
                               const WorldPoint& start, const WorldPoint& goal,
                               double max_length_m) {
  ValidationResult vr;
  if (path.empty()) {
    vr.ok = false;
    vr.message = "empty path";
    return vr;
  }
  const auto near = [](const WorldPoint& a, const WorldPoint& b) {
    return std::hypot(a.x - b.x, a.y - b.y) < 1e-3;
  };
  if (!near(path.front(), start)) {
    vr.ok = false;
    vr.message = "start mismatch";
    return vr;
  }
  if (!near(path.back(), goal)) {
    vr.ok = false;
    vr.message = "goal mismatch";
    return vr;
  }
  for (size_t i = 1; i < path.size(); ++i) {
    if (!segment_center_free(map, path[i - 1], path[i])) {
      vr.ok = false;
      vr.message = "segment collision";
      return vr;
    }
  }
  const double len = path_length_m(path);
  if (len > max_length_m || std::isnan(len)) {
    vr.ok = false;
    vr.message = "length bounds";
    return vr;
  }
  return vr;
}

}  // namespace ncp
