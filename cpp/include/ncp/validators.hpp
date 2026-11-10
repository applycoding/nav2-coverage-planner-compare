// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/costmap.hpp"
#include "ncp/types.hpp"

#include <string>

namespace ncp {

struct ValidationResult {
  bool ok = true;
  std::string message;
};

ValidationResult validate_path(const Costmap& map, const std::vector<WorldPoint>& path,
                               const WorldPoint& start, const WorldPoint& goal,
                               double max_length_m = 1e9);

}  // namespace ncp
