// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/costmap.hpp"
#include "ncp/types.hpp"

namespace ncp {

double path_length_m(const std::vector<WorldPoint>& path);
double path_objective_cost(const Costmap& map, const std::vector<WorldPoint>& path);
double path_clearance_m(const Costmap& map, const std::vector<WorldPoint>& path,
                        bool& has_obstacles);

}  // namespace ncp
