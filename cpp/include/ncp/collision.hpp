// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/costmap.hpp"
#include "ncp/types.hpp"

namespace ncp {

bool point_center_free(const Costmap& map, const WorldPoint& p);
bool segment_center_free(const Costmap& map, const WorldPoint& a, const WorldPoint& b,
                         double sample_step_m = 0.02);
bool line_of_sight(const Costmap& map, const WorldPoint& a, const WorldPoint& b,
                   double sample_step_m = 0.02);

}  // namespace ncp
