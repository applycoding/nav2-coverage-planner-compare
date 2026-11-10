// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/costmap.hpp"
#include "ncp/types.hpp"

namespace ncp {

PlanResult plan_rrt_star(const Costmap& map, const PlanRequest& req);

}  // namespace ncp
