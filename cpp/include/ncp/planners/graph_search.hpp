// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/costmap.hpp"
#include "ncp/types.hpp"

namespace ncp {

PlanResult plan_graph_search(const Costmap& map, const PlanRequest& req, bool use_astar);

}  // namespace ncp
