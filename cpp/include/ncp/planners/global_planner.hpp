// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/costmap.hpp"
#include "ncp/types.hpp"

namespace ncp {

class GlobalPlanner {
 public:
  virtual ~GlobalPlanner() = default;
  virtual PlanResult plan(const Costmap& map, const PlanRequest& req) = 0;
};

PlanResult plan_global(const Costmap& map, const PlanRequest& req);

}  // namespace ncp
