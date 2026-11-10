// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/costmap.hpp"
#include "ncp/types.hpp"

namespace ncp {

PlanResult plan_coverage(const Costmap& map, const CoverageRequest& req);

}  // namespace ncp
