// SPDX-License-Identifier: Apache-2.0
#include "ncp/planners/global_planner.hpp"

#include "ncp/planners/graph_search.hpp"
#include "ncp/planners/lazy_theta_star.hpp"
#include "ncp/metrics.hpp"
#include "ncp/planners/rrt_star.hpp"

namespace ncp {

PlanResult plan_global(const Costmap& map, const PlanRequest& req) {
  PlanResult r;
  switch (req.planner) {
    case PlannerId::AStar:
      r = plan_graph_search(map, req, true);
      break;
    case PlannerId::Dijkstra:
      r = plan_graph_search(map, req, false);
      break;
    case PlannerId::LazyThetaStar:
      r = plan_lazy_theta_star(map, req);
      break;
    case PlannerId::RrtStar:
      r = plan_rrt_star(map, req);
      break;
    default:
      r.reason = "invalid_request";
      return r;
  }
  if (r.success && !r.path.empty()) {
    bool has_obs = false;
    r.metrics.clearance_m = path_clearance_m(map, r.path, has_obs);
  } else {
    r.metrics.clearance_m = std::numeric_limits<double>::quiet_NaN();
  }
  if (r.success) {
    r.reason.clear();
  }
  r.metrics.success = r.success;
  r.metrics.reason = r.reason;
  return r;
}

}  // namespace ncp
