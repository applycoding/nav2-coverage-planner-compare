// SPDX-License-Identifier: Apache-2.0
#include "ncp/types.hpp"

namespace ncp {

const char* planner_id_to_string(PlannerId id) {
  switch (id) {
    case PlannerId::AStar:
      return "astar";
    case PlannerId::Dijkstra:
      return "dijkstra";
    case PlannerId::RrtStar:
      return "rrt_star";
    case PlannerId::LazyThetaStar:
      return "lazy_theta";
    default: {
      const PlannerId unknown = id;
      (void)unknown;
      return "astar";
    }
  }
}

PlannerId planner_id_from_string(const std::string& s) {
  if (s == "dijkstra") return PlannerId::Dijkstra;
  if (s == "rrt_star") return PlannerId::RrtStar;
  if (s == "lazy_theta") return PlannerId::LazyThetaStar;
  return PlannerId::AStar;
}

}  // namespace ncp
