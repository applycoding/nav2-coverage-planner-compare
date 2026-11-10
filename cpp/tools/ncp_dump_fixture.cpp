// SPDX-License-Identifier: Apache-2.0
#include "ncp/costmap.hpp"
#include "ncp/coverage/coverage_planner.hpp"
#include "ncp/grid_map.hpp"
#include "ncp/planners/global_planner.hpp"
#include "ncp/types.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <string>

using json = nlohmann::json;

static json metrics_to_json(const ncp::Metrics& m, const ncp::PlanResult& r) {
  json j;
  j["length_m"] = m.length_m;
  j["objective_cost"] = m.objective_cost;
  j["coverage_ratio_reachable"] = m.coverage_ratio_reachable;
  j["coverage_ratio_total"] = m.coverage_ratio_total;
  j["success"] = r.success;
  j["reason"] = r.success ? "" : r.reason;
  if (std::isnan(m.clearance_m)) {
    j["clearance_m"] = nullptr;
  } else {
    j["clearance_m"] = m.clearance_m;
  }
  if (r.rrt_vertices > 0) j["rrt_vertices"] = r.rrt_vertices;
  if (r.unreachable_free_ratio > 0) {
    j["unreachable_free_ratio"] = r.unreachable_free_ratio;
  }
  json path = json::array();
  for (const auto& p : r.path) path.push_back({{"x", p.x}, {"y", p.y}});
  j["path"] = path;
  return j;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: ncp_dump_fixture <fixture_dir>\n";
    return 1;
  }
  const std::string dir = argv[1];
  std::string err;
  ncp::GridMap grid =
      ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  if (!err.empty() && grid.width() <= 0) {
    std::cerr << err << "\n";
    return 1;
  }

  std::ifstream cases_in(dir + "/cases.json");
  if (!cases_in) {
    std::cerr << "missing cases.json\n";
    return 1;
  }
  json cases;
  cases_in >> cases;

  ncp::CostmapParams params;
  if (cases.contains("costmap_params")) {
    const auto& cp = cases["costmap_params"];
    params.robot_radius_m = cp.value("robot_radius_m", 0.15);
    params.inflation_radius_m = cp.value("inflation_radius_m", params.robot_radius_m);
    params.cost_decay = cp.value("cost_decay", 0.0);
    params.w_cost = cp.value("w_cost", 1.0);
    if (cp.contains("unknown_policy")) {
      params.unknown_policy = cp["unknown_policy"] == "free" ? ncp::UnknownPolicy::Free
                                                             : ncp::UnknownPolicy::Lethal;
    }
  }

  auto build_map = [&](const json& case_item) {
    ncp::CostmapParams p = params;
    if (case_item.contains("unknown_policy")) {
      p.unknown_policy = case_item["unknown_policy"] == "free" ? ncp::UnknownPolicy::Free
                                                               : ncp::UnknownPolicy::Lethal;
    }
  return ncp::Costmap::from_grid(grid, p);
  };

  ncp::Costmap map = build_map(json::object());

  json map_json;
  map_json["width"] = map.width();
  map_json["height"] = map.height();
  map_json["resolution"] = map.resolution();
  const auto& meta = map.map_meta();
  map_json["origin"] = {meta.origin_x, meta.origin_y, meta.origin_yaw};
  std::vector<uint8_t> costs;
  std::vector<int> occ;
  costs.reserve(static_cast<size_t>(map.width() * map.height()));
  occ.reserve(static_cast<size_t>(map.width() * map.height()));
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      costs.push_back(map.cost(x, y));
      occ.push_back(static_cast<int>(map.occupancy(x, y)));
    }
  }
  map_json["costs"] = costs;
  map_json["occupancy"] = occ;
  const auto& cp = map.params();
  map_json["params"] = {{"robot_radius_m", cp.robot_radius_m},
                        {"inflation_radius_m", cp.inflation_radius_m},
                        {"w_cost", cp.w_cost},
                        {"cost_decay", cp.cost_decay},
                        {"unknown_policy",
                         cp.unknown_policy == ncp::UnknownPolicy::Lethal ? "lethal" : "free"}};
  std::ofstream mj(dir + "/map.json");
  mj << map_json.dump(2);

  json expected = json::object();
  for (const auto& item : cases["cases"]) {
    const std::string id = item["id"];
    map = build_map(item);
    if (item.contains("type") && item["type"] == "coverage") {
      ncp::CoverageRequest creq;
      creq.start.x = item["start"]["x"];
      creq.start.y = item["start"]["y"];
      creq.linker = ncp::planner_id_from_string(item.value("linker", "astar"));
      creq.overlap_m = item.value("overlap_m", 0.0);
      creq.coverage_success_threshold = item.value("coverage_success_threshold", 0.95);
      auto res = ncp::plan_coverage(map, creq);
      expected[id] = metrics_to_json(res.metrics, res);
    } else {
      ncp::PlanRequest preq;
      preq.planner = ncp::planner_id_from_string(item.value("planner", "astar"));
      preq.start.x = item["start"]["x"];
      preq.start.y = item["start"]["y"];
      preq.goal.x = item["goal"]["x"];
      preq.goal.y = item["goal"]["y"];
      preq.rrt_seed = item.value("rrt_seed", 1u);
      preq.rrt_max_iterations = item.value("rrt_max_iterations", 5000);
      preq.rrt_max_time_ms = item.value("rrt_max_time_ms", 0);
      preq.rrt_mode = preq.rrt_max_time_ms > 0 ? ncp::RrtMode::Deadline : ncp::RrtMode::IterationOnly;
      auto res = ncp::plan_global(map, preq);
      expected[id] = metrics_to_json(res.metrics, res);
    }
  }

  std::ofstream ex(dir + "/expected.json");
  ex << expected.dump(2);
  std::cout << "wrote " << dir << "/map.json and expected.json\n";
  return 0;
}
