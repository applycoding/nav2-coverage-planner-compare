// SPDX-License-Identifier: Apache-2.0
#include "ncp/costmap.hpp"
#include "ncp/grid_map.hpp"
#include "ncp/planners/global_planner.hpp"
#include "ncp/planners/graph_search.hpp"
#include "ncp/types.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <fstream>

static ncp::Costmap load_open_map() {
  const std::string dir = "/tmp/ncp_open";
  system(("mkdir -p " + dir).c_str());
  std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
  const int w = 20, h = 20;
  pgm << "P5\n" << w << " " << h << "\n255\n";
  for (int i = 0; i < w * h; ++i) {
    unsigned char v = 0;
    pgm.write(reinterpret_cast<char*>(&v), 1);
  }
  std::ofstream yaml(dir + "/map.yaml");
  yaml << "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
          "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  pgm.close();
  yaml.close();
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  ncp::CostmapParams p;
  p.robot_radius_m = 0.05;
  p.inflation_radius_m = 0.05;
  p.cost_decay = 0;
  return ncp::Costmap::from_grid(grid, p);
}

static ncp::Costmap load_walled_map() {
  const std::string dir = "/tmp/ncp_wall";
  system(("mkdir -p " + dir).c_str());
  const int w = 20, h = 20;
  std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
  pgm << "P5\n" << w << " " << h << "\n255\n";
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      unsigned char v = (x == 10) ? 255 : 0;
      pgm.write(reinterpret_cast<char*>(&v), 1);
    }
  }
  std::ofstream yaml(dir + "/map.yaml");
  yaml << "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n";
  pgm.close();
  yaml.close();
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  ncp::CostmapParams p;
  p.robot_radius_m = 0.1;
  p.inflation_radius_m = 0.1;
  p.cost_decay = 0;
  return ncp::Costmap::from_grid(grid, p);
}

TEST(Planners, AstarMatchesDijkstraCostOpen) {
  auto map = load_open_map();
  ncp::PlanRequest req;
  req.start = {0.5, 0.5};
  req.goal = {1.5, 1.0};
  req.planner = ncp::PlannerId::AStar;
  auto astar = ncp::plan_global(map, req);
  req.planner = ncp::PlannerId::Dijkstra;
  auto dijk = ncp::plan_global(map, req);
  ASSERT_TRUE(astar.success);
  ASSERT_TRUE(dijk.success);
  EXPECT_NEAR(astar.metrics.objective_cost, dijk.metrics.objective_cost, 1e-6);
}

TEST(Planners, UnreachableWall) {
  auto map = load_walled_map();
  ncp::PlanRequest req;
  req.start = {0.5, 1.0};
  req.goal = {1.5, 1.0};
  req.planner = ncp::PlannerId::AStar;
  const auto t0 = std::chrono::steady_clock::now();
  auto r = ncp::plan_global(map, req);
  const auto ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.reason, "unreachable");
  EXPECT_LT(ms, 2000.0);
}

TEST(Planners, HandDerived10x10) {
  const std::string dir = "/tmp/ncp_hand";
  system(("mkdir -p " + dir).c_str());
  const int w = 10, h = 10;
  std::string occ(w * h, 'F');
  for (int y = 2; y < 8; ++y) occ[y * w + 5] = 'O';
  std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
  pgm << "P5\n" << w << " " << h << "\n255\n";
  for (char c : occ) {
    unsigned char v = (c == 'O') ? 255 : 0;
    pgm.write(reinterpret_cast<char*>(&v), 1);
  }
  std::ofstream yaml(dir + "/map.yaml");
  yaml << "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n";
  pgm.close();
  yaml.close();
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  ncp::CostmapParams p;
  p.robot_radius_m = 0.05;
  p.inflation_radius_m = 0.05;
  p.cost_decay = 0;
  auto map = ncp::Costmap::from_grid(grid, p);
  ncp::PlanRequest req;
  req.start = {0.25, 0.25};
  req.goal = {0.85, 0.85};
  req.planner = ncp::PlannerId::AStar;
  auto r = ncp::plan_global(map, req);
  req.planner = ncp::PlannerId::Dijkstra;
  auto d = ncp::plan_global(map, req);
  ASSERT_TRUE(r.success);
  const double optimal = 1.2242640687119288;
  EXPECT_NEAR(r.metrics.length_m, optimal, 1e-6);
  EXPECT_NEAR(d.metrics.length_m, optimal, 1e-6);
}

TEST(Planners, LazyThetaLengthLeAstar) {
  auto map = load_open_map();
  ncp::PlanRequest req;
  req.start = {0.5, 0.5};
  req.goal = {1.5, 1.0};
  req.planner = ncp::PlannerId::AStar;
  auto astar = ncp::plan_global(map, req);
  req.planner = ncp::PlannerId::LazyThetaStar;
  auto lazy = ncp::plan_global(map, req);
  ASSERT_TRUE(astar.success);
  ASSERT_TRUE(lazy.success);
  EXPECT_LE(lazy.metrics.length_m, astar.metrics.length_m + 1e-6);
}
