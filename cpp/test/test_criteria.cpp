// SPDX-License-Identifier: Apache-2.0
#include "ncp/costmap.hpp"
#include "ncp/coverage/coverage_planner.hpp"
#include "ncp/grid_map.hpp"
#include "ncp/planners/global_planner.hpp"
#include "ncp/rng.hpp"
#include "ncp/rng_golden.hpp"
#include "ncp/types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#ifndef NCP_FIXTURES_DIR
#define NCP_FIXTURES_DIR "fixtures/maps"
#endif

static ncp::Costmap load_fixture(const std::string& map_id, const ncp::CostmapParams& base,
                                 const std::string& unknown_policy = "") {
  const std::string dir = std::string(NCP_FIXTURES_DIR) + "/" + map_id;
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  EXPECT_TRUE(err.empty()) << err;
  ncp::CostmapParams p = base;
  if (unknown_policy == "free") p.unknown_policy = ncp::UnknownPolicy::Free;
  if (unknown_policy == "lethal") p.unknown_policy = ncp::UnknownPolicy::Lethal;
  return ncp::Costmap::from_grid(grid, p);
}

TEST(Criterion5, OpenAstarMatchesDijkstraObjective) {
  ncp::CostmapParams p;
  p.robot_radius_m = 0.15;
  p.inflation_radius_m = 0.15;
  p.cost_decay = 0;
  auto map = load_fixture("open", p);
  ncp::PlanRequest req;
  req.start = {0.5, 0.5};
  req.goal = {1.5, 1.0};
  req.planner = ncp::PlannerId::AStar;
  auto a = ncp::plan_global(map, req);
  req.planner = ncp::PlannerId::Dijkstra;
  auto d = ncp::plan_global(map, req);
  ASSERT_TRUE(a.success);
  EXPECT_NEAR(a.metrics.objective_cost, d.metrics.objective_cost, 1e-6);
}

TEST(Criterion6, UnreachableAndObstacleReasons) {
  ncp::CostmapParams p;
  p.robot_radius_m = 0.1;
  p.inflation_radius_m = 0.1;
  auto map = load_fixture("unreachable", p);
  ncp::PlanRequest req;
  req.planner = ncp::PlannerId::AStar;
  req.start = {0.5, 1.0};
  req.goal = {1.5, 1.0};
  auto u = ncp::plan_global(map, req);
  EXPECT_FALSE(u.success);
  EXPECT_EQ(u.reason, "unreachable");
  req.goal = {1.05, 1.0};
  auto g = ncp::plan_global(map, req);
  EXPECT_FALSE(g.success);
  EXPECT_EQ(g.reason, "goal_in_obstacle");
  int sx = 0, sy = 0;
  map.world_to_cell({1.05, 1.0}, sx, sy);
  map.is_hard_blocked(sx, sy);
  req.goal = {1.5, 1.0};
  req.start = {1.05, 1.0};
  auto s = ncp::plan_global(map, req);
  EXPECT_FALSE(s.success);
  EXPECT_EQ(s.reason, "start_in_obstacle");
}

TEST(Criterion7, NarrowPassVsBlock) {
  ncp::CostmapParams pass_p;
  pass_p.robot_radius_m = 0.15;
  pass_p.inflation_radius_m = 0.15;
  pass_p.cost_decay = 0;
  auto pass_map = load_fixture("narrow_pass", pass_p);
  ncp::PlanRequest req;
  req.planner = ncp::PlannerId::AStar;
  req.start = {0.5, 1.05};
  req.goal = {2.5, 1.05};
  auto ok = ncp::plan_global(pass_map, req);
  EXPECT_TRUE(ok.success);

  ncp::CostmapParams block_p;
  block_p.robot_radius_m = 0.22;
  block_p.inflation_radius_m = 0.22;
  auto block_map = load_fixture("narrow_block", block_p);
  req.start = {0.5, 1.05};
  req.goal = {2.5, 1.05};
  auto bad = ncp::plan_global(block_map, req);
  EXPECT_FALSE(bad.success);
  EXPECT_EQ(bad.reason, "unreachable");
}

TEST(Criterion8, OfficeInflationCorridor) {
  ncp::CostmapParams small;
  small.robot_radius_m = 0.15;
  small.inflation_radius_m = 0.15;
  auto m1 = load_fixture("office", small);
  ncp::PlanRequest req;
  req.planner = ncp::PlannerId::AStar;
  req.start = {0.5, 1.05};
  req.goal = {2.5, 1.05};
  EXPECT_TRUE(ncp::plan_global(m1, req).success);

  ncp::CostmapParams large;
  large.robot_radius_m = 0.22;
  large.inflation_radius_m = 0.22;
  auto m2 = load_fixture("office_large_inflation", large);
  EXPECT_FALSE(ncp::plan_global(m2, req).success);
}

TEST(Criterion9, HoleCoverageReachableRatio) {
  ncp::CostmapParams p;
  p.robot_radius_m = 0.12;
  p.inflation_radius_m = 0.12;
  auto map = load_fixture("hole", p);
  ncp::CoverageRequest creq;
  creq.start = {0.5, 0.5};
  creq.linker = ncp::PlannerId::AStar;
  creq.overlap_m = 0.02;
  creq.coverage_success_threshold = 0.95;
  auto res = ncp::plan_coverage(map, creq);
  EXPECT_GE(res.metrics.coverage_ratio_reachable, 0.95);
  EXPECT_FALSE(std::isnan(res.metrics.clearance_m));
}

TEST(Criterion10, UnknownRowPolicy) {
  ncp::CostmapParams base;
  base.robot_radius_m = 0.08;
  base.inflation_radius_m = 0.08;
  ncp::PlanRequest req;
  req.planner = ncp::PlannerId::AStar;
  req.start = {0.5, 0.05};
  req.goal = {1.5, 0.05};
  auto lethal = load_fixture("unknown_row", base, "lethal");
  EXPECT_FALSE(ncp::plan_global(lethal, req).success);
  auto free = load_fixture("unknown_row", base, "free");
  EXPECT_TRUE(ncp::plan_global(free, req).success);
}

TEST(Criterion11, TinyStartEqGoalAndEmptyMap) {
  ncp::CostmapParams p;
  p.robot_radius_m = 0.05;
  p.inflation_radius_m = 0.05;
  auto map = load_fixture("tiny", p);
  ncp::PlanRequest req;
  req.planner = ncp::PlannerId::AStar;
  req.start = {0.05, 0.05};
  req.goal = {0.05, 0.05};
  auto r = ncp::plan_global(map, req);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.path.size(), 1u);
  EXPECT_NEAR(r.metrics.length_m, 0.0, 1e-9);

  const std::string dir = "/tmp/ncp_empty";
  system(("mkdir -p " + dir).c_str());
  std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
  pgm << "P5\n0 0\n255\n";
  pgm.close();
  std::ofstream yaml(dir + "/map.yaml");
  yaml << "image: map.pgm\nresolution: 0.1\norigin: [0,0,0]\n";
  yaml.close();
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  EXPECT_TRUE(grid.width() <= 0);
  EXPECT_EQ(err, "empty_map");
}

TEST(Criterion12, RrtDeterminismAndTimeout) {
  ncp::CostmapParams p;
  p.robot_radius_m = 0.15;
  p.inflation_radius_m = 0.15;
  auto map = load_fixture("open", p);
  ncp::PlanRequest req;
  req.planner = ncp::PlannerId::RrtStar;
  req.start = {0.5, 0.5};
  req.goal = {1.8, 1.8};
  req.rrt_seed = 42;
  req.rrt_max_iterations = 3000;
  auto r1 = ncp::plan_global(map, req);
  auto r2 = ncp::plan_global(map, req);
  ASSERT_TRUE(r1.success);
  ASSERT_EQ(r1.path.size(), r2.path.size());
  for (size_t i = 0; i < r1.path.size(); ++i) {
    EXPECT_NEAR(r1.path[i].x, r2.path[i].x, 1e-9);
    EXPECT_NEAR(r1.path[i].y, r2.path[i].y, 1e-9);
  }
  req.rrt_seed = 99;
  auto r3 = ncp::plan_global(map, req);
  ASSERT_TRUE(r3.success);
  bool differs = r1.path.size() != r3.path.size();
  if (!differs) {
    for (size_t i = 0; i < r1.path.size(); ++i) {
      if (std::abs(r1.path[i].x - r3.path[i].x) > 1e-6 || std::abs(r1.path[i].y - r3.path[i].y) > 1e-6) {
        differs = true;
        break;
      }
    }
  }
  EXPECT_TRUE(differs);

  auto wall = load_fixture("unreachable", p);
  req.rrt_seed = 7;
  req.rrt_max_iterations = 500;
  req.goal = {1.5, 1.0};
  req.start = {0.5, 1.0};
  auto to = ncp::plan_global(wall, req);
  EXPECT_FALSE(to.success);
  EXPECT_EQ(to.reason, "timeout");
}

TEST(Criterion12, XorShift32GoldenVector) {
  auto v = ncp::xorshift32_reference_vector(ncp::kXorShift32GoldenSeed, ncp::kXorShift32GoldenCount);
  ASSERT_EQ(v.size(), ncp::kXorShift32GoldenCount);
  for (size_t i = 0; i < ncp::kXorShift32GoldenCount; ++i) {
    EXPECT_EQ(v[i], ncp::kXorShift32Golden[i]);
  }
}
