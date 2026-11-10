// SPDX-License-Identifier: Apache-2.0
#include "ncp/collision.hpp"
#include "ncp/costmap.hpp"
#include "ncp/grid_map.hpp"
#include "ncp/planners/global_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

static void write_map(const std::string& dir, int w, int h, const std::string& occ_row) {
  std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
  pgm << "P5\n# test\n" << w << " " << h << "\n255\n";
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      char c = occ_row[y * w + x];
      unsigned char v = (c == 'O') ? 255 : (c == 'U' ? 128 : 0);
      pgm.write(reinterpret_cast<char*>(&v), 1);
    }
  }
  std::ofstream yaml(dir + "/map.yaml");
  yaml << "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
          "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  pgm.close();
  yaml.close();
}

TEST(GridCostmap, TinyStartEqualsGoal) {
  const std::string dir = "/tmp/ncp_tiny";
  system(("mkdir -p " + dir).c_str());
  write_map(dir, 1, 1, "F");
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  ASSERT_TRUE(err.empty());
  ncp::CostmapParams p;
  p.robot_radius_m = 0.05;
  p.inflation_radius_m = 0.05;
  p.cost_decay = 0;
  auto map = ncp::Costmap::from_grid(grid, p);
  ncp::WorldPoint p0 = map.cell_center(0, 0);
  EXPECT_TRUE(ncp::point_center_free(map, p0));
  ncp::PlanRequest req;
  req.planner = ncp::PlannerId::AStar;
  req.start = p0;
  req.goal = p0;
  auto res = ncp::plan_global(map, req);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.path.size(), 1u);
  EXPECT_NEAR(res.metrics.length_m, 0.0, 1e-9);
}

TEST(GridCostmap, CornerCutDiagonalBlocked) {
  const std::string dir = "/tmp/ncp_corner";
  system(("mkdir -p " + dir).c_str());
  std::string rows = "FOFOOOFOF";
  write_map(dir, 3, 3, rows);
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  ncp::CostmapParams p;
  p.robot_radius_m = 0.05;
  p.inflation_radius_m = 0.05;
  p.cost_decay = 0;
  auto map = ncp::Costmap::from_grid(grid, p);
  ncp::WorldPoint a{0.15, 0.15};
  ncp::WorldPoint b{0.35, 0.35};
  EXPECT_FALSE(ncp::segment_center_free(map, a, b, 0.02));
}

TEST(GridCostmap, SegmentCrossesLethal) {
  const std::string dir = "/tmp/ncp_seg";
  system(("mkdir -p " + dir).c_str());
  write_map(dir, 5, 1, "FFOFF");
  std::string err;
  auto grid = ncp::GridMap::load_pgm_yaml(dir + "/map.pgm", dir + "/map.yaml", err);
  ncp::CostmapParams par;
  par.robot_radius_m = 0.05;
  par.inflation_radius_m = 0.05;
  par.cost_decay = 0;
  auto map = ncp::Costmap::from_grid(grid, par);
  ncp::WorldPoint a{0.05, 0.05};
  ncp::WorldPoint b{0.45, 0.05};
  EXPECT_FALSE(ncp::segment_center_free(map, a, b, 0.02));
}
