// SPDX-License-Identifier: Apache-2.0
#include "ncp/costmap.hpp"
#include "ncp/coverage/coverage_planner.hpp"
#include "ncp/grid_map.hpp"

#include <gtest/gtest.h>

#include <fstream>

TEST(Coverage, InvalidOverlap) {
  const std::string dir = "/tmp/ncp_cov";
  system(("mkdir -p " + dir).c_str());
  std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
  pgm << "P5\n5 5\n255\n";
  for (int i = 0; i < 25; ++i) {
    unsigned char v = 0;
    pgm.write(reinterpret_cast<char*>(&v), 1);
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
  auto map = ncp::Costmap::from_grid(grid, p);
  ncp::CoverageRequest creq;
  creq.start = {0.25, 0.25};
  creq.overlap_m = 0.25;
  auto r = ncp::plan_coverage(map, creq);
  EXPECT_EQ(r.reason, "invalid_request");
}
