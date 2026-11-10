// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/types.hpp"

#include <string>
#include <vector>

namespace ncp {

struct GridIndex {
  int x = 0;
  int y = 0;
};

struct MapYamlMeta {
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  bool negate = false;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
};

class GridMap {
 public:
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return meta_.resolution; }
  const MapYamlMeta& meta() const { return meta_; }

  Occupancy at(int x, int y) const;
  bool in_bounds(int x, int y) const;

  WorldPoint cell_center(int x, int y) const;
  bool world_to_cell(const WorldPoint& w, int& x, int& y) const;
  WorldPoint cell_to_world(int x, int y) const;

  static GridMap load_pgm_yaml(const std::string& pgm_path,
                               const std::string& yaml_path,
                               std::string& error);

  static GridMap from_occupancy(int width, int height, const MapYamlMeta& meta,
                                const std::vector<Occupancy>& occ);

  const std::vector<Occupancy>& occupancy() const { return occ_; }

 private:
  int width_ = 0;
  int height_ = 0;
  MapYamlMeta meta_;
  std::vector<Occupancy> occ_;
};

}  // namespace ncp
