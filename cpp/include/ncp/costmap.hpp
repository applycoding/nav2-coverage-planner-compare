// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ncp/grid_map.hpp"
#include "ncp/types.hpp"

#include <cstdint>
#include <vector>

namespace ncp {

constexpr uint8_t kCostUnknown = 255;
constexpr uint8_t kCostLethal = 254;
constexpr uint8_t kCostInscribed = 253;

class Costmap {
 public:
  static Costmap from_grid(const GridMap& grid, const CostmapParams& params);

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  const MapYamlMeta& map_meta() const { return meta_; }

  Occupancy occupancy(int x, int y) const;
  uint8_t cost(int x, int y) const;
  bool is_hard_lethal(int x, int y) const;
  bool is_hard_blocked(int x, int y) const;

  WorldPoint cell_center(int x, int y) const;
  bool world_to_cell(const WorldPoint& w, int& x, int& y) const;

  const CostmapParams& params() const { return params_; }
  const GridMap& source_grid() const { return grid_; }

 private:
  Costmap() = default;
  void build_inflation_and_costs();

  GridMap grid_;
  CostmapParams params_;
  MapYamlMeta meta_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;
  std::vector<uint8_t> costs_;
  std::vector<uint8_t> hard_mask_;
};

}  // namespace ncp
