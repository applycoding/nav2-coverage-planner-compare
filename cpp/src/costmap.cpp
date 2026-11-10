// SPDX-License-Identifier: Apache-2.0
#include "ncp/costmap.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace ncp {

Occupancy Costmap::occupancy(int x, int y) const { return grid_.at(x, y); }

uint8_t Costmap::cost(int x, int y) const {
  if (!grid_.in_bounds(x, y)) return kCostUnknown;
  return costs_[static_cast<size_t>(y) * width_ + x];
}

bool Costmap::is_hard_lethal(int x, int y) const {
  if (!grid_.in_bounds(x, y)) return true;
  return hard_mask_[static_cast<size_t>(y) * width_ + x] != 0;
}

bool Costmap::is_hard_blocked(int x, int y) const {
  if (!grid_.in_bounds(x, y)) return true;
  const size_t i = static_cast<size_t>(y) * width_ + x;
  if (hard_mask_[i]) return true;
  if (params_.collide_inscribed && costs_[i] >= kCostInscribed) return true;
  return false;
}

WorldPoint Costmap::cell_center(int x, int y) const { return grid_.cell_center(x, y); }

bool Costmap::world_to_cell(const WorldPoint& w, int& x, int& y) const {
  return grid_.world_to_cell(w, x, y);
}

static bool cell_blocks_center(const GridMap& grid, const CostmapParams& p, int x, int y) {
  const Occupancy o = grid.at(x, y);
  if (o == Occupancy::Occupied) return true;
  if (o == Occupancy::Unknown) {
    return p.unknown_policy == UnknownPolicy::Lethal;
  }
  return false;
}

void Costmap::build_inflation_and_costs() {
  const int n = width_ * height_;
  costs_.assign(n, 0);
  hard_mask_.assign(n, 0);

  const double res = resolution_;
  const int inflate_cells =
      static_cast<int>(std::ceil(params_.inflation_radius_m / res));
  std::vector<int> dist_sq(n, inflate_cells * inflate_cells + 1);

  struct Node {
    int idx;
    int d2;
  };
  std::queue<int> q;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (cell_blocks_center(grid_, params_, x, y)) {
        const int idx = y * width_ + x;
        dist_sq[idx] = 0;
        hard_mask_[idx] = 1;
        q.push(idx);
      }
    }
  }

  static const int dx[] = {-1, 0, 1, 0};
  static const int dy[] = {0, 1, 0, -1};
  while (!q.empty()) {
    const int idx = q.front();
    q.pop();
    const int x0 = idx % width_;
    const int y0 = idx / width_;
  for (int k = 0; k < 4; ++k) {
      const int x = x0 + dx[k];
      const int y = y0 + dy[k];
      if (!grid_.in_bounds(x, y)) continue;
      const int nidx = y * width_ + x;
      const int nd2 = dist_sq[idx] + 1;
      if (nd2 < dist_sq[nidx]) {
        dist_sq[nidx] = nd2;
        if (nd2 <= inflate_cells * inflate_cells) {
          hard_mask_[nidx] = 1;
        }
        q.push(nidx);
      }
    }
  }

  if (params_.cost_decay > 0.0) {
    const double radius = params_.inflation_radius_m;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const size_t i = static_cast<size_t>(y) * width_ + x;
        if (hard_mask_[i]) {
          costs_[i] = kCostLethal;
          continue;
        }
        const double d_m = std::sqrt(static_cast<double>(dist_sq[i])) * res;
        if (d_m <= params_.robot_radius_m) {
          costs_[i] = kCostInscribed;
        } else if (d_m < radius) {
          const double t = (radius - d_m) / std::max(radius, 1e-9);
          const uint8_t c = static_cast<uint8_t>(std::min(252.0, t * 252.0 * params_.cost_decay / 5.0));
          costs_[i] = std::max(costs_[i], c);
        }
        if (grid_.at(x, y) == Occupancy::Unknown &&
            params_.unknown_policy == UnknownPolicy::Lethal) {
          costs_[i] = kCostUnknown;
        }
      }
    }
  } else {
    for (int i = 0; i < n; ++i) {
      costs_[i] = hard_mask_[i] ? kCostLethal : 0;
    }
  }
}

Costmap Costmap::from_grid(const GridMap& grid, const CostmapParams& params) {
  Costmap cm;
  cm.grid_ = grid;
  cm.params_ = params;
  cm.width_ = grid.width();
  cm.height_ = grid.height();
  cm.resolution_ = grid.resolution();
  cm.meta_ = grid.meta();
  if (cm.width_ <= 0 || cm.height_ <= 0) return cm;
  cm.build_inflation_and_costs();
  return cm;
}

}  // namespace ncp
