// SPDX-License-Identifier: Apache-2.0
#include "ncp/planners/graph_search.hpp"

#include "ncp/collision.hpp"
#include "ncp/metrics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace ncp {

static double octile_heuristic(int dx, int dy, double res, double min_cost_per_m) {
  const int adx = std::abs(dx);
  const int ady = std::abs(dy);
  const int m = std::min(adx, ady);
  const int M = std::max(adx, ady);
  return min_cost_per_m * res * (M - m + m * std::sqrt(2.0));
}

static bool diagonal_legal(const Costmap& map, int x, int y, int nx, int ny) {
  const int ox1 = nx;
  const int oy1 = y;
  const int ox2 = x;
  const int oy2 = ny;
  if (map.is_hard_blocked(ox1, oy1)) return false;
  if (map.is_hard_blocked(ox2, oy2)) return false;
  return true;
}

static double step_cost(const Costmap& map, int x0, int y0, int x1, int y1) {
  const double res = map.resolution();
  const bool diag = x0 != x1 && y0 != y1;
  const double len = diag ? res * std::sqrt(2.0) : res;
  const uint8_t c0 = map.cost(x0, y0);
  const uint8_t c1 = map.cost(x1, y1);
  if (c0 >= kCostInscribed || c1 >= kCostInscribed) return std::numeric_limits<double>::infinity();
  const uint8_t cmax = std::max(c0, c1);
  const double wc = map.params().w_cost;
  return len * (1.0 + wc * static_cast<double>(cmax) / 252.0);
}

PlanResult plan_graph_search(const Costmap& map, const PlanRequest& req, bool use_astar) {
  PlanResult result;
  const auto t0 = std::chrono::steady_clock::now();

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!map.world_to_cell(req.start, sx, sy)) {
    result.reason = "out_of_bounds";
    result.metrics.reason = result.reason;
    return result;
  }
  if (!map.world_to_cell(req.goal, gx, gy)) {
    result.reason = "out_of_bounds";
    result.metrics.reason = result.reason;
    return result;
  }
  if (map.is_hard_blocked(sx, sy)) {
    result.reason = "start_in_obstacle";
    return result;
  }
  if (map.is_hard_blocked(gx, gy)) {
    result.reason = "goal_in_obstacle";
    return result;
  }
  if (sx == gx && sy == gy) {
    result.success = true;
    result.path = {req.start};
    result.metrics.length_m = 0.0;
    result.metrics.objective_cost = 0.0;
    result.metrics.success = true;
    const auto t1 = std::chrono::steady_clock::now();
    result.metrics.compute_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
  }

  const int w = map.width();
  const int h = map.height();
  const int n = w * h;
  const int start = sy * w + sx;
  const int goal = gy * w + gx;

  std::vector<double> gscore(n, std::numeric_limits<double>::infinity());
  std::vector<int> parent(n, -1);
  std::vector<char> closed(n, 0);
  gscore[start] = 0.0;

  const double min_cost_per_m = 1.0;
  const double res = map.resolution();

  struct HeapNode {
    double f;
    int idx;
    bool operator>(const HeapNode& o) const {
      if (f != o.f) return f > o.f;
      return idx > o.idx;
    }
  };
  std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>> open;

  const auto heuristic = [&](int idx) {
    if (!use_astar) return 0.0;
    const int x = idx % w;
    const int y = idx / w;
    return octile_heuristic(gx - x, gy - y, res, min_cost_per_m);
  };

  open.push({gscore[start] + heuristic(start), start});

  static const int dirs[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                 {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    const int cur = open.top().idx;
    open.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;
    if (cur == goal) break;

    const int cx = cur % w;
    const int cy = cur / w;
    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (map.is_hard_blocked(nx, ny)) continue;
      const bool diag = d[0] != 0 && d[1] != 0;
      if (diag && !diagonal_legal(map, cx, cy, nx, ny)) continue;
      const double ec = step_cost(map, cx, cy, nx, ny);
      if (!std::isfinite(ec)) continue;
      const int nidx = ny * w + nx;
      const double tentative = gscore[cur] + ec;
      if (tentative < gscore[nidx]) {
        gscore[nidx] = tentative;
        parent[nidx] = cur;
        open.push({tentative + heuristic(nidx), nidx});
      }
    }
  }

  if (!std::isfinite(gscore[goal])) {
    result.reason = "unreachable";
    const auto t1 = std::chrono::steady_clock::now();
    result.metrics.compute_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
  }

  std::vector<WorldPoint> path;
  int cur = goal;
  while (cur >= 0) {
    path.push_back(map.cell_center(cur % w, cur / w));
    cur = parent[cur];
  }
  std::reverse(path.begin(), path.end());
  path.front() = req.start;
  path.back() = req.goal;

  result.success = true;
  result.path = path;
  result.metrics.length_m = path_length_m(path);
  result.metrics.objective_cost = gscore[goal];
  result.metrics.success = true;
  const auto t1 = std::chrono::steady_clock::now();
  result.metrics.compute_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace ncp
