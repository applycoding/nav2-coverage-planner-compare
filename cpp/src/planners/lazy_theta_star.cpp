// SPDX-License-Identifier: Apache-2.0
// Lazy Theta*: deferred line-of-sight (Nash, Koenig, Likhachev, AAAI 2010).
#include "ncp/planners/lazy_theta_star.hpp"

#include "ncp/collision.hpp"
#include "ncp/metrics.hpp"
#include "ncp/planners/graph_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace ncp {

static double segment_traversal_cost(const Costmap& map, const WorldPoint& a,
                                     const WorldPoint& b) {
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  const double wc = map.params().w_cost;
  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  map.world_to_cell(a, x0, y0);
  map.world_to_cell(b, x1, y1);
  const uint8_t cmax = std::max(map.cost(x0, y0), map.cost(x1, y1));
  if (cmax >= kCostInscribed) return std::numeric_limits<double>::infinity();
  return len * (1.0 + wc * static_cast<double>(cmax) / 252.0);
}

PlanResult plan_lazy_theta_star(const Costmap& map, const PlanRequest& req) {
  PlanResult result;
  const auto t0 = std::chrono::steady_clock::now();

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!map.world_to_cell(req.start, sx, sy)) {
    result.reason = "out_of_bounds";
    return result;
  }
  if (!map.world_to_cell(req.goal, gx, gy)) {
    result.reason = "out_of_bounds";
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
  parent[start] = start;

  const double res = map.resolution();
  const double min_cost_per_m = 1.0;

  auto cell_point = [&](int idx) {
    return map.cell_center(idx % w, idx / w);
  };

  auto heuristic = [&](int idx) {
    const int x = idx % w;
    const int y = idx / w;
    const int adx = std::abs(gx - x);
    const int ady = std::abs(gy - y);
    const int m = std::min(adx, ady);
    const int M = std::max(adx, ady);
    return min_cost_per_m * res * (M - m + m * std::sqrt(2.0));
  };

  struct HeapNode {
    double f;
    int idx;
    bool operator>(const HeapNode& o) const {
      if (f != o.f) return f > o.f;
      return idx > o.idx;
    }
  };
  std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>> open;
  open.push({heuristic(start), start});

  static const int dirs[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                 {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    const int s = open.top().idx;
    open.pop();
    if (closed[s]) continue;
    closed[s] = 1;

    int par = parent[s];
    if (s != start) {
      int ppar = parent[par];
      if (line_of_sight(map, cell_point(ppar), cell_point(s))) {
        const double ng =
            gscore[ppar] + segment_traversal_cost(map, cell_point(ppar), cell_point(s));
        if (ng < gscore[s]) {
          gscore[s] = ng;
          parent[s] = ppar;
        }
      }
    }

    if (s == goal) break;

    const int cx = s % w;
    const int cy = s / w;
    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (map.is_hard_blocked(nx, ny)) continue;
      const int nidx = ny * w + nx;
      double nc = gscore[s] + segment_traversal_cost(map, cell_point(s), cell_point(nidx));
      if (line_of_sight(map, cell_point(parent[s]), cell_point(nidx))) {
        const double np =
            gscore[parent[s]] +
            segment_traversal_cost(map, cell_point(parent[s]), cell_point(nidx));
        nc = std::min(nc, np);
        if (nc < gscore[nidx]) parent[nidx] = parent[s];
      }
      if (nc < gscore[nidx]) {
        gscore[nidx] = nc;
        if (parent[nidx] != parent[s]) parent[nidx] = s;
        open.push({nc + heuristic(nidx), nidx});
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
  while (cur != parent[cur]) {
    path.push_back(cell_point(cur));
    cur = parent[cur];
  }
  path.push_back(cell_point(start));
  std::reverse(path.begin(), path.end());
  path.front() = req.start;
  path.back() = req.goal;

  result.success = true;
  result.path = path;
  result.metrics.length_m = path_length_m(path);
  result.metrics.objective_cost = path_objective_cost(map, path);
  result.metrics.success = true;
  const auto t1 = std::chrono::steady_clock::now();
  result.metrics.compute_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace ncp
