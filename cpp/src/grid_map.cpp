// SPDX-License-Identifier: Apache-2.0
#include "ncp/grid_map.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ncp {

Occupancy GridMap::at(int x, int y) const {
  if (!in_bounds(x, y)) return Occupancy::Unknown;
  return occ_[static_cast<size_t>(y) * width_ + x];
}

bool GridMap::in_bounds(int x, int y) const {
  return x >= 0 && y >= 0 && x < width_ && y < height_;
}

WorldPoint GridMap::cell_center(int x, int y) const { return cell_to_world(x, y); }

WorldPoint GridMap::cell_to_world(int x, int y) const {
  const double res = meta_.resolution;
  const double cx = meta_.origin_x + (static_cast<double>(x) + 0.5) * res;
  const double cy = meta_.origin_y + (static_cast<double>(y) + 0.5) * res;
  const double c = std::cos(meta_.origin_yaw);
  const double s = std::sin(meta_.origin_yaw);
  WorldPoint w;
  w.x = c * cx - s * cy;
  w.y = s * cx + c * cy;
  return w;
}

bool GridMap::world_to_cell(const WorldPoint& w, int& x, int& y) const {
  const double c = std::cos(meta_.origin_yaw);
  const double s = std::sin(meta_.origin_yaw);
  const double lx = c * w.x + s * w.y;
  const double ly = -s * w.x + c * w.y;
  const double res = meta_.resolution;
  const double fx = (lx - meta_.origin_x) / res - 0.5;
  const double fy = (ly - meta_.origin_y) / res - 0.5;
  x = static_cast<int>(std::floor(fx + 0.5));
  y = static_cast<int>(std::floor(fy + 0.5));
  return in_bounds(x, y);
}

static bool parse_pgm(const std::string& path, std::vector<int>& pixels, int& w, int& h,
                      std::string& error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "cannot open pgm";
    return false;
  }
  std::string magic;
  in >> magic;
  if (magic != "P5") {
    error = "unsupported pgm (need P5)";
    return false;
  }
  auto skip_ws_and_comments = [&in]() {
    for (;;) {
      const int ch = in.peek();
      if (ch == std::char_traits<char>::eof()) return;
      if (ch == '#') {
        std::string line;
        std::getline(in, line);
        continue;
      }
      if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
        in.get();
        continue;
      }
      break;
    }
  };
  skip_ws_and_comments();
  in >> w >> h;
  skip_ws_and_comments();
  int maxval = 0;
  in >> maxval;
  if (maxval != 255) {
    error = "unsupported maxval";
    return false;
  }
  in.get();
  pixels.resize(static_cast<size_t>(w * h));
  for (int i = 0; i < w * h; ++i) {
    unsigned char v = 0;
    in.read(reinterpret_cast<char*>(&v), 1);
    pixels[static_cast<size_t>(i)] = v;
  }
  if (!in) {
    error = "truncated pgm";
    return false;
  }
  return true;
}

GridMap GridMap::load_pgm_yaml(const std::string& pgm_path, const std::string& yaml_path,
                               std::string& error) {
  GridMap map;
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["image"] || !root["resolution"] || !root["origin"]) {
      error = "invalid_request";
      return map;
    }
    map.meta_.resolution = root["resolution"].as<double>();
    auto origin = root["origin"].as<std::vector<double>>();
    if (origin.size() < 3) {
      error = "invalid_request";
      return map;
    }
    map.meta_.origin_x = origin[0];
    map.meta_.origin_y = origin[1];
    map.meta_.origin_yaw = origin[2];
    map.meta_.negate = root["negate"] ? root["negate"].as<int>() != 0 : false;
    map.meta_.occupied_thresh =
        root["occupied_thresh"] ? root["occupied_thresh"].as<double>() : 0.65;
    map.meta_.free_thresh = root["free_thresh"] ? root["free_thresh"].as<double>() : 0.196;

    std::vector<int> pixels;
    if (!parse_pgm(pgm_path, pixels, map.width_, map.height_, error)) {
      if (error.empty()) error = "invalid_request";
      return map;
    }
    if (map.width_ <= 0 || map.height_ <= 0) {
      error = "empty_map";
      return map;
    }
    map.occ_.resize(pixels.size());
    for (size_t i = 0; i < pixels.size(); ++i) {
      int v = pixels[i];
      if (map.meta_.negate) v = 255 - v;
      double occ_prob = v / 255.0;
      if (occ_prob >= map.meta_.occupied_thresh) {
        map.occ_[i] = Occupancy::Occupied;
      } else if (occ_prob <= map.meta_.free_thresh) {
        map.occ_[i] = Occupancy::Free;
      } else {
        map.occ_[i] = Occupancy::Unknown;
      }
    }
    error.clear();
  } catch (...) {
    error = "invalid_request";
  }
  return map;
}

GridMap GridMap::from_occupancy(int width, int height, const MapYamlMeta& meta,
                                const std::vector<Occupancy>& occ) {
  GridMap map;
  map.width_ = width;
  map.height_ = height;
  map.meta_ = meta;
  map.occ_ = occ;
  return map;
}

}  // namespace ncp
