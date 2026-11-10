// SPDX-License-Identifier: Apache-2.0
#include "ncp/rng.hpp"

#include <cstddef>

namespace ncp {

XorShift32::XorShift32(uint32_t seed) {
  state_ = seed == 0 ? 1u : seed;
}

uint32_t XorShift32::next() {
  uint32_t x = state_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  state_ = x;
  return x;
}

double XorShift32::uniform01() {
  return static_cast<double>(next()) / static_cast<double>(UINT32_MAX);
}

std::vector<uint32_t> xorshift32_reference_vector(uint32_t seed, int count) {
  XorShift32 rng(seed);
  std::vector<uint32_t> out;
  out.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) out.push_back(rng.next());
  return out;
}

}  // namespace ncp
