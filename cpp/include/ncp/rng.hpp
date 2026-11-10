// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <vector>

namespace ncp {

class XorShift32 {
 public:
  explicit XorShift32(uint32_t seed = 1);
  uint32_t next();
  double uniform01();

 private:
  uint32_t state_;
};

std::vector<uint32_t> xorshift32_reference_vector(uint32_t seed, int count);

}  // namespace ncp
