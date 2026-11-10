// SPDX-License-Identifier: Apache-2.0
#include "ncp/rng.hpp"
#include "ncp/rng_golden.hpp"

#include <gtest/gtest.h>

TEST(Rng, XorShift32Reference) {
  auto v = ncp::xorshift32_reference_vector(ncp::kXorShift32GoldenSeed, ncp::kXorShift32GoldenCount);
  ASSERT_EQ(v.size(), ncp::kXorShift32GoldenCount);
  for (size_t i = 0; i < ncp::kXorShift32GoldenCount; ++i) {
    EXPECT_EQ(v[i], ncp::kXorShift32Golden[i]);
  }
}
