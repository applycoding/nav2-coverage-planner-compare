// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <cstddef>

namespace ncp {

constexpr size_t kXorShift32GoldenCount = 16;
constexpr uint32_t kXorShift32GoldenSeed = 42;
constexpr uint32_t kXorShift32Golden[kXorShift32GoldenCount] = {
    11355432u, 2836018348u, 476557059u, 3648046016u, 3759983556u, 1441438134u,
    3713466840u, 2431644334u, 3120216979u, 1067267639u, 2989017562u, 3379058227u,
    3060050788u, 1108305632u, 1958019241u, 1084823001u,
};

}  // namespace ncp
