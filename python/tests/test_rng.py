# SPDX-License-Identifier: Apache-2.0
import ncp_core

XORSHIFT32_GOLDEN_SEED = 42
XORSHIFT32_GOLDEN = [
    11355432,
    2836018348,
    476557059,
    3648046016,
    3759983556,
    1441438134,
    3713466840,
    2431644334,
    3120216979,
    1067267639,
    2989017562,
    3379058227,
    3060050788,
    1108305632,
    1958019241,
    1084823001,
]


def test_xorshift32_matches_cpp_reference() -> None:
    a = ncp_core.xorshift32_reference_vector(XORSHIFT32_GOLDEN_SEED, len(XORSHIFT32_GOLDEN))
    assert a == XORSHIFT32_GOLDEN
