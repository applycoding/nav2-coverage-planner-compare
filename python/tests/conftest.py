# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

import pytest

FIXTURES_ROOT = Path(__file__).resolve().parents[2] / "fixtures" / "maps"


@pytest.fixture
def fixtures_root() -> Path:
    return FIXTURES_ROOT


@pytest.fixture
def map_ids(fixtures_root: Path) -> list[str]:
    return sorted(
        p.name for p in fixtures_root.iterdir() if p.is_dir() and (p / "cases.json").exists()
    )
