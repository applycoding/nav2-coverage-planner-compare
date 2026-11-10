# SPDX-License-Identifier: Apache-2.0
from typing import List

class WorldPoint:
    x: float
    y: float

class PlannerId:
    AStar: PlannerId
    Dijkstra: PlannerId
    RrtStar: PlannerId
    LazyThetaStar: PlannerId

class UnknownPolicy:
    Lethal: UnknownPolicy
    Free: UnknownPolicy

class GridMap: ...

class Costmap: ...

class CostmapParams:
    inflation_radius_m: float
    robot_radius_m: float
    extra_margin_m: float
    w_cost: float
    cost_decay: float
    unknown_policy: UnknownPolicy

class PlanRequest:
    planner: PlannerId
    start: WorldPoint
    goal: WorldPoint
    rrt_seed: int
    rrt_max_iterations: int
    rrt_max_time_ms: int

class CoverageRequest:
    start: WorldPoint
    linker: PlannerId
    overlap_m: float
    coverage_success_threshold: float

class Metrics:
    length_m: float
    objective_cost: float
    coverage_ratio_reachable: float
    coverage_ratio_total: float
    clearance_m: float
    compute_ms: int
    success: bool

class PlanResult:
    success: bool
    reason: str
    path: List[WorldPoint]
    metrics: Metrics
    rrt_vertices: int
    unreachable_free_ratio: float

def load_grid(pgm: str, yaml: str) -> GridMap: ...
def make_costmap(grid: GridMap, params: CostmapParams) -> Costmap: ...
def load_costmap(pgm: str, yaml: str, params: CostmapParams) -> Costmap: ...
def plan_global(map: Costmap, req: PlanRequest) -> PlanResult: ...
def plan_coverage(map: Costmap, req: CoverageRequest) -> PlanResult: ...
def xorshift32_reference_vector(seed: int, count: int) -> List[int]: ...
def validate_path(
    map: object,
    path: List[WorldPoint],
    start: WorldPoint,
    goal: WorldPoint,
    max_length_m: float,
) -> dict[str, object]: ...
