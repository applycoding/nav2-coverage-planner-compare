// SPDX-License-Identifier: Apache-2.0
#include "ncp/costmap.hpp"
#include "ncp/coverage/coverage_planner.hpp"
#include "ncp/grid_map.hpp"
#include "ncp/planners/global_planner.hpp"
#include "ncp/rng.hpp"
#include "ncp/types.hpp"
#include "ncp/validators.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(ncp_core, m) {
  m.doc() = "ncp core bindings";

  py::class_<ncp::WorldPoint>(m, "WorldPoint")
      .def(py::init<>())
      .def_readwrite("x", &ncp::WorldPoint::x)
      .def_readwrite("y", &ncp::WorldPoint::y);

  py::enum_<ncp::PlannerId>(m, "PlannerId")
      .value("AStar", ncp::PlannerId::AStar)
      .value("Dijkstra", ncp::PlannerId::Dijkstra)
      .value("RrtStar", ncp::PlannerId::RrtStar)
      .value("LazyThetaStar", ncp::PlannerId::LazyThetaStar);

  py::class_<ncp::CostmapParams>(m, "CostmapParams")
      .def(py::init<>())
      .def_readwrite("inflation_radius_m", &ncp::CostmapParams::inflation_radius_m)
      .def_readwrite("robot_radius_m", &ncp::CostmapParams::robot_radius_m)
      .def_readwrite("extra_margin_m", &ncp::CostmapParams::extra_margin_m)
      .def_readwrite("w_cost", &ncp::CostmapParams::w_cost)
      .def_readwrite("cost_decay", &ncp::CostmapParams::cost_decay)
      .def_readwrite("unknown_policy", &ncp::CostmapParams::unknown_policy);

  py::class_<ncp::PlanRequest>(m, "PlanRequest")
      .def(py::init<>())
      .def_readwrite("planner", &ncp::PlanRequest::planner)
      .def_readwrite("start", &ncp::PlanRequest::start)
      .def_readwrite("goal", &ncp::PlanRequest::goal)
      .def_readwrite("rrt_seed", &ncp::PlanRequest::rrt_seed)
      .def_readwrite("rrt_max_iterations", &ncp::PlanRequest::rrt_max_iterations)
      .def_readwrite("rrt_max_time_ms", &ncp::PlanRequest::rrt_max_time_ms);

  py::enum_<ncp::UnknownPolicy>(m, "UnknownPolicy")
      .value("Lethal", ncp::UnknownPolicy::Lethal)
      .value("Free", ncp::UnknownPolicy::Free);

  py::class_<ncp::CoverageRequest>(m, "CoverageRequest")
      .def(py::init<>())
      .def_readwrite("start", &ncp::CoverageRequest::start)
      .def_readwrite("linker", &ncp::CoverageRequest::linker)
      .def_readwrite("overlap_m", &ncp::CoverageRequest::overlap_m)
      .def_readwrite("coverage_success_threshold",
                      &ncp::CoverageRequest::coverage_success_threshold);

  py::class_<ncp::Metrics>(m, "Metrics")
      .def_readwrite("length_m", &ncp::Metrics::length_m)
      .def_readwrite("objective_cost", &ncp::Metrics::objective_cost)
      .def_readwrite("compute_ms", &ncp::Metrics::compute_ms)
      .def_readwrite("clearance_m", &ncp::Metrics::clearance_m)
      .def_readwrite("coverage_ratio_reachable", &ncp::Metrics::coverage_ratio_reachable)
      .def_readwrite("coverage_ratio_total", &ncp::Metrics::coverage_ratio_total)
      .def_readwrite("success", &ncp::Metrics::success);

  py::class_<ncp::PlanResult>(m, "PlanResult")
      .def_readwrite("success", &ncp::PlanResult::success)
      .def_readwrite("reason", &ncp::PlanResult::reason)
      .def_readwrite("path", &ncp::PlanResult::path)
      .def_readwrite("metrics", &ncp::PlanResult::metrics)
      .def_readwrite("rrt_vertices", &ncp::PlanResult::rrt_vertices);

  py::class_<ncp::GridMap>(m, "GridMap");
  py::class_<ncp::Costmap>(m, "Costmap");

  m.def("load_grid", [](const std::string& pgm, const std::string& yaml) {
    std::string err;
    auto grid = ncp::GridMap::load_pgm_yaml(pgm, yaml, err);
    if (!err.empty() && grid.width() <= 0) throw std::runtime_error(err);
    return grid;
  });

  m.def("make_costmap", [](const ncp::GridMap& g, const ncp::CostmapParams& p) {
    return ncp::Costmap::from_grid(g, p);
  });

  m.def("load_costmap",
        [](const std::string& pgm, const std::string& yaml, const ncp::CostmapParams& p) {
          std::string err;
          auto grid = ncp::GridMap::load_pgm_yaml(pgm, yaml, err);
          if (!err.empty() && grid.width() <= 0) throw std::runtime_error(err);
          return ncp::Costmap::from_grid(grid, p);
        });

  m.def("plan_global", &ncp::plan_global);
  m.def("plan_coverage", &ncp::plan_coverage);

  m.def("xorshift32_reference_vector", &ncp::xorshift32_reference_vector);

  m.def(
      "validate_path",
      [](const ncp::Costmap& map, const std::vector<ncp::WorldPoint>& path,
         const ncp::WorldPoint& start, const ncp::WorldPoint& goal, double max_length_m) {
        auto vr = ncp::validate_path(map, path, start, goal, max_length_m);
        py::dict d;
        d["ok"] = vr.ok;
        d["message"] = vr.message;
        return d;
      });
}
