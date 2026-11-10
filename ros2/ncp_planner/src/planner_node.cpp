// SPDX-License-Identifier: Apache-2.0
#include "ncp/costmap.hpp"
#include "ncp/coverage/coverage_planner.hpp"
#include "ncp/grid_map.hpp"
#include "ncp/planners/global_planner.hpp"
#include "ncp/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ncp_planner/srv/plan_global.hpp"
#include "ncp_planner/srv/plan_coverage.hpp"

#include <mutex>
#include <optional>

class NcpPlannerNode : public rclcpp::Node {
 public:
  NcpPlannerNode() : Node("ncp_planner_node") {
    declare_parameter<double>("robot_radius_m", 0.15);
    declare_parameter<double>("inflation_radius_m", 0.15);

    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_map_ = msg;
        });

    plan_global_srv_ = create_service<ncp_planner::srv::PlanGlobal>(
        "plan_global", [this](const std::shared_ptr<ncp_planner::srv::PlanGlobal::Request> req,
                              std::shared_ptr<ncp_planner::srv::PlanGlobal::Response> res) {
          handlePlanGlobal(req, res);
        });

    plan_coverage_srv_ = create_service<ncp_planner::srv::PlanCoverage>(
        "plan_coverage",
        [this](const std::shared_ptr<ncp_planner::srv::PlanCoverage::Request> req,
               std::shared_ptr<ncp_planner::srv::PlanCoverage::Response> res) {
          handlePlanCoverage(req, res);
        });

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 10);
  }

 private:
  std::optional<ncp::Costmap> snapshot_costmap() {
    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_map_) return std::nullopt;
      map = latest_map_;
    }

    ncp::MapYamlMeta meta;
    meta.resolution = map->info.resolution;
    meta.origin_x = map->info.origin.position.x;
    meta.origin_y = map->info.origin.position.y;
    meta.origin_yaw = 0.0;

    const int w = static_cast<int>(map->info.width);
    const int h = static_cast<int>(map->info.height);
    std::vector<ncp::Occupancy> occ(static_cast<size_t>(w * h), ncp::Occupancy::Free);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int8_t v = map->data[static_cast<size_t>(y) * w + x];
        ncp::Occupancy o = ncp::Occupancy::Unknown;
        if (v == 0) {
          o = ncp::Occupancy::Free;
        } else if (v > 50) {
          o = ncp::Occupancy::Occupied;
        }
        occ[static_cast<size_t>(y) * w + x] = o;
      }
    }
    auto grid = ncp::GridMap::from_occupancy(w, h, meta, occ);

    ncp::CostmapParams params;
    params.robot_radius_m = get_parameter("robot_radius_m").as_double();
    params.inflation_radius_m = get_parameter("inflation_radius_m").as_double();
    params.cost_decay = 0.0;
    return ncp::Costmap::from_grid(grid, params);
  }

  static ncp::WorldPoint poseToPoint(const geometry_msgs::msg::Pose& p) {
    return ncp::WorldPoint{p.position.x, p.position.y};
  }

  static nav_msgs::msg::Path toPathMsg(const std::vector<ncp::WorldPoint>& path,
                                       const std::string& frame) {
    nav_msgs::msg::Path out;
    out.header.frame_id = frame;
    for (const auto& wp : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = frame;
      ps.pose.position.x = wp.x;
      ps.pose.position.y = wp.y;
      ps.pose.orientation.w = 1.0;
      out.poses.push_back(ps);
    }
    return out;
  }

  void handlePlanGlobal(const std::shared_ptr<ncp_planner::srv::PlanGlobal::Request> req,
                        std::shared_ptr<ncp_planner::srv::PlanGlobal::Response> res) {
    const auto cmap = snapshot_costmap();
    if (!cmap) {
      res->success = false;
      res->reason = "no_map";
      return;
    }
    ncp::PlanRequest preq;
    preq.planner = ncp::planner_id_from_string(req->planner_id);
    preq.start = poseToPoint(req->start.pose);
    preq.goal = poseToPoint(req->goal.pose);
    preq.rrt_seed = req->rrt_seed;
    preq.rrt_max_iterations = req->rrt_max_iterations;
    const auto result = ncp::plan_global(*cmap, preq);
    res->success = result.success;
    res->reason = result.success ? "" : result.reason;
    res->length_m = result.metrics.length_m;
    res->objective_cost = result.metrics.objective_cost;
  res->path = toPathMsg(result.path, req->start.header.frame_id.empty() ? "map" : req->start.header.frame_id);
    path_pub_->publish(res->path);
  }

  void handlePlanCoverage(const std::shared_ptr<ncp_planner::srv::PlanCoverage::Request> req,
                          std::shared_ptr<ncp_planner::srv::PlanCoverage::Response> res) {
    const auto cmap = snapshot_costmap();
    if (!cmap) {
      res->success = false;
      res->reason = "no_map";
      return;
    }
    ncp::CoverageRequest creq;
    creq.start = poseToPoint(req->start.pose);
    creq.linker = ncp::planner_id_from_string(req->linker_id);
    creq.overlap_m = req->overlap_m;
    creq.coverage_success_threshold = req->coverage_success_threshold;
    const auto result = ncp::plan_coverage(*cmap, creq);
    res->success = result.success;
    res->reason = result.success ? "" : result.reason;
    res->coverage_ratio_reachable = result.metrics.coverage_ratio_reachable;
    res->length_m = result.metrics.length_m;
    res->path = toPathMsg(result.path, req->start.header.frame_id.empty() ? "map" : req->start.header.frame_id);
    path_pub_->publish(res->path);
  }

  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Service<ncp_planner::srv::PlanGlobal>::SharedPtr plan_global_srv_;
  rclcpp::Service<ncp_planner::srv::PlanCoverage>::SharedPtr plan_coverage_srv_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NcpPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
