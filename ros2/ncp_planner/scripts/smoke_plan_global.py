#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Smoke: publish /map from open fixture, call plan_global, assert path."""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ncp_planner.srv import PlanGlobal


def load_open_grid() -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.1
    grid.info.width = 20
    grid.info.height = 20
    grid.info.origin.position.x = 0.0
    grid.info.origin.position.y = 0.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (20 * 20)
    return grid


class Smoke(Node):
    def __init__(self) -> None:
        super().__init__("ncp_smoke")
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, "/map", qos)
        self.cli = self.create_client(PlanGlobal, "plan_global")

    def run(self) -> int:
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("plan_global not available")
            return 1
        self.pub.publish(load_open_grid())
        time.sleep(0.5)
        req = PlanGlobal.Request()
        req.start = PoseStamped()
        req.start.header.frame_id = "map"
        req.start.pose.orientation.w = 1.0
        req.start.pose.position.x = 0.5
        req.start.pose.position.y = 0.5
        req.goal = PoseStamped()
        req.goal.header.frame_id = "map"
        req.goal.pose.orientation.w = 1.0
        req.goal.pose.position.x = 1.5
        req.goal.pose.position.y = 1.0
        req.planner_id = "astar"
        req.rrt_seed = 1
        req.rrt_max_iterations = 1000
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done():
            self.get_logger().error("service call timed out")
            return 1
        res = fut.result()
        if not res.success or not res.path.poses:
            self.get_logger().error(f"plan failed: {res.reason}")
            return 1
        self.get_logger().info(f"smoke ok, length={res.length_m}")
        return 0


def main() -> None:
    rclpy.init()
    node = Smoke()
    code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
