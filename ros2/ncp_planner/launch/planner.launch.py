# SPDX-License-Identifier: Apache-2.0
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ncp_planner",
                executable="ncp_planner_node",
                name="ncp_planner_node",
                output="screen",
                parameters=[
                    {
                        "robot_radius_m": 0.15,
                        "inflation_radius_m": 0.15,
                    }
                ],
            ),
        ]
    )
