#!/usr/bin/env python3
# mini_stereo_vo_ros2 — run_waypoints.py
"""Send a square waypoint loop to Nav2 BasicNavigator."""
from __future__ import annotations

import argparse
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# Edit this list to change the waypoint sequence.
# Each entry is (x_m, y_m, yaw_rad).
WAYPOINTS: List[Tuple[float, float, float]] = [
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 1.5708),
    (0.0, 1.0, 3.1416),
    (0.0, 0.0, -1.5708),
]


def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    import math
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


def main() -> None:
    parser = argparse.ArgumentParser(description="Send square waypoints via Nav2.")
    parser.add_argument(
        "--frame", default="map", help="Reference frame for waypoints (default: map)"
    )
    args = parser.parse_args()

    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    poses = [make_pose(x, y, yaw, args.frame) for x, y, yaw in WAYPOINTS]
    for i, (pose, (x, y, _)) in enumerate(zip(poses, WAYPOINTS)):
        pose.header.stamp = nav.get_clock().now().to_msg()
        print(f"Navigating to waypoint {i + 1}/{len(poses)}: ({x}, {y})")
        nav.goToPose(pose)
        while not nav.isTaskComplete():
            pass
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"  Reached waypoint {i + 1}.")
        else:
            print(f"  Failed to reach waypoint {i + 1}: {result}")

    print("Waypoint loop complete.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
