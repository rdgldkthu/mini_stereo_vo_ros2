#!/usr/bin/env python3
# mini_stereo_vo_ros2 — run_waypoints.py
"""Drive a square waypoint loop using VO odometry feedback + direct /cmd_vel control.

No Nav2 / SLAM required — works with the odom→base_footprint TF produced by
stereo_vo_node. Uses a simple two-phase P-controller per waypoint:
  1. Rotate in-place until heading error < ANGLE_TOL
  2. Drive forward (with heading correction) until distance < POS_TOL
"""
from __future__ import annotations

import argparse
import math
import signal
import threading
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


# ── Waypoints: (x_m, y_m) ────────────────────────────────────────────────────
WAYPOINTS: List[Tuple[float, float]] = [
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
]

# ── Controller gains & tolerances ─────────────────────────────────────────────
POS_TOL    = 0.15   # m  — declare waypoint reached within this radius
ANGLE_TOL  = 0.08   # rad — switch from rotate→drive once heading error is small
KP_ANG     = 1.2    # angular proportional gain
KP_LIN     = 0.5    # linear proportional gain
MAX_LIN    = 0.3    # m/s
MAX_ANG    = 0.8    # rad/s


def _quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _angle_wrap(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


class WaypointDriver(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("waypoint_driver")
        self._lock = threading.Lock()
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        self._got_odom: bool = False

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10
        )

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = _quat_to_yaw(msg.pose.pose.orientation)
            self._got_odom = True

    def _pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._x, self._y, self._yaw

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def wait_for_odom(self, timeout_s: float = 30.0) -> bool:
        rate = self.create_rate(10)
        elapsed = 0.0
        while not self._got_odom and elapsed < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1
        return self._got_odom

    def go_to(self, tx: float, ty: float) -> bool:
        """Drive to (tx, ty). Returns True on success."""
        rate_hz = 20
        dt = 1.0 / rate_hz

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=dt)
            x, y, yaw = self._pose()
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)

            if dist < POS_TOL:
                self._stop()
                return True

            heading_desired = math.atan2(dy, dx)
            ang_err = _angle_wrap(heading_desired - yaw)

            cmd = Twist()
            if abs(ang_err) > ANGLE_TOL:
                # Phase 1: rotate in place
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, KP_ANG * ang_err))
            else:
                # Phase 2: drive forward with heading correction
                cmd.linear.x = max(0.0, min(MAX_LIN, KP_LIN * dist))
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, KP_ANG * ang_err))

            self._cmd_pub.publish(cmd)

        self._stop()
        return False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Drive a square waypoint loop via VO odometry + /cmd_vel."
    )
    parser.add_argument(
        "--odom-topic",
        default="/vo/odometry",
        help="Odometry topic to use for pose feedback (default: /vo/odometry)",
    )
    args = parser.parse_args()

    rclpy.init()
    driver = WaypointDriver(odom_topic=args.odom_topic)

    def _shutdown(sig, frame):
        driver._stop()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"Waiting for odometry on '{args.odom_topic}'…")
    if not driver.wait_for_odom():
        print("ERROR: no odometry received within timeout. Is stereo_vo_node running?")
        rclpy.shutdown()
        return

    print(f"Odometry received. Navigating {len(WAYPOINTS)} waypoints.")
    for i, (x, y) in enumerate(WAYPOINTS):
        print(f"  [{i + 1}/{len(WAYPOINTS)}] → ({x:.2f}, {y:.2f})")
        ok = driver.go_to(x, y)
        status = "reached" if ok else "FAILED"
        print(f"         {status}")

    print("Waypoint loop complete.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
