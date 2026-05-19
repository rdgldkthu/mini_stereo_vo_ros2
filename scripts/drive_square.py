#!/usr/bin/env python3
# mini_stereo_vo_ros2 — drive_square.py
"""Drive the robot in a square by publishing /cmd_vel directly (no Nav2 needed)."""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def drive(node: Node, pub, linear: float, angular: float, duration: float) -> None:
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    end = time.time() + duration
    while time.time() < end:
        pub.publish(msg)
        time.sleep(0.1)


def stop(pub) -> None:
    pub.publish(Twist())
    time.sleep(0.2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive robot in a square via /cmd_vel.")
    parser.add_argument("--side", type=float, default=1.5, help="Side length in metres (default 1.5)")
    parser.add_argument("--speed", type=float, default=0.3, help="Linear speed m/s (default 0.3)")
    parser.add_argument("--turn-speed", type=float, default=0.5, help="Angular speed rad/s (default 0.5)")
    parser.add_argument("--loops", type=int, default=1, help="Number of square loops (default 1)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("drive_square")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    straight_t = args.side / args.speed
    turn_t = (math.pi / 2.0) / args.turn_speed

    print(f"Driving {args.loops} square loop(s): side={args.side}m, speed={args.speed}m/s")
    print("Publishing to /cmd_vel — Ctrl-C to stop early.\n")

    # Brief pause so subscribers connect
    time.sleep(0.5)

    try:
        for loop in range(args.loops):
            print(f"Loop {loop + 1}/{args.loops}")
            for side in range(4):
                print(f"  Side {side + 1}/4: straight {straight_t:.1f}s")
                drive(node, pub, args.speed, 0.0, straight_t)
                stop(pub)
                print(f"  Turn {side + 1}/4: left turn {turn_t:.1f}s")
                drive(node, pub, 0.0, args.turn_speed, turn_t)
                stop(pub)
    except KeyboardInterrupt:
        pass
    finally:
        stop(pub)
        print("Done.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
