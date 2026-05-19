#!/usr/bin/env python3
# mini_stereo_vo_ros2 — cmd_vel_relay.py
# Relays geometry_msgs/Twist from /cmd_vel → diffdrive_controller/cmd_vel
# so standard teleop tools work without knowing the Create3 controller topic.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_relay")
        self._pub = self.create_publisher(Twist, "diffdrive_controller/cmd_vel", 1)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._cb, 1)

    def _cb(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
