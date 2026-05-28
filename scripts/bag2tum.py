#!/usr/bin/env python3
# mini_stereo_vo_ros2 — bag2tum.py
"""Convert a ROS2 bag to TUM trajectory files and run evo APE."""
import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import List, Tuple

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def _detect_storage_id(bag_path: str) -> str:
    import glob
    if glob.glob(os.path.join(bag_path, "*.mcap")):
        return "mcap"
    return "sqlite3"


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id=_detect_storage_id(bag_path)
    )
    convert_opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_opts, convert_opts)
    return reader


def read_odometry_tum(bag_path: str, topic: str) -> List[str]:
    """Read nav_msgs/Odometry from bag; return TUM-format lines."""
    from nav_msgs.msg import Odometry

    reader = open_reader(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic not in topic_types:
        print(f"[warn] Topic {topic} not found in bag.", file=sys.stderr)
        return []

    lines: List[str] = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg: Odometry = deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        lines.append(f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                     f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}")
    return lines


def read_pose_stamped_tum(bag_path: str, topic: str) -> List[str]:
    """Read geometry_msgs/PoseStamped from bag; return TUM-format lines."""
    from geometry_msgs.msg import PoseStamped

    reader = open_reader(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic not in topic_types:
        print(f"[warn] Topic {topic} not found in bag.", file=sys.stderr)
        return []

    lines: List[str] = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg: PoseStamped = deserialize_message(data, PoseStamped)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        lines.append(f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                     f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}")
    return lines


def write_tum(path: str, lines: List[str]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {len(lines)} poses to {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert ROS2 bag to TUM trajectory files and run evo APE."
    )
    parser.add_argument("bag_path", help="Path to the .bag directory")
    parser.add_argument(
        "--robot-name", default="turtlebot4", help="Robot name (unused, reserved)"
    )
    parser.add_argument(
        "--vo-topic", default="/vo/odometry", help="VO odometry topic"
    )
    parser.add_argument(
        "--gt-topic", default="/model/turtlebot4/odometry",
        help="Ground-truth Odometry topic (DiffDrive gz odometry, bridged from gz.msgs.Odometry)"
    )
    parser.add_argument(
        "--results-dir", default="results", help="Output directory"
    )
    args = parser.parse_args()

    vo_tum_path = os.path.join(args.results_dir, "vo_traj.txt")
    gt_tum_path = os.path.join(args.results_dir, "gt_traj.txt")

    vo_lines = read_odometry_tum(args.bag_path, args.vo_topic)
    if not vo_lines:
        sys.exit(f"No VO data found in {args.bag_path} on {args.vo_topic}")
    write_tum(vo_tum_path, vo_lines)

    gt_lines = read_odometry_tum(args.bag_path, args.gt_topic)
    if not gt_lines:
        print("[warn] No ground-truth data — skipping evo APE.", file=sys.stderr)
        return
    write_tum(gt_tum_path, gt_lines)

    save_zip = str(Path(args.results_dir) / "ape_results.zip")
    cmd = [
        "evo_ape", "tum",
        gt_tum_path, vo_tum_path,
        "-p",
        "--save_results", save_zip,
    ]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
