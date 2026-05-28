# mini_stereo_vo_ros2

ROS 2 Jazzy wrapper for [mini_stereo_vo](https://github.com/rdgldkthu/mini_stereo_vo) — runs stereo VO on a TurtleBot4 in Gazebo Harmonic, drives configurable waypoints, and evaluates trajectory accuracy with evo APE.

![System Architecture](docs/images/system_architecture.png)

---

## Prerequisites

```bash
sudo apt install \
  ros-jazzy-turtlebot4-gz-bringup \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-ros

python3 -m venv .venv && .venv/bin/pip install evo
```

---

## Build

```bash
git clone --recurse-submodules <repo-url>
cd mini_stereo_vo_ros2
source /opt/ros/jazzy/setup.bash
make build
source install/setup.bash
```

---

## Usage

```bash
make sim    # Gazebo + stereo_vo_node + RViz2
```

Once simulation is up (TurtleBot4 spawns after ~30 s):

```bash
# Drive a 1m × 1m square using VO odometry feedback
python3 scripts/run_waypoints.py

# Record a bag
make bag

# Evaluate with evo APE → results/
make eval
```

![RViz2 — run_waypoints](docs/images/rviz_run_waypoints.png)

---

## Key parameters (`config/stereo_vo_params.yaml`)

| Parameter | Default | Notes |
|-----------|---------|-------|
| `baseline_m` | `0.075` | Override when Gazebo `P[3]` is near zero |
| `pose_smooth_alpha` | `0.4` | EMA on published position; lower = smoother |
| `reinit_on_failure` | `true` | Re-bootstrap after tracking loss |

---

## Makefile

| Target | Action |
|--------|--------|
| `make build` | `colcon build --symlink-install` |
| `make sim` | Launch full simulation |
| `make bag` | Record odometry + images to `bags/<timestamp>/` |
| `make eval` | evo APE on latest bag → `results/` |
| `make clean` | Remove `build/`, `install/`, `log/` |

---

## License

MIT
