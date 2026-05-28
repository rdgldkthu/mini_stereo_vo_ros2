#!/usr/bin/env python3
# mini_stereo_vo_ros2 — gen_readme_images.py
"""Generate the system architecture diagram for the README.

Usage:
    python3 scripts/gen_readme_images.py

Output:
    docs/images/system_architecture.png
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "docs" / "images"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Palette ────────────────────────────────────────────────────────────────────
GZ_F, GZ_E = "#D6EAF8", "#1565C0"   # blue — Gazebo
BR_F, BR_E = "#FFF8E1", "#E65100"   # amber — bridge
RO_F, RO_E = "#E8F5E9", "#2E7D32"   # green — ROS nodes
OU_F, OU_E = "#F3E5F5", "#6A1B9A"   # purple — eval / output
GR_F, GR_E = "#ECEFF1", "#455A64"   # gray — topics / misc
BG        = "#FAFBFC"
TEXT      = "#212121"
GRAY      = "#546E7A"


# ── Primitives ────────────────────────────────────────────────────────────────

def _box(ax, cx: float, cy: float, w: float, h: float,
         title: str, sub: str = "",
         fc: str = GR_F, ec: str = GR_E,
         ts: float = 9.5, ss: float = 7.5) -> None:
    """Rounded box centred at (cx, cy)."""
    ax.add_patch(FancyBboxPatch(
        (cx - w / 2, cy - h / 2), w, h,
        boxstyle="round,pad=0.04",
        linewidth=1.8, edgecolor=ec, facecolor=fc, zorder=3,
    ))
    if sub:
        ax.text(cx, cy + h * 0.19, title,
                ha="center", va="center",
                fontsize=ts, fontweight="bold", color=TEXT, zorder=4)
        ax.text(cx, cy - h * 0.20, sub,
                ha="center", va="center",
                fontsize=ss, color=GRAY, zorder=4, linespacing=1.4)
    else:
        ax.text(cx, cy, title,
                ha="center", va="center",
                fontsize=ts, fontweight="bold", color=TEXT, zorder=4)


def _arr(ax, x0: float, y0: float, x1: float, y1: float,
         label: str = "", lc: str = GRAY, lfs: float = 7.0,
         rad: float = 0.0,
         lt: float = 0.5, ldx: float = 0.0, ldy: float = 0.0) -> None:
    """Arrow from (x0,y0) to (x1,y1) with optional label.

    lt   – fractional position along the path (0=tail, 1=head)
    ldx/ldy – additional x/y offset applied to the label position
    """
    ax.annotate(
        "", xy=(x1, y1), xytext=(x0, y0),
        arrowprops=dict(
            arrowstyle="-|>", color=lc, lw=1.3, mutation_scale=11,
            connectionstyle=f"arc3,rad={rad}",
        ),
        zorder=5,
    )
    if label:
        mx = x0 + lt * (x1 - x0) + ldx
        my = y0 + lt * (y1 - y0) + ldy
        ax.text(mx, my, label,
                ha="center", va="center",
                fontsize=lfs, color=lc, zorder=6,
                bbox=dict(facecolor=BG, edgecolor="none", alpha=0.92, pad=1.5))


def _new_ax(fw: float, fh: float):
    fig, ax = plt.subplots(figsize=(fw, fh))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, fw)
    ax.set_ylim(0, fh)
    ax.axis("off")
    return fig, ax


# ── Diagram 1: System Architecture ────────────────────────────────────────────
#
# Layout  (FW=17, FH=9)
#
#   Row 1 y=7.0 :  [Gazebo]  →  [Bridge]  →  [stereo_vo_node]
#   Row 2 y=4.6 :                            [/vo/* topics + TF]
#   Row 3 y=2.3 :  [Nav2]                    [evo evaluation]
#
# Box geometry (cx, cy, w, h) — no pair overlaps:
#   Gazebo      : ( 2.2, 7.0, 3.8, 1.6)   x=[0.3 , 4.1], y=[6.2, 7.8]
#   Bridge      : ( 6.2, 7.0, 1.8, 1.6)   x=[5.3 , 7.1], y=[6.2, 7.8]  gap_x=1.2
#   stereo_vo   : (11.5, 7.0, 4.4, 1.6)   x=[9.3 ,13.7], y=[6.2, 7.8]  gap_x=2.2
#   vo_topics   : (11.5, 4.6, 4.4, 2.2)   x=[9.3 ,13.7], y=[3.5, 5.7]  gap_y=0.5
#   nav2        : ( 3.8, 2.3, 4.6, 1.6)   x=[1.5 , 6.1], y=[1.5, 3.1]
#   evo         : (11.5, 2.3, 4.4, 1.6)   x=[9.3 ,13.7], y=[1.5, 3.1]  gap_y=0.4

def gen_architecture() -> None:
    FW, FH = 17, 9.5
    fig, ax = _new_ax(FW, FH)

    # ── Boxes ─────────────────────────────────────────────────────────────────
    # All cx values shifted +1.5 vs the natural layout to centre the content
    # in the 17-unit-wide figure (content spans x≈1.8–15.2, centre≈8.5 ✓).
    _box(ax, 3.7, 7.0, 3.8, 1.6,
         "Gazebo Harmonic",
         "TurtleBot4 (standard)\nOAK-D stereo · warehouse world",
         GZ_F, GZ_E)

    _box(ax, 7.7, 7.0, 1.8, 1.6,
         "ros_gz\nbridge",
         "",
         BR_F, BR_E, ts=9.0)

    _box(ax, 13.0, 7.0, 4.4, 1.6,
         "stereo_vo_node",
         "StereoInit · KLT Tracker\nPnP RANSAC · Pose Refine",
         RO_F, RO_E)

    _box(ax, 13.0, 4.6, 4.4, 2.2,
         "Published Topics",
         "/vo/odometry   (nav_msgs/Odometry)\n"
         "/vo/pose          (geometry_msgs/PoseStamped)\n"
         "/vo/path           (nav_msgs/Path)\n"
         "TF: odom → base_footprint",
         GR_F, GR_E, ts=8.5, ss=7.5)

    _box(ax, 5.3, 2.3, 4.6, 1.6,
         "run_waypoints.py",
         "VO odom feedback · P-controller\n"
         "/cmd_vel out",
         RO_F, RO_E)

    _box(ax, 13.0, 2.3, 4.4, 1.6,
         "bag2tum.py + evo APE",
         "ROS 2 bag → TUM format\nevo_ape · absolute pose error",
         OU_F, OU_E)

    # ── Arrows ────────────────────────────────────────────────────────────────
    # (1) Gazebo → Bridge  [images + camera_info]
    # Label floated above the row so it clears the box tops (top = 7.8).
    _arr(ax, 5.6, 7.0, 6.8, 7.0,
         "/left|right/image_raw   /left|right/camera_info",
         GZ_E, 6.8, lt=0.45, ldy=1.2)

    # (2) Bridge → stereo_vo_node  [label above, centred in the wider gap]
    _arr(ax, 8.6, 7.0, 10.8, 7.0, "ROS 2 topics", GZ_E, 7.0, ldy=0.55)

    # (3) stereo_vo_node bottom → vo_topics top
    _arr(ax, 13.0, 6.2, 13.0, 5.7, "", GRAY)

    # (4) vo_topics bottom → evo top  [label to the right to avoid evo edge]
    _arr(ax, 13.0, 3.5, 13.0, 3.1, "via bag", GRAY, 6.8, ldx=0.9)

    # (5) run_waypoints.py top → Bridge bottom  [/cmd_vel up to bridge]
    _arr(ax, 5.3, 3.1, 7.3, 6.2, "/cmd_vel", BR_E, 6.8, rad=-0.15)

    # (6) Published Topics bottom-left → run_waypoints.py right  [/vo/odometry feedback]
    _arr(ax, 10.8, 3.5, 7.6, 2.6, "/vo/odometry", OU_E, 7.0, lt=0.5, ldy=-0.3)

    # (7) Bridge left → Gazebo right  [/cmd_vel forwarded to DiffDrive]
    _arr(ax, 6.8, 6.7, 5.6, 6.7, "", BR_E)

    # ── Title ─────────────────────────────────────────────────────────────────
    ax.set_title("mini_stereo_vo_ros2 — System Architecture",
                 fontsize=13, fontweight="bold", color=TEXT, pad=10)

    out = OUT_DIR / "system_architecture.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"Saved {out}")


if __name__ == "__main__":
    gen_architecture()
    print("Done.")
