#!/usr/bin/env python3
# mini_stereo_vo_ros2 — gen_readme_images.py
"""Generate architecture and pipeline diagrams for the README.

Usage:
    python3 scripts/gen_readme_images.py

Output:
    docs/images/system_architecture.png
    docs/images/vo_pipeline.png
    docs/images/tf_tree.png
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
         "StereoInit · KLT Tracker\nPnP RANSAC · Local BA",
         RO_F, RO_E)

    _box(ax, 13.0, 4.6, 4.4, 2.2,
         "Published Topics",
         "/vo/odometry   (nav_msgs/Odometry)\n"
         "/vo/pose          (geometry_msgs/PoseStamped)\n"
         "/vo/path           (nav_msgs/Path)\n"
         "TF: odom → base_footprint",
         GR_F, GR_E, ts=8.5, ss=7.5)

    _box(ax, 5.3, 2.3, 4.6, 1.6,
         "Nav2 — run_waypoints.py",
         "BasicNavigator · 1 m × 1 m square\n"
         "/vo/odometry in · /cmd_vel out",
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

    # (5) vo_topics left-side → Nav2 right edge  [/vo/odometry for Nav2]
    _arr(ax, 10.8, 4.6, 7.6, 2.3, "/vo/odometry", RO_E, 6.8)

    # (6) Nav2 top → Bridge bottom  [/cmd_vel up to bridge]
    _arr(ax, 5.3, 3.1, 7.3, 6.2, "/cmd_vel", BR_E, 6.8, rad=-0.15)

    # (7) Bridge left → Gazebo right  [/cmd_vel forwarded to DiffDrive]
    _arr(ax, 6.8, 6.7, 5.6, 6.7, "", BR_E)

    # ── Title ─────────────────────────────────────────────────────────────────
    ax.set_title("mini_stereo_vo_ros2 — System Architecture",
                 fontsize=13, fontweight="bold", color=TEXT, pad=10)

    out = OUT_DIR / "system_architecture.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"Saved {out}")


# ── Diagram 2: VO Pipeline ─────────────────────────────────────────────────────
#
# 6 stages in a single horizontal row.
# FW=16, FH=5
# 6 boxes of w=2.2, h=2.8, gap=0.35 between boxes.
# total width used = 6*2.2 + 5*0.35 = 13.2+1.75 = 14.95
# left margin = (16-14.95)/2 = 0.525
# cx[i] = 0.525 + 1.1 + i*(2.2+0.35) = 1.625 + i*2.55

PIPELINE_STAGES = [
    ("Stereo\nFrame",
     "left + right\nmono8",
     GZ_F, GZ_E),
    ("Stereo\nInitializer",
     "ORB detect\nSGM match\ntriangulate ≥20",
     RO_F, RO_E),
    ("KLT\nTracker",
     "optical flow\n21×21 win\n3 pyr · bidir check",
     RO_F, RO_E),
    ("PnP\nRANSAC",
     "≥6 pts · 4 px err\n99% confidence\n100 iters",
     RO_F, RO_E),
    ("L-M Pose\nRefine",
     "Levenberg-\nMarquardt\n≥10 inliers",
     RO_F, RO_E),
    ("Keyframe\n+ Map + BA",
     "KF insert · prune\nlocal BA\nevery 2 KFs",
     RO_F, RO_E),
    ("Publish",
     "/vo/odometry\n/vo/path\nTF broadcast",
     GZ_F, GZ_E),
]


def gen_vo_pipeline() -> None:
    N = len(PIPELINE_STAGES)
    BOX_W, BOX_H, GAP = 2.0, 2.8, 0.4
    STEP = BOX_W + GAP
    TOTAL_W = N * BOX_W + (N - 1) * GAP
    FW = TOTAL_W + 2.0          # 1 inch margin each side
    FH = 5.5
    CY = FH / 2 + 0.2           # vertical centre of boxes

    fig, ax = _new_ax(FW, FH)

    cx_first = 1.0 + BOX_W / 2
    for i, (title, sub, fc, ec) in enumerate(PIPELINE_STAGES):
        cx = cx_first + i * STEP
        _box(ax, cx, CY, BOX_W, BOX_H, title, sub, fc, ec, ts=9.0, ss=7.5)
        if i > 0:
            prev_cx = cx_first + (i - 1) * STEP
            _arr(ax,
                 prev_cx + BOX_W / 2, CY,
                 cx - BOX_W / 2, CY,
                 "", GRAY)

    # Reinit feedback arrow: from bottom of KF+Map+BA box (index 5)
    # back to top of StereoInitializer box (index 1), arc below the boxes.
    init_cx  = cx_first + 1 * STEP   # StereoInitializer
    kfmap_cx = cx_first + 5 * STEP   # Keyframe+Map+BA
    y_bottom = CY - BOX_H / 2
    y_arc    = y_bottom - 0.55
    # Draw as two downward segments + horizontal + two upward, approximated with
    # a low annotation arc.
    ax.annotate(
        "",
        xy=(init_cx, y_bottom),
        xytext=(kfmap_cx, y_bottom),
        arrowprops=dict(
            arrowstyle="-|>",
            color=OU_E,
            lw=1.2,
            mutation_scale=10,
            linestyle="dashed",
            connectionstyle="arc3,rad=-0.2",
        ),
        zorder=4,
    )
    ax.text((init_cx + kfmap_cx) / 2, y_arc,
            "reinit on tracking loss",
            ha="center", va="top",
            fontsize=7.5, color=OU_E, style="italic")

    ax.set_title("mini_stereo_vo_ros2 — VO Pipeline",
                 fontsize=13, fontweight="bold", color=TEXT, pad=10)

    out = OUT_DIR / "vo_pipeline.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"Saved {out}")


# ── Diagram 3: TF Tree ─────────────────────────────────────────────────────────
#
# Clean tree layout — no per-node annotations that could collide.
# Publisher info is shown in a legend instead.
#
# Layout  (FW=12, FH=9):
#   odom            : (6.0, 8.0)
#   base_footprint  : (6.0, 6.8)
#   base_link       : (6.0, 5.6)
#   oakd_link       : (6.0, 4.4)
#   oakd_left_cam   : (2.8, 3.0)   oakd_right_cam  : (9.2, 3.0)
#   oakd_left_opt   : (2.8, 1.6)   oakd_right_opt  : (9.2, 1.6)
#
# Box width 3.0 for single-word frames; checked: left x=[1.3,4.3], right x=[7.7,10.7] — no overlap.

TF_NODES: dict[str, tuple[float, float, str, str]] = {
    # name: (cx, cy, face_color, edge_color)
    # cy values are shifted up by +1.0 vs the natural layout so the content
    # sits close to the title and leaves room for the legend below.
    "odom":                          (6.0, 9.0, OU_F, OU_E),
    "base_footprint":                (6.0, 7.8, RO_F, RO_E),
    "base_link":                     (6.0, 6.6, RO_F, RO_E),
    "oakd_link":                     (6.0, 5.4, RO_F, RO_E),
    "oakd_left_camera_frame":        (2.8, 4.0, GZ_F, GZ_E),
    "oakd_right_camera_frame":       (9.2, 4.0, GZ_F, GZ_E),
    "oakd_left_camera_optical_frame":(2.8, 2.6, GZ_F, GZ_E),
    "oakd_right_camera_optical_frame":(9.2, 2.6, GZ_F, GZ_E),
}

TF_EDGES = [
    ("odom",             "base_footprint"),
    ("base_footprint",   "base_link"),
    ("base_link",        "oakd_link"),
    ("oakd_link",        "oakd_left_camera_frame"),
    ("oakd_link",        "oakd_right_camera_frame"),
    ("oakd_left_camera_frame",   "oakd_left_camera_optical_frame"),
    ("oakd_right_camera_frame",  "oakd_right_camera_optical_frame"),
]

BOX_W_TF, BOX_H_TF = 3.0, 0.60


def gen_tf_tree() -> None:
    FW, FH = 12, 9.8
    fig, ax = _new_ax(FW, FH)

    # Edges first (behind boxes)
    for parent, child in TF_EDGES:
        pcx, pcy = TF_NODES[parent][:2]
        ccx, ccy = TF_NODES[child][:2]
        _arr(ax,
             pcx, pcy - BOX_H_TF / 2,
             ccx, ccy + BOX_H_TF / 2,
             "", GRAY)

    # Boxes on top
    for name, (cx, cy, fc, ec) in TF_NODES.items():
        ax.add_patch(FancyBboxPatch(
            (cx - BOX_W_TF / 2, cy - BOX_H_TF / 2), BOX_W_TF, BOX_H_TF,
            boxstyle="round,pad=0.04",
            linewidth=1.8, edgecolor=ec, facecolor=fc, zorder=3,
        ))
        ax.text(cx, cy, name,
                ha="center", va="center",
                fontsize=8.5, fontweight="bold", color=TEXT, zorder=4)

    # Publisher labels — placed safely to the side of the spine (x=6),
    # not near the split frames.
    PUBLISHERS = [
        ("odom",         "published by stereo_vo_node\n(TF broadcaster)", "right"),
        ("base_footprint","child frame of stereo_vo_node", "right"),
    ]
    for name, pub, side in PUBLISHERS:
        cx, cy = TF_NODES[name][:2]
        offset = BOX_W_TF / 2 + 0.15
        xtext = cx + offset if side == "right" else cx - offset
        ha = "left" if side == "right" else "right"
        ax.text(xtext, cy, pub,
                ha=ha, va="center",
                fontsize=7.0, color=GRAY, style="italic", zorder=5,
                bbox=dict(facecolor=BG, edgecolor=GR_E,
                          boxstyle="round,pad=0.25", alpha=0.9))

    # Legend — positioned with enough gap below the lowest tree nodes
    # (optical frames bottom ≈ 2.3) to avoid crowding.
    legend_items = [
        (OU_F, OU_E, "Published by stereo_vo_node"),
        (RO_F, RO_E, "Published by robot_state_publisher (URDF)"),
        (GZ_F, GZ_E, "Gazebo camera plugin (optical frames)"),
    ]
    lx, ly, lw, lh, lstep = 0.5, 0.45, 0.28, 0.28, 0.50
    for i, (fc, ec, label) in enumerate(legend_items):
        yx = ly + i * lstep
        ax.add_patch(FancyBboxPatch(
            (lx, yx), lw, lh,
            boxstyle="round,pad=0.02",
            linewidth=1.5, edgecolor=ec, facecolor=fc, zorder=3,
        ))
        ax.text(lx + lw + 0.18, yx + lh / 2, label,
                ha="left", va="center", fontsize=7.5, color=TEXT)

    ax.set_title("mini_stereo_vo_ros2 — TF Tree",
                 fontsize=13, fontweight="bold", color=TEXT, pad=4)

    out = OUT_DIR / "tf_tree.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"Saved {out}")


if __name__ == "__main__":
    gen_architecture()
    gen_vo_pipeline()
    gen_tf_tree()
    print("Done.")
