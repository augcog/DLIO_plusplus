#!/usr/bin/env python3
"""
Offline XY path comparison: GICP localization vs. ground-truth odometry.

Reads two Odometry topics from a rosbag2 MCAP and overlays their XY traces.
Paths are matched by header timestamp so latency differences don't skew the plot.

Usage:
  python3 plot_odom_comparison_bag.py /path/to/bag
  python3 plot_odom_comparison_bag.py /path/to/bag \\
      --gicp-topic /gicp/localization/odom \\
      --gt-topic   /localization/global/odom
  python3 plot_odom_comparison_bag.py /path/to/bag -o comparison.png
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import rosbag2_py
    import rclpy.serialization
    from nav_msgs.msg import Odometry
except ImportError as exc:
    print(
        "rosbag2_py / rclpy are required. Source your ROS 2 environment first.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    print(
        "matplotlib is required: sudo apt install python3-matplotlib",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


def read_odom(bag_path: str, topics: list[str]) -> dict[str, list[tuple[float, float, float]]]:
    opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(opts, conv)

    filt = rosbag2_py.StorageFilter()
    filt.topics = topics
    reader.set_filter(filt)

    data: dict[str, list[tuple[float, float, float]]] = {t: [] for t in topics}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in data:
            continue
        msg: Odometry = rclpy.serialization.deserialize_message(raw, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        data[topic].append((t, x, y))

    return data


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("bag", help="Path to rosbag2 MCAP directory or file")
    p.add_argument(
        "--gicp-topic", default="/gicp/localization/odom",
        help="GICP localization odom topic (default: %(default)s)",
    )
    p.add_argument(
        "--gt-topic", default="/localization/global/odom",
        help="Ground-truth odom topic (default: %(default)s)",
    )
    p.add_argument(
        "-o", "--output", type=Path, default=None,
        help="Save plot to PNG instead of (or in addition to) displaying it",
    )
    p.add_argument("--title", default="GICP vs GT Odometry")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    print(f"Reading bag: {args.bag}")
    print(f"  GICP topic : {args.gicp_topic}")
    print(f"  GT topic   : {args.gt_topic}")

    raw = read_odom(args.bag, [args.gicp_topic, args.gt_topic])
    gicp_pts = raw[args.gicp_topic]
    gt_pts   = raw[args.gt_topic]

    if not gicp_pts and not gt_pts:
        print("ERROR: no messages found for either topic.", file=sys.stderr)
        return 1
    if not gicp_pts:
        print(f"WARNING: no messages on {args.gicp_topic}", file=sys.stderr)
    if not gt_pts:
        print(f"WARNING: no messages on {args.gt_topic}", file=sys.stderr)

    print(f"  GICP samples : {len(gicp_pts)}")
    print(f"  GT samples   : {len(gt_pts)}")

    # ------------------------------------------------------------------ #
    # XY path plot                                                         #
    # ------------------------------------------------------------------ #
    fig, ax = plt.subplots(figsize=(12, 10))

    _GICP_COLOR = "steelblue"
    _GT_COLOR   = "tomato"

    def plot_path(pts, color, label, linestyle="-"):
        if not pts:
            return
        xs = [p[1] for p in pts]
        ys = [p[2] for p in pts]
        ax.plot(xs, ys, color=color, linewidth=1.5, linestyle=linestyle, label=label, zorder=2)
        ax.plot(xs[0],  ys[0],  color=color, marker="o", linestyle="None",
                markersize=8, zorder=5)
        ax.plot(xs[-1], ys[-1], color=color, marker="x", linestyle="None",
                markersize=8, markeredgewidth=2, zorder=6)

    plot_path(gicp_pts, _GICP_COLOR, args.gicp_topic)
    plot_path(gt_pts,   _GT_COLOR,   args.gt_topic, linestyle="--")

    ax.set_title(args.title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=9)

    fig.tight_layout()

    # ------------------------------------------------------------------ #
    # Per-timestamp error plot (matched by nearest header stamp)          #
    # ------------------------------------------------------------------ #
    if gicp_pts and gt_pts:
        gicp_arr = np.array(gicp_pts)   # (N, 3): t, x, y
        gt_arr   = np.array(gt_pts)     # (M, 3): t, x, y

        # For each GICP sample find the nearest GT sample by header timestamp
        gicp_t = gicp_arr[:, 0]
        gt_t   = gt_arr[:, 0]
        idx = np.searchsorted(gt_t, gicp_t).clip(0, len(gt_t) - 1)
        # Also check neighbour to get true nearest
        idx_prev = (idx - 1).clip(0, len(gt_t) - 1)
        dt_curr = np.abs(gt_t[idx]      - gicp_t)
        dt_prev = np.abs(gt_t[idx_prev] - gicp_t)
        best = np.where(dt_prev < dt_curr, idx_prev, idx)

        dt_match = np.abs(gt_t[best] - gicp_t)
        valid = dt_match < 0.5   # drop pairs more than 500ms apart

        err_x = gicp_arr[valid, 1] - gt_arr[best[valid], 1]
        err_y = gicp_arr[valid, 2] - gt_arr[best[valid], 2]
        err_2d = np.sqrt(err_x**2 + err_y**2)
        t_rel  = gicp_t[valid] - gicp_t[0]

        fig2, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
        fig2.suptitle(f"{args.title} — position error (GICP − GT)")

        axes[0].plot(t_rel, err_x, color=_GICP_COLOR, linewidth=1.0, label="err x")
        axes[0].plot(t_rel, err_y, color=_GT_COLOR,   linewidth=1.0, label="err y", linestyle="--")
        axes[0].axhline(0, color="k", linewidth=0.5)
        axes[0].set_ylabel("error [m]")
        axes[0].legend(fontsize=8)
        axes[0].grid(True, alpha=0.25)

        axes[1].plot(t_rel, err_2d, color="purple", linewidth=1.0, label="2D error")
        axes[1].fill_between(t_rel, err_2d, alpha=0.2, color="purple")
        axes[1].set_ylabel("2D error [m]")
        axes[1].set_xlabel("time [s]")
        axes[1].grid(True, alpha=0.25)
        axes[1].legend(fontsize=8)

        print(f"\nPosition error (GICP − GT, {valid.sum()} matched pairs):")
        print(f"  mean 2D : {err_2d.mean():.3f} m")
        print(f"  median  : {np.median(err_2d):.3f} m")
        print(f"  p95     : {np.percentile(err_2d, 95):.3f} m")
        print(f"  max     : {err_2d.max():.3f} m")

        fig2.tight_layout()

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=160)
        print(f"\nSaved path plot to {args.output}")
        if gicp_pts and gt_pts:
            err_path = args.output.with_stem(args.output.stem + "_error")
            fig2.savefig(err_path, dpi=160)
            print(f"Saved error plot to {err_path}")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
