#!/usr/bin/env python3
"""
Live plotter for GT recovery snap events.

Subscribes to /gicp/localization/odom for the path trace and
/gicp/localization/gt_snap for snap events, plotting them in real time.

Usage:
  python3 plot_source_switches.py
  python3 plot_source_switches.py --odom-topic /gicp/localization/odom
  python3 plot_source_switches.py -o out.png   # save on exit
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(os.environ.get("TMPDIR", "/tmp")) / "gicp_gt_snap_matplotlib"),
)

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    print(
        "matplotlib is required. Install with: sudo apt install python3-matplotlib",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
except ImportError as exc:
    print(
        "ROS 2 Python modules are required. Source your ROS 2 environment first.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


PathPoint = Tuple[float, float, float]   # (t, x, y)
SnapPoint = Tuple[float, float, float]   # (t, x, y)

_SNAP_COLOR = "red"
_PATH_COLOR = "black"


class LiveGtSnapPlotter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("gicp_gt_snap_plotter")
        self.args = args

        self.path_points: List[PathPoint] = []
        self.snap_points: List[SnapPoint] = []
        self.path_seen = False
        self.start_wall = time.monotonic()
        self.last_wait_log = 0.0

        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.path_line, = self.ax.plot([], [], color=_PATH_COLOR, linewidth=1.0,
                                       label="localization path", zorder=1)
        self.start_dot, = self.ax.plot([], [], color="dodgerblue", marker="o",
                                       linestyle="None", markersize=7,
                                       label="start", zorder=5)
        self.latest_dot, = self.ax.plot([], [], color="black", marker="x",
                                        linestyle="None", markersize=7,
                                        label="latest", zorder=5)
        self.snap_scatter = self.ax.scatter([], [], color=_SNAP_COLOR,
                                            edgecolors="white", linewidths=0.8,
                                            s=160, marker="*", zorder=6,
                                            label="GT snap")
        self._snap_annotations: list = []

        self.waiting_text = self.ax.text(
            0.5, 0.5,
            f"Waiting for path on {args.odom_topic}",
            transform=self.ax.transAxes,
            ha="center", va="center", fontsize=14, color="0.4",
        )

        self.ax.set_title(args.title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True, alpha=0.25)
        self.ax.legend(loc="best", fontsize=8)

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, args.odom_topic,
                                 self._odom_cb, best_effort_qos)
        self.create_subscription(PoseStamped, args.snap_topic,
                                 self._snap_cb, reliable_qos)
        self.create_timer(args.update_period, self._update_plot)

        plt.ion()
        self.fig.show()
        self.get_logger().info(
            f"Listening: path={args.odom_topic}  snaps={args.snap_topic}"
        )

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _odom_cb(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.path_points.append((t, x, y))
        if not self.path_seen:
            self.path_seen = True
            if self.waiting_text is not None:
                self.waiting_text.remove()
                self.waiting_text = None
            self.get_logger().info(
                f"first odom sample: xy=({x:.2f}, {y:.2f}) frame={msg.header.frame_id}"
            )

    def _snap_cb(self, msg: PoseStamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.snap_points.append((t, x, y))
        self.get_logger().warn(
            f"GT snap #{len(self.snap_points)}: xy=({x:.2f}, {y:.2f})"
        )

    # ------------------------------------------------------------------ #
    # Plot update                                                          #
    # ------------------------------------------------------------------ #

    def _update_plot(self) -> None:
        if not self.path_points:
            self._maybe_log_waiting()
            plt.pause(0.001)
            return

        xs = [p[1] for p in self.path_points]
        ys = [p[2] for p in self.path_points]
        self.path_line.set_data(xs, ys)
        self.start_dot.set_data([xs[0]], [ys[0]])
        self.latest_dot.set_data([xs[-1]], [ys[-1]])

        if self.snap_points:
            snap_xs = [p[1] for p in self.snap_points]
            snap_ys = [p[2] for p in self.snap_points]
            self.snap_scatter.set_offsets(list(zip(snap_xs, snap_ys)))

            # Rebuild annotations only when count changed
            if len(self._snap_annotations) != len(self.snap_points):
                for ann in self._snap_annotations:
                    ann.remove()
                self._snap_annotations = []
                for idx, (_, sx, sy) in enumerate(self.snap_points, start=1):
                    ann = self.ax.annotate(
                        str(idx),
                        xy=(sx, sy),
                        xytext=(6, 6),
                        textcoords="offset points",
                        fontsize=8,
                        color=_SNAP_COLOR,
                        weight="bold",
                    )
                    self._snap_annotations.append(ann)

        n_snaps = len(self.snap_points)
        self.ax.set_title(f"{self.args.title}  |  GT snaps: {n_snaps}")
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.axis("equal")
        self.ax.legend(loc="best", fontsize=8)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _maybe_log_waiting(self) -> None:
        now = time.monotonic()
        if now - self.last_wait_log < 3.0:
            return
        self.last_wait_log = now
        elapsed = now - self.start_wall
        pubs = self.get_publishers_info_by_topic(self.args.odom_topic)
        self.get_logger().warn(
            f"waiting {elapsed:.1f}s for path on {self.args.odom_topic} "
            f"(publishers={len(pubs)})"
        )

    def save(self) -> None:
        if not self.args.output:
            return
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.fig.savefig(self.args.output, dpi=160)
        print(f"Saved {self.args.output}")
        print(f"Path samples : {len(self.path_points)}")
        print(f"GT snaps     : {len(self.snap_points)}")
        for idx, (t, x, y) in enumerate(self.snap_points, start=1):
            print(f"  {idx:02d}  t={t:.3f}  xy=({x:.3f}, {y:.3f})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--odom-topic",
        default="/gicp/localization/odom",
        help="Odometry topic for path trace (default: %(default)s)",
    )
    parser.add_argument(
        "--snap-topic",
        default="/gicp/localization/gt_snap",
        help="PoseStamped topic published on each GT snap (default: %(default)s)",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Save plot to this PNG path when the window is closed.",
    )
    parser.add_argument("--title", default="GT Recovery Snaps (live)")
    parser.add_argument(
        "--update-period", type=float, default=0.25,
        help="Plot refresh interval in seconds (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = LiveGtSnapPlotter(args)
    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
