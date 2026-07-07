#!/usr/bin/env python3
"""
Live XY path comparison: GICP localization vs. ground-truth odometry.

Subscribes to two Odometry topics and overlays their XY traces in real time.

Usage:
  python3 plot_odom_comparison.py
  python3 plot_odom_comparison.py --gicp-topic /gicp/localization/odom \
                                   --gt-topic   /localization/global/odom
  python3 plot_odom_comparison.py -o comparison.png   # save on exit
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List, Tuple

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(os.environ.get("TMPDIR", "/tmp")) / "gicp_odom_cmp_matplotlib"),
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
except ImportError as exc:
    print(
        "ROS 2 Python modules are required. Source your ROS 2 environment first.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


XY = Tuple[float, float]

_GICP_COLOR = "steelblue"
_GT_COLOR = "tomato"


class OdomComparisonPlotter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("gicp_odom_comparison_plotter")
        self.args = args

        self.gicp_pts: List[XY] = []
        self.gt_pts: List[XY] = []
        self.start_wall = time.monotonic()
        self.last_wait_log = 0.0

        self.fig, self.ax = plt.subplots(figsize=(12, 10))

        self.gicp_line, = self.ax.plot([], [], color=_GICP_COLOR, linewidth=1.5,
                                       label=args.gicp_topic, zorder=2)
        self.gt_line, = self.ax.plot([], [], color=_GT_COLOR, linewidth=1.5,
                                     linestyle="--", label=args.gt_topic, zorder=2)

        self.gicp_start, = self.ax.plot([], [], color=_GICP_COLOR, marker="o",
                                        linestyle="None", markersize=8, zorder=5)
        self.gt_start, = self.ax.plot([], [], color=_GT_COLOR, marker="o",
                                      linestyle="None", markersize=8, zorder=5)

        self.gicp_latest, = self.ax.plot([], [], color=_GICP_COLOR, marker="x",
                                         linestyle="None", markersize=8,
                                         markeredgewidth=2, zorder=6)
        self.gt_latest, = self.ax.plot([], [], color=_GT_COLOR, marker="x",
                                       linestyle="None", markersize=8,
                                       markeredgewidth=2, zorder=6)

        self.waiting_text = self.ax.text(
            0.5, 0.5, "Waiting for odometry…",
            transform=self.ax.transAxes,
            ha="center", va="center", fontsize=14, color="0.4",
        )

        self.ax.set_title(args.title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True, alpha=0.25)
        self.ax.legend(loc="best", fontsize=8)

        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, args.gicp_topic,
                                 self._gicp_cb, best_effort_qos)
        self.create_subscription(Odometry, args.gt_topic,
                                 self._gt_cb, best_effort_qos)
        self.create_timer(args.update_period, self._update_plot)

        plt.ion()
        self.fig.show()
        self.get_logger().info(
            f"Listening: gicp={args.gicp_topic}  gt={args.gt_topic}"
        )

    def _gicp_cb(self, msg: Odometry) -> None:
        self.gicp_pts.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
        if len(self.gicp_pts) == 1:
            self.get_logger().info(f"first GICP sample: frame={msg.header.frame_id}")

    def _gt_cb(self, msg: Odometry) -> None:
        self.gt_pts.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
        if len(self.gt_pts) == 1:
            self.get_logger().info(f"first GT sample: frame={msg.header.frame_id}")

    def _update_plot(self) -> None:
        if not self.gicp_pts and not self.gt_pts:
            self._maybe_log_waiting()
            plt.pause(0.001)
            return

        if self.waiting_text is not None:
            self.waiting_text.remove()
            self.waiting_text = None

        if self.gicp_pts:
            xs, ys = zip(*self.gicp_pts)
            self.gicp_line.set_data(xs, ys)
            self.gicp_start.set_data([xs[0]], [ys[0]])
            self.gicp_latest.set_data([xs[-1]], [ys[-1]])

        if self.gt_pts:
            xs, ys = zip(*self.gt_pts)
            self.gt_line.set_data(xs, ys)
            self.gt_start.set_data([xs[0]], [ys[0]])
            self.gt_latest.set_data([xs[-1]], [ys[-1]])

        self.ax.set_title(
            f"{self.args.title}  |  GICP: {len(self.gicp_pts)} pts  "
            f"GT: {len(self.gt_pts)} pts"
        )
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.axis("equal")
        self.ax.legend(loc="best", fontsize=8)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def _maybe_log_waiting(self) -> None:
        now = time.monotonic()
        if now - self.last_wait_log < 3.0:
            return
        self.last_wait_log = now
        elapsed = now - self.start_wall
        gicp_pubs = len(self.get_publishers_info_by_topic(self.args.gicp_topic))
        gt_pubs = len(self.get_publishers_info_by_topic(self.args.gt_topic))
        self.get_logger().warn(
            f"waiting {elapsed:.1f}s — gicp publishers={gicp_pubs}  "
            f"gt publishers={gt_pubs}"
        )

    def save(self) -> None:
        if not self.args.output:
            return
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.fig.savefig(self.args.output, dpi=160)
        print(f"Saved {self.args.output}")
        print(f"GICP samples : {len(self.gicp_pts)}")
        print(f"GT samples   : {len(self.gt_pts)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--gicp-topic",
        default="/gicp/localization/odom",
        help="GICP localization odometry topic (default: %(default)s)",
    )
    parser.add_argument(
        "--gt-topic",
        default="/localization/global/odom",
        help="Ground-truth odometry topic (default: %(default)s)",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Save plot to this PNG path when the window is closed.",
    )
    parser.add_argument("--title", default="GICP vs GT Odometry (live)")
    parser.add_argument(
        "--update-period", type=float, default=0.25,
        help="Plot refresh interval in seconds (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = OdomComparisonPlotter(args)
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
