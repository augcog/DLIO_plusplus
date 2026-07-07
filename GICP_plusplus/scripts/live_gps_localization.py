#!/usr/bin/env python3
"""
Live ROS 2 visualizer: GPS ground truth vs. localization output.

Subscribes to a GPS topic (PoseStamped / PoseWithCovarianceStamped / Odometry)
and the localizer's pose topic, then updates a matplotlib window with the two
trajectories and the same stats box produced by ``compare_gps_localization.py``.

Usage::

    source /opt/ros/iron/setup.bash
    source /home/roar-blacktank/DLIO_plusplus/install/setup.bash
    python3 live_gps_localization.py \
        --gps-topic /gps_bot/odom \
        --loc-topic /gicp/localization/pose

Run this alongside the localization node (whether live from sensors or during
``ros2 bag play``). Press Ctrl+C or close the plot window to exit.
"""

import argparse
import sys
import threading
import time
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


_TYPE_STRING_TO_CLS = {
    "geometry_msgs/msg/PoseStamped": PoseStamped,
    "geometry_msgs/msg/PoseWithCovarianceStamped": PoseWithCovarianceStamped,
    "nav_msgs/msg/Odometry": Odometry,
}


def _extract_xy(msg) -> Optional[Tuple[float, float, float, float, float]]:
    """Return (stamp, x, y, z, yaw_rad) — z and yaw are 0 if message has no orientation."""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if isinstance(msg, PoseStamped):
        pose = msg.pose
    elif isinstance(msg, PoseWithCovarianceStamped):
        pose = msg.pose.pose
    elif isinstance(msg, Odometry):
        pose = msg.pose.pose
    else:
        return None
    p = pose.position
    if not (np.isfinite(p.x) and np.isfinite(p.y)):
        return None
    q = pose.orientation
    # Yaw from quaternion: atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
    yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    return stamp, float(p.x), float(p.y), float(p.z), yaw


def umeyama_2d(src: np.ndarray, dst: np.ndarray, with_scale: bool = False) -> np.ndarray:
    """Rigid (or similarity) SE(2) mapping ``src`` onto ``dst``."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    s_c = src - mu_s
    d_c = dst - mu_d
    H = s_c.T @ d_c / src.shape[0]
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        S[-1] *= -1
        R = Vt.T @ U.T
    scale = 1.0
    if with_scale:
        var_s = (s_c ** 2).sum() / src.shape[0]
        scale = S.sum() / var_s if var_s > 0 else 1.0
    t = mu_d - scale * R @ mu_s
    T = np.eye(3)
    T[:2, :2] = scale * R
    T[:2, 2] = t
    return T


def apply_se2(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return (pts @ T[:2, :2].T) + T[:2, 2]


def time_match_distances(
    loc_t: np.ndarray, loc_xy: np.ndarray, gps_t: np.ndarray, gps_xy: np.ndarray, tol: float
) -> np.ndarray:
    if len(loc_t) == 0 or len(gps_t) == 0:
        return np.empty(0)
    idx = np.searchsorted(gps_t, loc_t)
    left = np.clip(idx - 1, 0, len(gps_t) - 1)
    right = np.clip(idx, 0, len(gps_t) - 1)
    dt_l = np.abs(loc_t - gps_t[left])
    dt_r = np.abs(gps_t[right] - loc_t)
    pick_r = dt_r < dt_l
    best = np.where(pick_r, right, left)
    dt = np.where(pick_r, dt_r, dt_l)
    keep = dt <= tol
    if not keep.any():
        return np.empty(0)
    return np.linalg.norm(loc_xy[keep] - gps_xy[best[keep]], axis=1)


def nearest_neighbor_distances(loc_xy: np.ndarray, gps_xy: np.ndarray) -> np.ndarray:
    if len(loc_xy) == 0 or len(gps_xy) == 0:
        return np.empty(0)
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(gps_xy)
        d, _ = tree.query(loc_xy, k=1)
        return d
    except ImportError:
        out = np.empty(len(loc_xy), dtype=np.float64)
        chunk = 2048
        for i in range(0, len(loc_xy), chunk):
            blk = loc_xy[i : i + chunk]
            diff = blk[:, None, :] - gps_xy[None, :, :]
            out[i : i + chunk] = np.sqrt((diff ** 2).sum(axis=2)).min(axis=1)
        return out


def summarize(d: np.ndarray) -> dict:
    if len(d) == 0:
        return {"rmse": float("nan"), "mean": float("nan"), "std": float("nan"),
                "min": float("nan"), "max": float("nan"), "samples": 0}
    return {
        "rmse": float(np.sqrt(np.mean(d ** 2))),
        "mean": float(d.mean()),
        "std": float(d.std()),
        "min": float(d.min()),
        "max": float(d.max()),
        "samples": int(len(d)),
    }


def format_stats_block(title: str, s: dict) -> str:
    return (f"{title}\n"
            f"RMSE: {s['rmse']:.3f} m\n"
            f"Mean: {s['mean']:.3f} m\n"
            f"Std:  {s['std']:.3f} m\n"
            f"Min:  {s['min']:.3f} m\n"
            f"Max:  {s['max']:.3f} m\n"
            f"Samples: {s['samples']}")


class LiveCompare(Node):
    def __init__(self, gps_topic: str, loc_topic: str, gps_type: Optional[str],
                 loc_type: Optional[str], align: bool, with_scale: bool,
                 time_tol: float, max_points: int):
        super().__init__("live_gps_vs_localization")

        self.time_tol = time_tol
        self.align = align
        self.with_scale = with_scale
        self.max_points = max_points

        self.lock = threading.Lock()
        # Each entry: (stamp, x, y, z, yaw_rad)
        self.gps: List[Tuple[float, float, float, float, float]] = []
        self.loc: List[Tuple[float, float, float, float, float]] = []

        gps_cls = self._resolve_type(gps_topic, gps_type, default=Odometry)
        loc_cls = self._resolve_type(loc_topic, loc_type, default=PoseStamped)

        gps_qos = self._match_publisher_qos(gps_topic)
        loc_qos = self._match_publisher_qos(loc_topic)
        self.create_subscription(gps_cls, gps_topic, self._on_gps, gps_qos)
        self.create_subscription(loc_cls, loc_topic, self._on_loc, loc_qos)

        self.get_logger().info(
            f"Subscribed: GPS '{gps_topic}' ({gps_cls.__name__}, {gps_qos.reliability.name}) | "
            f"LOC '{loc_topic}' ({loc_cls.__name__}, {loc_qos.reliability.name})"
        )

    def _match_publisher_qos(self, topic: str) -> QoSProfile:
        """Return a QoS profile compatible with whatever is publishing ``topic``.

        Falls back to BEST_EFFORT (the permissive setting — a best-effort
        subscriber accepts messages from both best-effort and reliable
        publishers). This avoids the classic silent-subscription QoS mismatch
        when a bag replays a best-effort topic.
        """
        reliability = ReliabilityPolicy.BEST_EFFORT
        durability = DurabilityPolicy.VOLATILE
        for info in self.get_publishers_info_by_topic(topic):
            pub_qos = info.qos_profile
            if pub_qos.reliability == ReliabilityPolicy.RELIABLE:
                reliability = ReliabilityPolicy.RELIABLE
            if pub_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL:
                durability = DurabilityPolicy.TRANSIENT_LOCAL
            break
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=reliability,
            durability=durability,
        )

    def _resolve_type(self, topic: str, explicit: Optional[str], default):
        if explicit is not None:
            if explicit not in _TYPE_STRING_TO_CLS:
                raise RuntimeError(f"Unsupported type override: {explicit}")
            return _TYPE_STRING_TO_CLS[explicit]
        # Attempt discovery — wait briefly for publisher info.
        for _ in range(20):
            infos = self.get_publishers_info_by_topic(topic)
            for info in infos:
                t = info.topic_type
                if t in _TYPE_STRING_TO_CLS:
                    return _TYPE_STRING_TO_CLS[t]
            time.sleep(0.1)
        self.get_logger().warn(
            f"Could not discover publisher type for {topic}; defaulting to {default.__name__}."
        )
        return default

    def _on_gps(self, msg):
        v = _extract_xy(msg)
        if v is None:
            return
        with self.lock:
            self.gps.append(v)
            if self.max_points and len(self.gps) > self.max_points:
                del self.gps[: len(self.gps) - self.max_points]

    def _on_loc(self, msg):
        v = _extract_xy(msg)
        if v is None:
            return
        with self.lock:
            self.loc.append(v)
            if self.max_points and len(self.loc) > self.max_points:
                del self.loc[: len(self.loc) - self.max_points]

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray]:
        with self.lock:
            gps = np.asarray(self.gps, dtype=np.float64) if self.gps else np.empty((0, 5))
            loc = np.asarray(self.loc, dtype=np.float64) if self.loc else np.empty((0, 5))
        return gps, loc


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gps-topic", default="/gps_bot/odom")
    p.add_argument("--loc-topic", default="/gicp/localization/pose")
    p.add_argument("--gps-type", default=None,
                   help=f"Override detected type, one of {sorted(_TYPE_STRING_TO_CLS)}.")
    p.add_argument("--loc-type", default=None)
    p.add_argument("--no-align", action="store_true", help="Skip rigid XY alignment.")
    p.add_argument("--with-scale", action="store_true", help="Similarity instead of rigid alignment.")
    p.add_argument("--time-tolerance-ms", type=float, default=50.0)
    p.add_argument("--update-hz", type=float, default=3.0, help="Plot refresh rate.")
    p.add_argument("--min-pairs-for-lock", type=int, default=200,
                   help="Lock alignment permanently once this many time-matched pairs are available.")
    p.add_argument("--realign-every-s", type=float, default=2.0,
                   help="Recompute alignment no faster than this (before lock).")
    p.add_argument("--max-points", type=int, default=0,
                   help="Cap stored samples per topic (0 = unlimited).")
    p.add_argument("--title", default="GPS vs Localization Trajectories (live)")
    p.add_argument("--bag-start", type=float, default=None,
                   help="Bag start epoch seconds (from `ros2 bag info` Start: line). "
                        "When provided, click prints offset = clicked_stamp - bag_start, "
                        "directly usable as `ros2 bag play --start-offset`.")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    import matplotlib.pyplot as plt

    rclpy.init()
    node = LiveCompare(
        gps_topic=args.gps_topic,
        loc_topic=args.loc_topic,
        gps_type=args.gps_type,
        loc_type=args.loc_type,
        align=not args.no_align,
        with_scale=args.with_scale,
        time_tol=args.time_tolerance_ms / 1000.0,
        max_points=args.max_points,
    )

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 7))
    gps_line, = ax.plot([], [], color="green", linewidth=1.5, label="GPS Trajectory")
    loc_line, = ax.plot([], [], color="red", linestyle="--", linewidth=1.2,
                        label="Localization Trajectory (Aligned)" if not args.no_align
                        else "Localization Trajectory")
    ax.set_title(args.title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")
    ax.legend(loc="upper right")
    stats_text = ax.text(
        0.02, 0.98, "",
        transform=ax.transAxes, fontsize=9, family="monospace",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="wheat", edgecolor="0.6", alpha=0.9),
    )
    fig.tight_layout()

    # Close-handler flag so the main loop can exit cleanly when the window shuts.
    closed = {"v": False}
    fig.canvas.mpl_connect("close_event", lambda _e: closed.update(v=True))

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    update_period = 1.0 / max(args.update_hz, 0.1)
    last_plot = 0.0
    last_realign = 0.0
    last_heartbeat = 0.0
    T_align = np.eye(3)
    alignment_locked = False

    # Mutable state shared with the click handler.
    click_state = {"T_align": T_align}

    def _format_time(stamp_s: float) -> str:
        """Print absolute bag time (sim time) and, if --bag-start was given, the offset."""
        if args.bag_start is not None:
            offset = stamp_s - args.bag_start
            return f"bag_t={stamp_s:.3f}s offset={offset:.2f}s"
        return f"bag_t={stamp_s:.3f}s"

    def on_click(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        click_xy = np.array([event.xdata, event.ydata])
        gps_s, loc_s = node.snapshot()
        parts = []
        loc_idx = None
        loc_min_d = None
        if gps_s.shape[0] > 0:
            d = np.linalg.norm(gps_s[:, 1:3] - click_xy, axis=1)
            i = int(np.argmin(d))
            parts.append(
                f"GPS {_format_time(float(gps_s[i, 0]))} "
                f"xy=({gps_s[i,1]:.2f},{gps_s[i,2]:.2f}) z={gps_s[i,3]:.2f} "
                f"yaw={np.degrees(gps_s[i,4]):.2f}deg d={d[i]:.2f}m"
            )
        if loc_s.shape[0] > 0:
            loc_disp = loc_s[:, 1:3].copy()
            if not args.no_align:
                loc_disp = apply_se2(click_state["T_align"], loc_disp)
            d = np.linalg.norm(loc_disp - click_xy, axis=1)
            i = int(np.argmin(d))
            loc_idx = i
            loc_min_d = float(d[i])
            parts.append(
                f"LOC {_format_time(float(loc_s[i, 0]))} "
                f"xy_aligned=({loc_disp[i,0]:.2f},{loc_disp[i,1]:.2f}) "
                f"xy_raw=({loc_s[i,1]:.2f},{loc_s[i,2]:.2f}) z={loc_s[i,3]:.2f} "
                f"yaw={np.degrees(loc_s[i,4]):.2f}deg d={d[i]:.2f}m"
            )
        print(f"[click @ ({click_xy[0]:.2f},{click_xy[1]:.2f})] " + " | ".join(parts), flush=True)
        # Ready-to-paste YAML snippet for restarting from this map-frame pose.
        if loc_idx is not None:
            if loc_min_d > 5.0:
                print(f"  WARNING: nearest LOC sample is {loc_min_d:.2f} m from click — "
                      f"zoom in and click directly on the trajectory line.", flush=True)
            t = float(loc_s[loc_idx, 0])
            offset_str = (f"    # ros2 bag play <bag> --start-offset {t - args.bag_start:.2f}"
                          if args.bag_start is not None
                          else f"    # bag time = {t:.3f}s; subtract `ros2 bag info` Start: to get --start-offset")
            print(
                "  YAML snippet (paste into localization.yaml):\n"
                f"    localization/initial_pose/use: true\n"
                f"    localization/initial_pose/frame: \"base_link\"   # /gicp/localization/pose is T_world_baselink\n"
                f"    localization/initial_pose/x: {loc_s[loc_idx,1]:.6f}\n"
                f"    localization/initial_pose/y: {loc_s[loc_idx,2]:.6f}\n"
                f"    localization/initial_pose/z: {loc_s[loc_idx,3]:.6f}\n"
                f"    localization/initial_pose/roll: 0.0\n"
                f"    localization/initial_pose/pitch: 0.0\n"
                f"    localization/initial_pose/yaw: {loc_s[loc_idx,4]:.6f}\n"
                f"{offset_str}",
                flush=True,
            )

    fig.canvas.mpl_connect("button_press_event", on_click)

    try:
        while rclpy.ok() and not closed["v"]:
            executor.spin_once(timeout_sec=0.02)

            now = time.monotonic()

            if now - last_heartbeat >= 2.0:
                gps_n = len(node.gps)
                loc_n = len(node.loc)
                node.get_logger().info(f"messages received: gps={gps_n}  loc={loc_n}")
                last_heartbeat = now

            if now - last_plot < update_period:
                continue
            last_plot = now

            gps, loc = node.snapshot()

            # If we don't yet have enough for an alignment, still render
            # whichever trace has data so an empty side hints at a silent topic.
            if gps.shape[0] < 2 or loc.shape[0] < 2:
                if gps.shape[0] >= 1:
                    gps_line.set_data(gps[:, 1], gps[:, 2])
                if loc.shape[0] >= 1:
                    loc_line.set_data(loc[:, 1], loc[:, 2])
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw_idle()
                plt.pause(0.001)
                continue

            gps_xy = gps[:, 1:3]
            loc_xy = loc[:, 1:3].copy()
            gps_t = gps[:, 0]
            loc_t = loc[:, 0]

            if (not args.no_align and not alignment_locked
                    and now - last_realign >= args.realign_every_s):
                # Use time-matched pairs for alignment to avoid density bias.
                idx = np.clip(np.searchsorted(gps_t, loc_t), 0, len(gps_t) - 1)
                keep = np.abs(gps_t[idx] - loc_t) <= node.time_tol
                n_pairs = int(keep.sum())
                if n_pairs >= 10:
                    T_align = umeyama_2d(loc_xy[keep], gps_xy[idx[keep]],
                                         with_scale=args.with_scale)
                    click_state["T_align"] = T_align
                    last_realign = now
                    if n_pairs >= args.min_pairs_for_lock:
                        alignment_locked = True
                        node.get_logger().info(
                            f"Alignment locked after {n_pairs} time-matched pairs."
                        )
            if not args.no_align:
                loc_xy = apply_se2(T_align, loc_xy)

            tm_dist = time_match_distances(loc_t, loc_xy, gps_t, gps_xy, node.time_tol)
            nn_dist = nearest_neighbor_distances(loc_xy, gps_xy)

            gps_line.set_data(gps_xy[:, 0], gps_xy[:, 1])
            loc_line.set_data(loc_xy[:, 0], loc_xy[:, 1])
            ax.relim()
            ax.autoscale_view()

            stats_text.set_text(
                format_stats_block("Time-matched", summarize(tm_dist))
                + "\n\n"
                + format_stats_block("Nearest-neighbor", summarize(nn_dist))
            )

            fig.canvas.draw_idle()
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        if not closed["v"]:
            plt.close(fig)

    return 0


if __name__ == "__main__":
    sys.exit(main())
