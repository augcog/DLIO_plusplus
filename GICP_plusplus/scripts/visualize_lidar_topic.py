#!/usr/bin/env python3
"""
Visualize and verify the LiDAR topic used by localization.

This tool subscribes to the raw LiDAR topic and optionally to the localization
debug topic that publishes the cloud after the node's input transform chain:

  1. transform raw cloud into base frame
  2. optionally flip Y

When the debug topic is available, the script reconstructs that same expected
cloud from the raw topic and compares the two point sets directly. This lets
you verify that localization is consuming the topic and transform you think it
is.
"""

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def point_field_type_name(datatype: int) -> str:
    mapping = {
        PointField.INT8: "int8",
        PointField.UINT8: "uint8",
        PointField.INT16: "int16",
        PointField.UINT16: "uint16",
        PointField.INT32: "int32",
        PointField.UINT32: "uint32",
        PointField.FLOAT32: "float32",
        PointField.FLOAT64: "float64",
    }
    return mapping.get(datatype, f"unknown({datatype})")


def field_names(msg: PointCloud2) -> List[str]:
    return [field.name for field in msg.fields]


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def cloud_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(points, dtype=np.float64)


def sample_points(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    stride = int(math.ceil(len(points) / float(max_points)))
    return points[::stride]


def bounds_text(points: np.ndarray) -> str:
    if points.size == 0:
        return "empty"
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    return (
        f"x[{mins[0]:.2f}, {maxs[0]:.2f}] "
        f"y[{mins[1]:.2f}, {maxs[1]:.2f}] "
        f"z[{mins[2]:.2f}, {maxs[2]:.2f}]"
    )


def radial_summary(points: np.ndarray) -> Tuple[float, float]:
    if points.size == 0:
        return float("nan"), float("nan")
    radii = np.linalg.norm(points[:, :2], axis=1)
    return float(np.mean(radii)), float(np.max(radii))


@dataclass
class CloudSample:
    stamp_sec: float
    frame_id: str
    point_count: int
    points: np.ndarray
    fields: List[str]


class LidarTopicVisualizer(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("lidar_topic_visualizer")

        self.raw_topic = args.topic
        self.verify_topic = args.verify_topic
        self.base_frame = args.base_frame
        self.expected_frame = args.expected_frame
        self.flip_y = args.flip_y
        self.max_plot_points = args.max_plot_points
        self.verify_timeout_sec = args.verify_timeout_sec
        self.finished = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_raw: Optional[CloudSample] = None
        self.latest_expected: Optional[CloudSample] = None
        self.latest_verify: Optional[CloudSample] = None

        self.raw_count = 0
        self.verify_count = 0
        self.last_raw_wall_time: Optional[float] = None
        self.last_verify_wall_time: Optional[float] = None
        self.raw_rate_hz: Optional[float] = None
        self.verify_rate_hz: Optional[float] = None

        self.last_compare_summary = "Waiting for data"
        self.last_raw_summary = "Waiting for raw cloud"
        self.last_verify_summary = "Waiting for localization debug cloud"

        self.raw_subscription = self.create_subscription(
            PointCloud2,
            self.raw_topic,
            self._on_raw_cloud,
            qos_profile_sensor_data,
        )
        self.verify_subscription = self.create_subscription(
            PointCloud2,
            self.verify_topic,
            self._on_verify_cloud,
            qos_profile_sensor_data,
        )

        self._setup_plot()
        self.get_logger().info(f"Subscribing to raw topic: {self.raw_topic}")
        self.get_logger().info(f"Subscribing to verification topic: {self.verify_topic}")
        self.get_logger().info(
            f"Reconstructing expected localization input with base_frame='{self.base_frame}', flip_y={self.flip_y}"
        )

    def _setup_plot(self) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError(
                "matplotlib is required for visualization. Install python3-matplotlib."
            ) from exc

        self.plt = plt
        plt.ion()
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig = fig
        self.raw_ax = axes[0, 0]
        self.expected_ax = axes[0, 1]
        self.verify_ax = axes[1, 0]
        self.overlay_ax = axes[1, 1]

        self.raw_ax.set_title("Raw LiDAR Topic (XY)")
        self.expected_ax.set_title("Expected Localization Input (XY)")
        self.verify_ax.set_title("Actual Localization Debug Input (XY)")
        self.overlay_ax.set_title("Expected vs Actual Overlay (XY)")

        self.raw_scatter = self.raw_ax.scatter([], [], s=1.0, c="tab:blue", alpha=0.65)
        self.expected_scatter = self.expected_ax.scatter([], [], s=1.0, c="tab:orange", alpha=0.65)
        self.verify_scatter = self.verify_ax.scatter([], [], s=1.0, c="tab:green", alpha=0.65)
        self.overlay_expected_scatter = self.overlay_ax.scatter([], [], s=1.0, c="tab:orange", alpha=0.45, label="Expected")
        self.overlay_verify_scatter = self.overlay_ax.scatter([], [], s=1.0, c="tab:green", alpha=0.45, label="Actual")
        self.overlay_ax.legend(loc="upper right")

        for axis in (self.raw_ax, self.expected_ax, self.verify_ax, self.overlay_ax):
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.grid(True)
            axis.set_aspect("equal", adjustable="box")

        self.status_text = fig.text(0.02, 0.02, "", family="monospace", fontsize=10, va="bottom")
        fig.tight_layout(rect=(0.0, 0.06, 1.0, 1.0))

    def _stamp_to_sec(self, msg: PointCloud2) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _update_rate(self, last_time: Optional[float], current_time: float) -> Optional[float]:
        if last_time is None:
            return None
        dt = current_time - last_time
        if dt <= 0.0:
            return None
        return 1.0 / dt

    def _sample_from_msg(self, msg: PointCloud2) -> CloudSample:
        points = cloud_to_xyz_array(msg)
        return CloudSample(
            stamp_sec=self._stamp_to_sec(msg),
            frame_id=msg.header.frame_id,
            point_count=int(points.shape[0]),
            points=points,
            fields=field_names(msg),
        )

    def _transform_into_base(self, sample: CloudSample, stamp_sec: float) -> Optional[np.ndarray]:
        if sample.frame_id == self.base_frame:
            return sample.points.copy()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                sample.frame_id,
                rclpy.time.Time(seconds=stamp_sec),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.last_compare_summary = (
                f"TF lookup failed for {sample.frame_id} -> {self.base_frame}: {exc}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot = quaternion_to_rotation_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        trans = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        return sample.points @ rot.T + trans

    def _build_expected_sample(self, raw_sample: CloudSample) -> Optional[CloudSample]:
        base_points = self._transform_into_base(raw_sample, raw_sample.stamp_sec)
        if base_points is None:
            return None

        if self.flip_y and base_points.size > 0:
            base_points = base_points.copy()
            base_points[:, 1] *= -1.0

        return CloudSample(
            stamp_sec=raw_sample.stamp_sec,
            frame_id=self.base_frame,
            point_count=int(base_points.shape[0]),
            points=base_points,
            fields=raw_sample.fields,
        )

    def _on_raw_cloud(self, msg: PointCloud2) -> None:
        wall_now = time.time()
        self.raw_rate_hz = self._update_rate(self.last_raw_wall_time, wall_now)
        self.last_raw_wall_time = wall_now
        self.raw_count += 1

        raw_sample = self._sample_from_msg(msg)
        self.latest_raw = raw_sample
        self.latest_expected = self._build_expected_sample(raw_sample)

        expected_frame_ok = self.expected_frame is None or raw_sample.frame_id == self.expected_frame
        timestamp_present = "timestamp" in raw_sample.fields
        mean_range, max_range = radial_summary(raw_sample.points)
        self.last_raw_summary = (
            f"raw#{self.raw_count} frame={raw_sample.frame_id} "
            f"expected_frame_ok={expected_frame_ok} points={raw_sample.point_count} "
            f"fields={','.join(raw_sample.fields)} timestamp_field={timestamp_present} "
            f"xy_mean_range={mean_range:.2f}m xy_max_range={max_range:.2f}m "
            f"bounds={bounds_text(raw_sample.points)}"
        )

        if self.latest_expected is not None:
            self._update_comparison()

    def _on_verify_cloud(self, msg: PointCloud2) -> None:
        wall_now = time.time()
        self.verify_rate_hz = self._update_rate(self.last_verify_wall_time, wall_now)
        self.last_verify_wall_time = wall_now
        self.verify_count += 1

        verify_sample = self._sample_from_msg(msg)
        self.latest_verify = verify_sample
        mean_range, max_range = radial_summary(verify_sample.points)
        self.last_verify_summary = (
            f"verify#{self.verify_count} frame={verify_sample.frame_id} "
            f"points={verify_sample.point_count} xy_mean_range={mean_range:.2f}m "
            f"xy_max_range={max_range:.2f}m bounds={bounds_text(verify_sample.points)}"
        )
        self._update_comparison()

    def _update_comparison(self) -> None:
        if self.latest_expected is None:
            return

        if self.latest_verify is None:
            self.last_compare_summary = (
                "No localization debug cloud yet. Enable localization/debug/enable_pub and "
                "subscribe to /gicp/localization/debug/input_cloud_base."
            )
            return

        dt = abs(self.latest_expected.stamp_sec - self.latest_verify.stamp_sec)
        if dt > self.verify_timeout_sec:
            self.last_compare_summary = (
                f"Latest clouds are too far apart in time: dt={dt:.3f}s "
                f"(threshold={self.verify_timeout_sec:.3f}s)"
            )
            return

        expected_points = self.latest_expected.points
        actual_points = self.latest_verify.points
        min_len = min(len(expected_points), len(actual_points))
        if min_len == 0:
            self.last_compare_summary = "Comparison unavailable because one of the clouds is empty."
            return

        expected_trimmed = expected_points[:min_len]
        actual_trimmed = actual_points[:min_len]
        deltas = expected_trimmed - actual_trimmed
        norms = np.linalg.norm(deltas, axis=1)

        self.last_compare_summary = (
            f"compare dt={dt:.3f}s "
            f"count_expected={len(expected_points)} count_actual={len(actual_points)} "
            f"count_diff={len(expected_points) - len(actual_points)} "
            f"mean_err={float(np.mean(norms)):.4f}m "
            f"p95_err={float(np.percentile(norms, 95)):.4f}m "
            f"max_err={float(np.max(norms)):.4f}m"
        )

    def _set_scatter_points(self, scatter, points: np.ndarray) -> None:
        if points.size == 0:
            scatter.set_offsets(np.empty((0, 2)))
            return
        scatter.set_offsets(points[:, :2])

    def _autoscale_axis(self, axis, point_sets: Sequence[np.ndarray]) -> None:
        valid = [points[:, :2] for points in point_sets if points.size > 0]
        if not valid:
            return

        stacked = np.vstack(valid)
        mins = np.min(stacked, axis=0)
        maxs = np.max(stacked, axis=0)
        span = np.maximum(maxs - mins, 1.0)
        padding = 0.05 * span
        axis.set_xlim(mins[0] - padding[0], maxs[0] + padding[0])
        axis.set_ylim(mins[1] - padding[1], maxs[1] + padding[1])

    def maybe_update_plot(self) -> None:
        raw_points = np.empty((0, 3), dtype=np.float64)
        expected_points = np.empty((0, 3), dtype=np.float64)
        verify_points = np.empty((0, 3), dtype=np.float64)

        if self.latest_raw is not None:
            raw_points = sample_points(self.latest_raw.points, self.max_plot_points)
        if self.latest_expected is not None:
            expected_points = sample_points(self.latest_expected.points, self.max_plot_points)
        if self.latest_verify is not None:
            verify_points = sample_points(self.latest_verify.points, self.max_plot_points)

        self._set_scatter_points(self.raw_scatter, raw_points)
        self._set_scatter_points(self.expected_scatter, expected_points)
        self._set_scatter_points(self.verify_scatter, verify_points)
        self._set_scatter_points(self.overlay_expected_scatter, expected_points)
        self._set_scatter_points(self.overlay_verify_scatter, verify_points)

        self._autoscale_axis(self.raw_ax, [raw_points])
        self._autoscale_axis(self.expected_ax, [expected_points])
        self._autoscale_axis(self.verify_ax, [verify_points])
        self._autoscale_axis(self.overlay_ax, [expected_points, verify_points])

        status_lines = [
            f"raw topic: {self.raw_topic}",
            f"verify topic: {self.verify_topic}",
            f"expected raw frame: {self.expected_frame or '<any>'}",
            f"base frame: {self.base_frame} | flip_y: {self.flip_y}",
            f"raw rate: {self.raw_rate_hz:.2f} Hz" if self.raw_rate_hz is not None else "raw rate: n/a",
            f"verify rate: {self.verify_rate_hz:.2f} Hz" if self.verify_rate_hz is not None else "verify rate: n/a",
            self.last_raw_summary,
            self.last_verify_summary,
            self.last_compare_summary,
        ]
        self.status_text.set_text("\n".join(status_lines))

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize the LiDAR topic used by localization and compare it with the localization debug input cloud."
    )
    parser.add_argument(
        "--topic",
        default="/luminar_front/points",
        help="Raw LiDAR PointCloud2 topic to inspect.",
    )
    parser.add_argument(
        "--verify-topic",
        default="/gicp/localization/debug/input_cloud_base",
        help="Localization debug cloud topic published after TF and flip_y.",
    )
    parser.add_argument(
        "--base-frame",
        default="base_link",
        help="Base frame used by localization before GICP.",
    )
    parser.add_argument(
        "--expected-frame",
        default="luminar_front",
        help="Expected raw cloud frame_id. Use an empty string to disable the frame check.",
    )
    parser.add_argument(
        "--flip-y",
        action="store_true",
        default=True,
        help="Apply the same Y-axis flip configured in cfg/localization.yaml.",
    )
    parser.add_argument(
        "--no-flip-y",
        action="store_false",
        dest="flip_y",
        help="Disable Y-axis flip in the reconstructed localization input.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=30000,
        help="Maximum points to draw per cloud. Full clouds are still used for verification.",
    )
    parser.add_argument(
        "--verify-timeout-sec",
        type=float,
        default=0.20,
        help="Maximum timestamp delta allowed when comparing expected and actual localization inputs.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.expected_frame == "":
        args.expected_frame = None

    rclpy.init()
    node = None
    executor = None

    def handle_signal(signum, frame):
        del signum, frame
        if node is not None:
            node.get_logger().info("Stopping LiDAR topic visualizer.")
            node.finished = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        node = LidarTopicVisualizer(args)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
            node.maybe_update_plot()

        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
