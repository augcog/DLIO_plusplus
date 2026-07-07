#!/usr/bin/env python3
"""
Live inspector for GICP debug poses.

Subscribes to the initial-guess and final pose debug topics, prints matched
samples, writes them to CSV, and optionally plots the guess relative to the
final pose over time.
"""

import argparse
import csv
import math
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


def wrap_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0 and angle_deg > 0:
        return 180.0
    return wrapped


def quaternion_to_rpy_deg(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def quaternion_relative_angle_deg(q_from: Tuple[float, float, float, float],
                                  q_to: Tuple[float, float, float, float]) -> float:
    x1, y1, z1, w1 = q_from
    x2, y2, z2, w2 = q_to

    qc = (-x1, -y1, -z1, w1)
    rx = qc[3] * x2 + qc[0] * w2 + qc[1] * z2 - qc[2] * y2
    ry = qc[3] * y2 - qc[0] * z2 + qc[1] * w2 + qc[2] * x2
    rz = qc[3] * z2 + qc[0] * y2 - qc[1] * x2 + qc[2] * w2
    rw = qc[3] * w2 - qc[0] * x2 - qc[1] * y2 - qc[2] * z2

    norm = math.sqrt(rx * rx + ry * ry + rz * rz + rw * rw)
    if norm == 0.0:
        return float("nan")

    rw /= norm
    angle = 2.0 * math.acos(max(-1.0, min(1.0, abs(rw))))
    return math.degrees(angle)


@dataclass
class PoseSample:
    stamp: float
    position: Tuple[float, float, float]
    rpy_deg: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]


class DebugPoseInspector(Node):
    def __init__(self, csv_path: Path, time_tolerance_ms: float, max_samples: int, enable_plot: bool):
        super().__init__("debug_pose_inspector")

        self.csv_path = csv_path
        self.tolerance_sec = time_tolerance_ms / 1000.0
        self.max_samples = max_samples
        self.enable_plot = enable_plot

        self.pending_guess: Deque[PoseSample] = deque()
        self.pending_final: Deque[PoseSample] = deque()
        self.latest_converged: Optional[bool] = None
        self.latest_fitness: Optional[float] = None

        self.rows_written = 0
        self.finished = False

        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(
            self._csv_file,
            fieldnames=[
                "stamp",
                "current_x", "current_y", "current_z",
                "current_roll_deg", "current_pitch_deg", "current_yaw_deg",
                "guess_x", "guess_y", "guess_z",
                "guess_roll_deg", "guess_pitch_deg", "guess_yaw_deg",
                "delta_x", "delta_y", "delta_z",
                "delta_trans_norm",
                "delta_roll_deg", "delta_pitch_deg", "delta_yaw_deg",
                "delta_rot_deg",
                "converged",
                "fitness",
            ],
        )
        self._csv_writer.writeheader()
        self._csv_file.flush()

        self.current_traj_xy: List[Tuple[float, float]] = []
        self.guess_traj_xy: List[Tuple[float, float]] = []
        self.delta_trans_series: List[Tuple[float, float]] = []
        self.delta_rot_series: List[Tuple[float, float]] = []
        self.recent_pairs: Deque[Tuple[Tuple[float, float], Tuple[float, float]]] = deque(maxlen=30)

        self._plot_last_update = 0.0
        self._plot_objects: Dict[str, object] = {}
        if self.enable_plot:
            self._setup_plot()

        self.create_subscription(
            PoseStamped,
            "/gicp/localization/debug/initial_guess_pose",
            self._on_guess_pose,
            50,
        )
        self.create_subscription(
            PoseStamped,
            "/gicp/localization/debug/final_pose",
            self._on_final_pose,
            50,
        )
        self.create_subscription(
            Bool,
            "/gicp/localization/debug/converged",
            self._on_converged,
            50,
        )
        self.create_subscription(
            Float64,
            "/gicp/localization/debug/fitness",
            self._on_fitness,
            50,
        )

        self.get_logger().info(f"Writing CSV to {self.csv_path}")
        self.get_logger().info(
            "Waiting for /gicp/localization/debug/initial_guess_pose and /gicp/localization/debug/final_pose"
        )

    def destroy_node(self) -> bool:
        try:
            self._csv_file.flush()
            self._csv_file.close()
        finally:
            return super().destroy_node()

    def _setup_plot(self) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError(
                "matplotlib is required for plotting. Install python3-matplotlib or rerun with --no-plot."
            ) from exc

        plt.ion()
        fig, (traj_ax, delta_ax, rot_ax) = plt.subplots(3, 1, figsize=(11, 12))
        fig.suptitle("GICP Initial Guess vs Final Pose")

        current_line, = traj_ax.plot([], [], color="tab:blue", linewidth=2.0, label="Final Pose XY")
        guess_line, = traj_ax.plot([], [], color="tab:orange", linestyle="--", linewidth=1.5, label="Initial Guess XY")
        current_point, = traj_ax.plot([], [], "o", color="tab:blue", markersize=8)
        guess_point, = traj_ax.plot([], [], "o", color="tab:orange", markersize=8)

        traj_ax.set_title("Top-Down XY")
        traj_ax.set_xlabel("x [m]")
        traj_ax.set_ylabel("y [m]")
        traj_ax.grid(True)
        traj_ax.axis("equal")
        traj_ax.legend(loc="best")

        delta_line, = delta_ax.plot([], [], color="tab:green", linewidth=2.0)
        delta_ax.set_title("Initial Guess Translation Error")
        delta_ax.set_xlabel("stamp [s]")
        delta_ax.set_ylabel("||guess - current|| [m]")
        delta_ax.grid(True)

        rot_line, = rot_ax.plot([], [], color="tab:red", linewidth=2.0)
        rot_ax.set_title("Initial Guess Rotation Error")
        rot_ax.set_xlabel("stamp [s]")
        rot_ax.set_ylabel("relative angle [deg]")
        rot_ax.grid(True)

        self._plot_objects = {
            "plt": plt,
            "fig": fig,
            "traj_ax": traj_ax,
            "delta_ax": delta_ax,
            "rot_ax": rot_ax,
            "current_line": current_line,
            "guess_line": guess_line,
            "current_point": current_point,
            "guess_point": guess_point,
            "delta_line": delta_line,
            "rot_line": rot_line,
            "pair_lines": [],
        }

    def _on_converged(self, msg: Bool) -> None:
        self.latest_converged = msg.data

    def _on_fitness(self, msg: Float64) -> None:
        self.latest_fitness = msg.data

    def _on_guess_pose(self, msg: PoseStamped) -> None:
        self.pending_guess.append(self._pose_from_msg(msg))
        self._attempt_matches()

    def _on_final_pose(self, msg: PoseStamped) -> None:
        self.pending_final.append(self._pose_from_msg(msg))
        self._attempt_matches()

    def _pose_from_msg(self, msg: PoseStamped) -> PoseSample:
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        return PoseSample(
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            position=(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
            rpy_deg=quaternion_to_rpy_deg(qx, qy, qz, qw),
            quaternion=(qx, qy, qz, qw),
        )

    def _attempt_matches(self) -> None:
        while self.pending_guess and self.pending_final:
            guess = self.pending_guess[0]

            best_index = None
            best_dt = None
            for index, final_sample in enumerate(self.pending_final):
                dt = abs(final_sample.stamp - guess.stamp)
                if best_dt is None or dt < best_dt:
                    best_dt = dt
                    best_index = index
                if final_sample.stamp > guess.stamp + self.tolerance_sec:
                    break

            if best_dt is not None and best_dt <= self.tolerance_sec and best_index is not None:
                matched_guess = self.pending_guess.popleft()
                matched_final = self.pending_final[best_index]
                del self.pending_final[best_index]
                self._process_pair(matched_guess, matched_final)
                continue

            earliest_final = self.pending_final[0].stamp
            if guess.stamp + self.tolerance_sec < earliest_final:
                self.get_logger().warn(
                    f"Dropping unmatched initial guess sample at {guess.stamp:.3f}s; no final pose within tolerance."
                )
                self.pending_guess.popleft()
                continue

            earliest_guess = self.pending_guess[0].stamp
            final_sample = self.pending_final[0]
            if final_sample.stamp + self.tolerance_sec < earliest_guess:
                self.get_logger().warn(
                    f"Dropping unmatched final pose sample at {final_sample.stamp:.3f}s; no guess pose within tolerance."
                )
                self.pending_final.popleft()
                continue

            break

    def _process_pair(self, guess: PoseSample, current: PoseSample) -> None:
        delta_x = guess.position[0] - current.position[0]
        delta_y = guess.position[1] - current.position[1]
        delta_z = guess.position[2] - current.position[2]
        delta_trans_norm = math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)

        delta_roll = wrap_angle_deg(guess.rpy_deg[0] - current.rpy_deg[0])
        delta_pitch = wrap_angle_deg(guess.rpy_deg[1] - current.rpy_deg[1])
        delta_yaw = wrap_angle_deg(guess.rpy_deg[2] - current.rpy_deg[2])
        delta_rot_deg = quaternion_relative_angle_deg(current.quaternion, guess.quaternion)

        row = {
            "stamp": f"{current.stamp:.6f}",
            "current_x": f"{current.position[0]:.6f}",
            "current_y": f"{current.position[1]:.6f}",
            "current_z": f"{current.position[2]:.6f}",
            "current_roll_deg": f"{current.rpy_deg[0]:.3f}",
            "current_pitch_deg": f"{current.rpy_deg[1]:.3f}",
            "current_yaw_deg": f"{current.rpy_deg[2]:.3f}",
            "guess_x": f"{guess.position[0]:.6f}",
            "guess_y": f"{guess.position[1]:.6f}",
            "guess_z": f"{guess.position[2]:.6f}",
            "guess_roll_deg": f"{guess.rpy_deg[0]:.3f}",
            "guess_pitch_deg": f"{guess.rpy_deg[1]:.3f}",
            "guess_yaw_deg": f"{guess.rpy_deg[2]:.3f}",
            "delta_x": f"{delta_x:.6f}",
            "delta_y": f"{delta_y:.6f}",
            "delta_z": f"{delta_z:.6f}",
            "delta_trans_norm": f"{delta_trans_norm:.6f}",
            "delta_roll_deg": f"{delta_roll:.3f}",
            "delta_pitch_deg": f"{delta_pitch:.3f}",
            "delta_yaw_deg": f"{delta_yaw:.3f}",
            "delta_rot_deg": f"{delta_rot_deg:.3f}",
            "converged": "" if self.latest_converged is None else str(self.latest_converged).lower(),
            "fitness": "" if self.latest_fitness is None else f"{self.latest_fitness:.6f}",
        }

        self._csv_writer.writerow(row)
        self._csv_file.flush()
        self.rows_written += 1

        print(
            f"[{self.rows_written:05d}] t={current.stamp:.3f} "
            f"current xyz=({current.position[0]:.2f},{current.position[1]:.2f},{current.position[2]:.2f}) "
            f"rpy=({current.rpy_deg[0]:.1f},{current.rpy_deg[1]:.1f},{current.rpy_deg[2]:.1f}) "
            f"| guess xyz=({guess.position[0]:.2f},{guess.position[1]:.2f},{guess.position[2]:.2f}) "
            f"rpy=({guess.rpy_deg[0]:.1f},{guess.rpy_deg[1]:.1f},{guess.rpy_deg[2]:.1f}) "
            f"| delta xyz=({delta_x:.2f},{delta_y:.2f},{delta_z:.2f}) "
            f"trans={delta_trans_norm:.2f}m rot={delta_rot_deg:.2f}deg "
            f"| converged={row['converged'] or 'na'} fitness={row['fitness'] or 'na'}",
            flush=True,
        )

        self.current_traj_xy.append((current.position[0], current.position[1]))
        self.guess_traj_xy.append((guess.position[0], guess.position[1]))
        self.delta_trans_series.append((current.stamp, delta_trans_norm))
        self.delta_rot_series.append((current.stamp, delta_rot_deg))
        self.recent_pairs.append(
            ((current.position[0], current.position[1]), (guess.position[0], guess.position[1]))
        )

        if self.max_samples > 0 and self.rows_written >= self.max_samples:
            self.get_logger().info(f"Reached max_samples={self.max_samples}; exiting.")
            self.finished = True

    def maybe_update_plot(self) -> None:
        if not self.enable_plot or not self._plot_objects:
            return

        now = time.time()
        if now - self._plot_last_update < 0.2:
            return
        self._plot_last_update = now

        plt = self._plot_objects["plt"]
        traj_ax = self._plot_objects["traj_ax"]
        delta_ax = self._plot_objects["delta_ax"]
        rot_ax = self._plot_objects["rot_ax"]

        current_line = self._plot_objects["current_line"]
        guess_line = self._plot_objects["guess_line"]
        current_point = self._plot_objects["current_point"]
        guess_point = self._plot_objects["guess_point"]
        delta_line = self._plot_objects["delta_line"]
        rot_line = self._plot_objects["rot_line"]

        pair_lines = self._plot_objects["pair_lines"]
        for line in pair_lines:
            line.remove()
        pair_lines.clear()

        if self.current_traj_xy:
            xs = [xy[0] for xy in self.current_traj_xy]
            ys = [xy[1] for xy in self.current_traj_xy]
            current_line.set_data(xs, ys)
            current_point.set_data([xs[-1]], [ys[-1]])
        else:
            current_line.set_data([], [])
            current_point.set_data([], [])

        if self.guess_traj_xy:
            xs = [xy[0] for xy in self.guess_traj_xy]
            ys = [xy[1] for xy in self.guess_traj_xy]
            guess_line.set_data(xs, ys)
            guess_point.set_data([xs[-1]], [ys[-1]])
        else:
            guess_line.set_data([], [])
            guess_point.set_data([], [])

        for current_xy, guess_xy in self.recent_pairs:
            pair_line, = traj_ax.plot(
                [current_xy[0], guess_xy[0]],
                [current_xy[1], guess_xy[1]],
                color="0.75",
                linewidth=1.0,
                alpha=0.7,
            )
            pair_lines.append(pair_line)

        if self.delta_trans_series:
            ts = [item[0] for item in self.delta_trans_series]
            values = [item[1] for item in self.delta_trans_series]
            delta_line.set_data(ts, values)
        else:
            delta_line.set_data([], [])

        if self.delta_rot_series:
            ts = [item[0] for item in self.delta_rot_series]
            values = [item[1] for item in self.delta_rot_series]
            rot_line.set_data(ts, values)
        else:
            rot_line.set_data([], [])

        traj_ax.relim()
        traj_ax.autoscale_view()
        delta_ax.relim()
        delta_ax.autoscale_view()
        rot_ax.relim()
        rot_ax.autoscale_view()

        self._plot_objects["fig"].canvas.draw_idle()
        plt.pause(0.001)


def default_csv_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"/tmp/gicp_pose_debug_{timestamp}.csv")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect GICP debug pose topics and export pose deltas to CSV.")
    parser.add_argument(
        "--csv-path",
        default=str(default_csv_path()),
        help="Output CSV path. Default: /tmp/gicp_pose_debug_<timestamp>.csv",
    )
    parser.add_argument(
        "--time-tolerance-ms",
        type=float,
        default=20.0,
        help="Maximum allowed timestamp difference when matching guess and final pose samples.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable matplotlib plots and run in console/CSV mode only.",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=0,
        help="Stop after N matched samples. Use 0 to run until interrupted.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    csv_path = Path(args.csv_path).expanduser().resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    inspector = None
    executor = None

    def handle_signal(signum, frame):
        del signum, frame
        if inspector is not None:
            inspector.get_logger().info("Stopping debug pose inspector.")
            inspector.finished = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        inspector = DebugPoseInspector(
            csv_path=csv_path,
            time_tolerance_ms=args.time_tolerance_ms,
            max_samples=args.max_samples,
            enable_plot=not args.no_plot,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(inspector)

        while rclpy.ok() and not inspector.finished:
            executor.spin_once(timeout_sec=0.1)
            inspector.maybe_update_plot()

        return 0
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        if executor is not None:
            executor.shutdown()
        if inspector is not None:
            inspector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
