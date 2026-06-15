#!/usr/bin/env python3
"""Publish live GNSS/GICP trajectory overlays and error diagnostics.

This node is meant to run during scripts/run_localization_replay.sh. It compares
the localization odometry against the GNSS odometry already transformed into the
map frame, publishes a GNSS path for RViz, publishes live error markers, and
periodically writes a PNG error plot.
"""

import signal
from pathlib import Path
from threading import Lock

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import colormaps
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

from error_viz_common import error_color, make_error_cylinders, make_marker, marker_point


ROLLER_NS = "gicp_gnss_error_rollercoaster"


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_stamped_from_odom(msg):
    out = PoseStamped()
    out.header = msg.header
    out.pose = msg.pose.pose
    return out


def point_from_odom(msg):
    p = msg.pose.pose.position
    return np.array([p.x, p.y, p.z], dtype=float)


class LiveGnssErrorMonitor(Node):
    def __init__(self):
        super().__init__("live_gnss_error_monitor")
        self.declare_parameter("est_topic", "/gicp/localization/odom")
        self.declare_parameter("gnss_topic", "/gps_p1/filtered_odom_map")
        self.declare_parameter("gicp_path_topic", "/gicp/localization/odom_path_live")
        self.declare_parameter("gnss_path_topic", "/gps_p1/filtered_odom_map/path")
        self.declare_parameter("error_topic", "/gicp/localization/debug/gnss_error_m")
        self.declare_parameter("marker_topic", "/gicp/localization/debug/gnss_error_markers")
        self.declare_parameter("rollercoaster_topic", "/gicp/localization/debug/gnss_error_rollercoaster")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("plot_path", "")
        self.declare_parameter("max_dt", 0.20)
        self.declare_parameter("gt_max_var_xy", 1e-3)
        self.declare_parameter("path_stride", 3)
        self.declare_parameter("plot_period", 2.0)
        self.declare_parameter("rollercoaster_enable", True)
        self.declare_parameter("rollercoaster_publish_period", 0.5)
        self.declare_parameter("rollercoaster_min_step", 0.75)
        self.declare_parameter("rollercoaster_max_points", 3000)
        self.declare_parameter("rollercoaster_z_scale", 20.0)
        self.declare_parameter("rollercoaster_baseline_z", 0.0)
        self.declare_parameter("rollercoaster_color_max", 3.0)
        self.declare_parameter("rollercoaster_curtain_alpha", 1.0)
        self.declare_parameter("rollercoaster_cylinder_diameter", 1.5)
        self.declare_parameter("rollercoaster_line_width", -1.0)

        self.est_topic = self.get_parameter("est_topic").value
        self.gnss_topic = self.get_parameter("gnss_topic").value
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.gt_max_var_xy = float(self.get_parameter("gt_max_var_xy").value)
        self.path_stride = max(1, int(self.get_parameter("path_stride").value))
        self.plot_period = max(0.25, float(self.get_parameter("plot_period").value))
        self.rollercoaster_enable = bool(self.get_parameter("rollercoaster_enable").value)
        self.rollercoaster_period = max(
            0.1, float(self.get_parameter("rollercoaster_publish_period").value))
        self.rollercoaster_min_step = max(
            0.05, float(self.get_parameter("rollercoaster_min_step").value))
        self.rollercoaster_max_points = max(
            16, int(self.get_parameter("rollercoaster_max_points").value))
        self.rollercoaster_z_scale = max(
            0.01, float(self.get_parameter("rollercoaster_z_scale").value))
        self.rollercoaster_baseline_z = float(
            self.get_parameter("rollercoaster_baseline_z").value)
        self.rollercoaster_color_max = max(
            1e-6, float(self.get_parameter("rollercoaster_color_max").value))
        self.rollercoaster_curtain_alpha = float(
            np.clip(self.get_parameter("rollercoaster_curtain_alpha").value, 0.0, 1.0))
        self.rollercoaster_cylinder_diameter = max(
            0.01, float(self.get_parameter("rollercoaster_cylinder_diameter").value))
        legacy_line_width = float(self.get_parameter("rollercoaster_line_width").value)
        if legacy_line_width > 0.0:
            self.rollercoaster_cylinder_diameter = max(0.01, legacy_line_width)
        self.rollercoaster_cmap = colormaps["inferno"]

        csv = str(self.get_parameter("csv_path").value)
        plot = str(self.get_parameter("plot_path").value)
        self.csv_path = Path(csv) if csv else None
        self.plot_path = Path(plot) if plot else None
        if self.csv_path:
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = self.csv_path.open("w", buffering=1)
            self.csv_file.write("stamp,dt,gicp_x,gicp_y,gicp_z,gnss_x,gnss_y,gnss_z,e2d,e3d\n")
        else:
            self.csv_file = None
        if self.plot_path:
            self.plot_path.parent.mkdir(parents=True, exist_ok=True)

        self.lock = Lock()
        self.latest_gnss = None
        self.gnss_count = 0
        self.gicp_count = 0
        self.rows = []
        self.first_stamp = None
        self.roller_points = []
        self.roller_frame = "map"
        self.roller_stamp = None
        self.last_roller_xy = None

        self.gnss_path = PathMsg()
        self.gnss_path.header.frame_id = "map"
        self.gicp_path = PathMsg()
        self.gicp_path.header.frame_id = "map"

        self.gnss_path_pub = self.create_publisher(
            PathMsg, self.get_parameter("gnss_path_topic").value, 10)
        self.gicp_path_pub = self.create_publisher(
            PathMsg, self.get_parameter("gicp_path_topic").value, 10)
        self.error_pub = self.create_publisher(Float64, self.get_parameter("error_topic").value, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 10)
        self.rollercoaster_pub = self.create_publisher(
            MarkerArray, self.get_parameter("rollercoaster_topic").value, 2)

        self.create_subscription(Odometry, self.gnss_topic, self.on_gnss, 100)
        self.create_subscription(Odometry, self.est_topic, self.on_est, 100)
        self.create_timer(self.plot_period, self.save_plot)
        if self.rollercoaster_enable:
            self.create_timer(self.rollercoaster_period, self.publish_rollercoaster)

        self.get_logger().info(
            f"monitoring est={self.est_topic} vs gnss={self.gnss_topic}; "
            f"csv={self.csv_path or '(disabled)'} plot={self.plot_path or '(disabled)'}")

    def on_gnss(self, msg):
        if self.gt_max_var_xy > 0 and max(msg.pose.covariance[0], msg.pose.covariance[7]) > self.gt_max_var_xy:
            return
        with self.lock:
            self.latest_gnss = msg
            self.gnss_count += 1
            if self.gnss_count % self.path_stride == 0:
                self.gnss_path.header.stamp = msg.header.stamp
                self.gnss_path.header.frame_id = msg.header.frame_id or "map"
                self.gnss_path.poses.append(pose_stamped_from_odom(msg))
                self.gnss_path_pub.publish(self.gnss_path)

    def on_est(self, msg):
        with self.lock:
            self.gicp_count += 1
            if self.gicp_count % self.path_stride == 0:
                self.gicp_path.header.stamp = msg.header.stamp
                self.gicp_path.header.frame_id = msg.header.frame_id or "map"
                self.gicp_path.poses.append(pose_stamped_from_odom(msg))
                self.gicp_path_pub.publish(self.gicp_path)

            gnss = self.latest_gnss
            if gnss is None:
                return
            t_est = stamp_to_sec(msg.header.stamp)
            t_gnss = stamp_to_sec(gnss.header.stamp)
            dt = t_est - t_gnss
            if abs(dt) > self.max_dt:
                return

            p_est = point_from_odom(msg)
            p_gnss = point_from_odom(gnss)
            err = p_est - p_gnss
            e2d = float(np.linalg.norm(err[:2]))
            e3d = float(np.linalg.norm(err))
            if self.first_stamp is None:
                self.first_stamp = t_est
            self.rows.append((t_est, dt, *p_est, *p_gnss, e2d, e3d))
            if self.rollercoaster_enable:
                self.add_rollercoaster_sample(gnss, e2d, e3d)

            out = Float64()
            out.data = e2d
            self.error_pub.publish(out)
            self.marker_pub.publish(self.make_markers(msg, gnss, e2d, e3d, dt))
            if self.csv_file:
                self.csv_file.write(
                    f"{t_est:.9f},{dt:.6f},"
                    f"{p_est[0]:.6f},{p_est[1]:.6f},{p_est[2]:.6f},"
                    f"{p_gnss[0]:.6f},{p_gnss[1]:.6f},{p_gnss[2]:.6f},"
                    f"{e2d:.6f},{e3d:.6f}\n")

    def make_markers(self, est, gnss, e2d, e3d, dt):
        now = est.header.stamp
        frame = est.header.frame_id or "map"
        p_est = est.pose.pose.position
        p_gnss = gnss.pose.pose.position

        delete = Marker()
        delete.header.frame_id = frame
        delete.header.stamp = now
        delete.action = Marker.DELETEALL

        line = Marker()
        line.header.frame_id = frame
        line.header.stamp = now
        line.ns = "gicp_gnss_error"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.25
        line.color.r = 1.0
        line.color.g = 0.05
        line.color.b = 0.05
        line.color.a = 1.0
        line.points = [Point(x=p_est.x, y=p_est.y, z=p_est.z),
                       Point(x=p_gnss.x, y=p_gnss.y, z=p_gnss.z)]

        text = Marker()
        text.header.frame_id = frame
        text.header.stamp = now
        text.ns = "gicp_gnss_error"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = p_est.x
        text.pose.position.y = p_est.y
        text.pose.position.z = p_est.z + 4.0
        text.scale.z = 3.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"GICP-GNSS\nerror: {e2d:.2f} m"

        gnss_sphere = Marker()
        gnss_sphere.header.frame_id = frame
        gnss_sphere.header.stamp = now
        gnss_sphere.ns = "gicp_gnss_error"
        gnss_sphere.id = 2
        gnss_sphere.type = Marker.SPHERE
        gnss_sphere.action = Marker.ADD
        gnss_sphere.pose.position.x = p_gnss.x
        gnss_sphere.pose.position.y = p_gnss.y
        gnss_sphere.pose.position.z = p_gnss.z
        gnss_sphere.scale.x = gnss_sphere.scale.y = gnss_sphere.scale.z = 1.5
        gnss_sphere.color.r = 0.0
        gnss_sphere.color.g = 0.9
        gnss_sphere.color.b = 1.0
        gnss_sphere.color.a = 1.0

        return MarkerArray(markers=[delete, line, text, gnss_sphere])

    def color_for_error(self, err, alpha=1.0):
        return error_color(self.rollercoaster_cmap, err, self.rollercoaster_color_max, alpha)

    def add_rollercoaster_sample(self, gnss, e2d, e3d):
        p = gnss.pose.pose.position
        xy = np.array([p.x, p.y], dtype=float)
        if self.last_roller_xy is not None:
            if float(np.linalg.norm(xy - self.last_roller_xy)) < self.rollercoaster_min_step:
                return
        self.last_roller_xy = xy
        self.roller_frame = gnss.header.frame_id or "map"
        self.roller_stamp = gnss.header.stamp
        self.roller_points.append((float(p.x), float(p.y), float(p.z), float(e2d), float(e3d)))
        if len(self.roller_points) > self.rollercoaster_max_points:
            self.roller_points = self.roller_points[::2]
            last = self.roller_points[-1]
            self.last_roller_xy = np.array([last[0], last[1]], dtype=float)

    def base_marker(self, frame, stamp, marker_id, marker_type, action=Marker.ADD):
        return make_marker(frame, stamp, ROLLER_NS, marker_id, marker_type, action)

    def publish_rollercoaster(self):
        with self.lock:
            if len(self.roller_points) < 2:
                return
            pts = np.asarray(self.roller_points, dtype=float).copy()
            frame = self.roller_frame
            stamp = self.roller_stamp
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        base = pts[:, :3]
        e2d = pts[:, 3]
        shadow_base = base.copy()
        shadow_base[:, 2] += self.rollercoaster_baseline_z
        top = shadow_base.copy()
        top[:, 2] = shadow_base[:, 2] + e2d * self.rollercoaster_z_scale

        delete = self.base_marker(frame, stamp, 0, Marker.LINE_STRIP, Marker.DELETEALL)

        shadow = self.base_marker(frame, stamp, 3, Marker.LINE_STRIP)
        shadow.scale.x = max(0.25, self.rollercoaster_cylinder_diameter * 0.25)
        shadow.color.r = 0.95
        shadow.color.g = 0.95
        shadow.color.b = 0.95
        shadow.color.a = 0.55
        shadow.points = [marker_point(*p) for p in shadow_base]

        cylinders = make_error_cylinders(
            frame,
            stamp,
            ROLLER_NS,
            1000,
            shadow_base,
            top,
            e2d,
            self.rollercoaster_cylinder_diameter,
            self.rollercoaster_curtain_alpha,
            self.color_for_error,
        )

        imax = int(np.argmax(e2d))
        peak = self.base_marker(frame, stamp, 4, Marker.SPHERE)
        peak.pose.position = marker_point(*top[imax])
        peak.scale.x = peak.scale.y = peak.scale.z = max(1.5, min(5.0, e2d[imax] * 0.45))
        peak.color.r = 1.0
        peak.color.g = 1.0
        peak.color.b = 1.0
        peak.color.a = 0.95

        text = self.base_marker(frame, stamp, 5, Marker.TEXT_VIEW_FACING)
        text.pose.position = marker_point(top[imax, 0], top[imax, 1], top[imax, 2] + 3.0)
        text.scale.z = 3.0
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"max GICP-GNSS {e2d[imax]:.1f} m"

        self.rollercoaster_pub.publish(MarkerArray(
            markers=[delete, shadow, *cylinders, peak, text]))

    def save_plot(self):
        if not self.plot_path:
            return
        with self.lock:
            if len(self.rows) < 2:
                return
            arr = np.array(self.rows, dtype=float)
        t = arr[:, 0] - arr[0, 0]
        e2d = arr[:, -2]
        e3d = arr[:, -1]
        fig, ax = plt.subplots(figsize=(10, 4.8), dpi=130)
        ax.plot(t, e2d, label="horizontal error", color="#d62728", linewidth=1.5)
        ax.plot(t, e3d, label="3D error", color="#1f77b4", linewidth=1.2, alpha=0.85)
        ax.set_xlabel("replay time [s]")
        ax.set_ylabel("GICP - GNSS error [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
        ax.set_title(
            f"Live localization error  median 2D={np.median(e2d):.2f} m, "
            f"p95 2D={np.percentile(e2d, 95):.2f} m")
        fig.tight_layout()
        fig.savefig(self.plot_path)
        plt.close(fig)

    def destroy_node(self):
        try:
            self.save_plot()
        finally:
            if self.csv_file:
                self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LiveGnssErrorMonitor()

    def stop(_signum, _frame):
        node.get_logger().info("signal received; saving live error plot")
        node.save_plot()
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
