#!/usr/bin/env python3
"""Replay cached GICP-vs-GNSS error results into RViz without rerunning GICP.

This reads the `live_error.csv` written by `run_localization_replay.sh` and
republishes the visualization topics that RViz expects:

- red GNSS path
- green GICP path
- current GICP/GNSS error marker
- 3D error "rollercoaster" curtain
- optional sampled PCD map backdrop

It is intentionally lightweight: no rosbag playback, no localization node, no
GICP, no map preprocessing.
"""

import argparse
import csv
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import colormaps
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Float64, Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def marker_point(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


def load_error_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        required = (
            "stamp", "gicp_x", "gicp_y", "gicp_z",
            "gnss_x", "gnss_y", "gnss_z", "e2d", "e3d",
        )
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            sys.exit(f"{path}: missing columns: {', '.join(missing)}")
        for row in reader:
            try:
                rows.append([float(row[name]) for name in required])
            except (TypeError, ValueError):
                continue
    if len(rows) < 2:
        sys.exit(f"{path}: not enough rows")
    return np.asarray(rows, dtype=np.float64)


def load_pcd_xyz(path, bbox_xy=None, padding=0.0, max_points=250000, seed=7):
    with open(path, "rb") as f:
        header = b""
        while not header.endswith(b"DATA binary\n"):
            line = f.readline()
            if not line:
                sys.exit(f"{path}: unsupported PCD, expected DATA binary")
            header += line
        text = header.decode(errors="replace")
        fields_match = re.search(r"^FIELDS (.+)$", text, re.M)
        points_match = re.search(r"^POINTS (\d+)$", text, re.M)
        if not fields_match or not points_match:
            sys.exit(f"{path}: invalid PCD header")
        fields = fields_match.group(1).split()
        if not {"x", "y", "z"}.issubset(fields):
            sys.exit(f"{path}: PCD must contain x y z fields")
        npts = int(points_match.group(1))
        data = np.fromfile(f, dtype=np.float32, count=npts * len(fields))
    data = data.reshape(-1, len(fields))
    xyz = data[:, [fields.index("x"), fields.index("y"), fields.index("z")]]

    if bbox_xy is not None:
        lo, hi = bbox_xy
        lo = np.asarray(lo, dtype=np.float32) - padding
        hi = np.asarray(hi, dtype=np.float32) + padding
        keep = (
            (xyz[:, 0] >= lo[0]) & (xyz[:, 0] <= hi[0]) &
            (xyz[:, 1] >= lo[1]) & (xyz[:, 1] <= hi[1])
        )
        xyz = xyz[keep]

    if len(xyz) > max_points:
        rng = np.random.default_rng(seed)
        xyz = xyz[rng.choice(len(xyz), size=max_points, replace=False)]
    return xyz.astype(np.float32)


def pose_from_xyz(xyz, frame, stamp):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position = marker_point(*xyz)
    pose.pose.orientation.w = 1.0
    return pose


class CachedErrorViz(Node):
    def __init__(self, args):
        super().__init__("cached_error_viz")
        self.args = args
        self.rows = load_error_csv(args.csv)
        self.index = 0
        self.publish_count = 0
        self.frame = args.frame
        self.cmap = colormaps[args.cmap]
        self.color_max = args.color_max
        if self.color_max <= 0:
            self.color_max = float(np.percentile(self.rows[:, 7], args.color_percentile))
        self.color_max = max(self.color_max, 1e-6)
        self.roller_points = []
        self.last_roller_xy = None

        self.gnss_path = PathMsg()
        self.gnss_path.header.frame_id = self.frame
        self.gicp_path = PathMsg()
        self.gicp_path.header.frame_id = self.frame

        self.gnss_odom_pub = self.create_publisher(Odometry, "/gps_p1/filtered_odom_map", 10)
        self.gicp_odom_pub = self.create_publisher(Odometry, "/gicp/localization/odom", 10)
        self.gnss_path_pub = self.create_publisher(PathMsg, "/gps_p1/filtered_odom_map/path", 10)
        self.gicp_path_pub = self.create_publisher(PathMsg, "/gicp/localization/odom_path_live", 10)
        self.error_pub = self.create_publisher(Float64, "/gicp/localization/debug/gnss_error_m", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/gicp/localization/debug/gnss_error_markers", 10)
        self.roller_pub = self.create_publisher(
            MarkerArray, "/gicp/localization/debug/gnss_error_rollercoaster", 2)
        self.tf_pub = TransformBroadcaster(self)

        self.map_msg = None
        if args.map:
            self.map_msg = self.build_map_msg(args.map)
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.VOLATILE
            self.map_pub = self.create_publisher(PointCloud2, "/gicp/localization/map", qos)
            self.create_timer(args.map_period, self.publish_map)
            self.publish_map()
        else:
            self.map_pub = None

        self.step = max(1, int(np.ceil(len(self.rows) / max(1.0, args.duration * args.rate))))
        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            f"cached RViz replay from {args.csv}: rows={len(self.rows)} step={self.step} "
            f"duration~{len(self.rows) / self.step / args.rate:.1f}s color_max={self.color_max:.2f}m")

    def build_map_msg(self, map_path):
        bbox = (self.rows[:, 4:6].min(axis=0), self.rows[:, 4:6].max(axis=0))
        self.get_logger().info(f"loading sampled map from {map_path}")
        xyz = load_pcd_xyz(
            map_path,
            bbox_xy=bbox,
            padding=self.args.map_padding,
            max_points=self.args.map_max_points,
            seed=self.args.seed,
        )
        header = Header()
        header.frame_id = self.frame
        msg = point_cloud2.create_cloud_xyz32(header, xyz.tolist())
        self.get_logger().info(f"sampled map points for RViz: {len(xyz):,}")
        return msg

    def publish_map(self):
        if self.map_msg is None:
            return
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def color_for_error(self, err, alpha=1.0):
        rgba = self.cmap(float(np.clip(err / self.color_max, 0.0, 1.0)))
        return ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(alpha))

    def tick(self):
        if self.index >= len(self.rows):
            if self.args.loop:
                self.reset()
            elif self.args.hold:
                self.publish_rollercoaster()
                return
            else:
                rclpy.shutdown()
                return

        row = self.rows[self.index]
        self.index += self.step
        stamp = self.get_clock().now().to_msg()
        gicp = row[1:4]
        gnss = row[4:7]
        e2d = float(row[7])
        e3d = float(row[8])

        self.publish_odometry(gicp, gnss, stamp)
        self.publish_paths(gicp, gnss, stamp)
        self.publish_current_error(gicp, gnss, e2d, e3d, stamp)
        self.add_roller_sample(gnss, e2d, e3d)
        self.publish_rollercoaster(stamp)

    def reset(self):
        self.index = 0
        self.roller_points.clear()
        self.last_roller_xy = None
        self.gnss_path.poses.clear()
        self.gicp_path.poses.clear()

    def publish_odometry(self, gicp, gnss, stamp):
        for xyz, pub, child in (
            (gicp, self.gicp_odom_pub, self.args.base_frame),
            (gnss, self.gnss_odom_pub, "gnss_reference"),
        ):
            msg = Odometry()
            msg.header.frame_id = self.frame
            msg.header.stamp = stamp
            msg.child_frame_id = child
            msg.pose.pose.position = marker_point(*xyz)
            msg.pose.pose.orientation.w = 1.0
            pub.publish(msg)

        tf = TransformStamped()
        tf.header.frame_id = self.frame
        tf.header.stamp = stamp
        tf.child_frame_id = self.args.base_frame
        tf.transform.translation.x = float(gicp[0])
        tf.transform.translation.y = float(gicp[1])
        tf.transform.translation.z = float(gicp[2])
        tf.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(tf)

    def publish_paths(self, gicp, gnss, stamp):
        self.publish_count += 1
        if self.publish_count % self.args.path_stride == 0:
            self.gnss_path.header.stamp = stamp
            self.gnss_path.poses.append(pose_from_xyz(gnss, self.frame, stamp))
            self.gnss_path_pub.publish(self.gnss_path)

            self.gicp_path.header.stamp = stamp
            self.gicp_path.poses.append(pose_from_xyz(gicp, self.frame, stamp))
            self.gicp_path_pub.publish(self.gicp_path)

    def base_marker(self, stamp, marker_id, marker_type, action=Marker.ADD):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.ns = "cached_gicp_gnss_error"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = action
        return marker

    def publish_current_error(self, gicp, gnss, e2d, e3d, stamp):
        delete = self.base_marker(stamp, 0, Marker.LINE_STRIP, Marker.DELETEALL)

        line = self.base_marker(stamp, 1, Marker.LINE_STRIP)
        line.scale.x = 0.25
        line.color.r = 1.0
        line.color.g = 0.05
        line.color.b = 0.05
        line.color.a = 1.0
        line.points = [marker_point(*gicp), marker_point(*gnss)]

        text = self.base_marker(stamp, 2, Marker.TEXT_VIEW_FACING)
        text.pose.position = marker_point(gicp[0], gicp[1], gicp[2] + 4.0)
        text.scale.z = 3.0
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"cached GICP-GNSS\n2D {e2d:.2f} m\n3D {e3d:.2f} m"

        sphere = self.base_marker(stamp, 3, Marker.SPHERE)
        sphere.pose.position = marker_point(*gnss)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.5
        sphere.color.r = 0.0
        sphere.color.g = 0.9
        sphere.color.b = 1.0
        sphere.color.a = 1.0

        out = Float64()
        out.data = e2d
        self.error_pub.publish(out)
        self.marker_pub.publish(MarkerArray(markers=[delete, line, text, sphere]))

    def add_roller_sample(self, gnss, e2d, e3d):
        xy = gnss[:2]
        if self.last_roller_xy is not None:
            if float(np.linalg.norm(xy - self.last_roller_xy)) < self.args.rollercoaster_min_step:
                return
        self.last_roller_xy = xy.copy()
        self.roller_points.append((float(gnss[0]), float(gnss[1]), float(gnss[2]), float(e2d), float(e3d)))
        if len(self.roller_points) > self.args.rollercoaster_max_points:
            self.roller_points = self.roller_points[::2]
            last = self.roller_points[-1]
            self.last_roller_xy = np.array([last[0], last[1]], dtype=float)

    def publish_rollercoaster(self, stamp=None):
        if len(self.roller_points) < 2:
            return
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        pts = np.asarray(self.roller_points, dtype=np.float64)
        base = pts[:, :3]
        e2d = pts[:, 3]
        shadow_base = base.copy()
        shadow_base[:, 2] += self.args.rollercoaster_baseline_z
        top = shadow_base.copy()
        top[:, 2] = shadow_base[:, 2] + e2d * self.args.rollercoaster_z_scale

        delete = self.base_marker(stamp, 10, Marker.LINE_STRIP, Marker.DELETEALL)

        curtain = self.base_marker(stamp, 11, Marker.TRIANGLE_LIST)
        curtain_points = []
        curtain_colors = []
        for i in range(len(shadow_base) - 1):
            avg = 0.5 * (e2d[i] + e2d[i + 1])
            color = self.color_for_error(avg, self.args.rollercoaster_curtain_alpha)
            vertices = (
                shadow_base[i], top[i], top[i + 1],
                shadow_base[i], top[i + 1], shadow_base[i + 1],
            )
            curtain_points.extend(marker_point(*v) for v in vertices)
            curtain_colors.extend([color] * 6)
        curtain.points = curtain_points
        curtain.colors = curtain_colors

        shadow = self.base_marker(stamp, 12, Marker.LINE_STRIP)
        shadow.scale.x = 0.35
        shadow.color.r = shadow.color.g = shadow.color.b = 0.95
        shadow.color.a = 0.55
        shadow.points = [marker_point(*p) for p in shadow_base]

        ridge = self.base_marker(stamp, 13, Marker.LINE_STRIP)
        ridge.scale.x = self.args.rollercoaster_line_width
        ridge.points = [marker_point(*p) for p in top]
        ridge.colors = [self.color_for_error(e, 1.0) for e in e2d]

        imax = int(np.argmax(e2d))
        peak = self.base_marker(stamp, 14, Marker.SPHERE)
        peak.pose.position = marker_point(*top[imax])
        peak.scale.x = peak.scale.y = peak.scale.z = max(1.5, min(5.0, e2d[imax] * 0.45))
        peak.color.r = peak.color.g = peak.color.b = peak.color.a = 1.0

        text = self.base_marker(stamp, 15, Marker.TEXT_VIEW_FACING)
        text.pose.position = marker_point(top[imax, 0], top[imax, 1], top[imax, 2] + 3.0)
        text.scale.z = 3.0
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"cached max GICP-GNSS {e2d[imax]:.1f} m"

        self.roller_pub.publish(MarkerArray(markers=[delete, curtain, shadow, ridge, peak, text]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", required=True, help="live_error.csv from a previous replay")
    ap.add_argument("--map", default="", help="optional PCD map backdrop")
    ap.add_argument("--duration", type=float, default=90.0, help="seconds to sweep through the cached run")
    ap.add_argument("--rate", type=float, default=30.0, help="publish ticks per second")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--no-hold", dest="hold", action="store_false", help="exit after one cached sweep")
    ap.set_defaults(hold=True)
    ap.add_argument("--frame", default="map")
    ap.add_argument("--base-frame", default="gps_antenna_top")
    ap.add_argument("--path-stride", type=int, default=1)
    ap.add_argument("--map-padding", type=float, default=90.0)
    ap.add_argument("--map-max-points", type=int, default=250000)
    ap.add_argument("--map-period", type=float, default=1.0)
    ap.add_argument("--rollercoaster-min-step", type=float, default=0.75)
    ap.add_argument("--rollercoaster-max-points", type=int, default=3000)
    ap.add_argument("--rollercoaster-z-scale", type=float, default=10.0)
    ap.add_argument(
        "--rollercoaster-baseline-z",
        type=float,
        default=0.0,
        help="vertical offset added to the physical map z baseline before drawing error height",
    )
    ap.add_argument("--rollercoaster-curtain-alpha", type=float, default=0.28)
    ap.add_argument("--rollercoaster-line-width", type=float, default=0.75)
    ap.add_argument("--cmap", default="inferno")
    ap.add_argument("--color-percentile", type=float, default=98.0)
    ap.add_argument("--color-max", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    rclpy.init()
    node = CachedErrorViz(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
