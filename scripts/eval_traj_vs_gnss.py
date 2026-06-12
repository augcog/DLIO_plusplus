#!/usr/bin/env python3
"""Quantify GLIM mapping trajectory error against RTK GNSS.

Reads a GLIM dump (traj_imu.txt — the IMU/antenna pose in the map frame — and
T_world_utm.txt) plus the prepped bag's /gps_p1/filtered_odom (UTM frame),
time-interpolates GNSS to each trajectory stamp, and reports the position
error statistics of

    err(t) = p_map_glim(t) - T_world_utm * p_utm_gnss(t)

Both GLIM's traj_imu and the Atlas INS solution live at the antenna phase
centre (gps_antenna_top), so this is an apples-to-apples comparison.

Run from a local ROS 2 Jazzy shell:
    python3 scripts/eval_traj_vs_gnss.py --dump ./dlio_data/run_5_dump --bag ./dlio_data/run_5_prepped
"""

import argparse
import os
import sys

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def load_tum(path):
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    return data  # stamp x y z qx qy qz qw


def load_t_world_utm(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "T_world_utm" in line:
                continue
            vals = line.split()
            if len(vals) == 4:
                rows.append([float(v) for v in vals])
    if len(rows) != 4:
        raise RuntimeError(f"malformed {path}")
    return np.array(rows)


def read_odom(bag_path, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, Odometry)
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
    if not rows:
        sys.exit(f"no messages on {topic} in {bag_path}")
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True, help="GLIM dump dir (traj_imu.txt, T_world_utm.txt)")
    ap.add_argument("--bag", required=True, help="prepped bag with /gps_p1/filtered_odom (utm frame)")
    ap.add_argument("--gnss-topic", default="/gps_p1/filtered_odom_rtk_fixed",
                    help="RTK-gated reference by default; the ungated topic includes INS coasting")
    ap.add_argument("--max-gap", type=float, default=0.5,
                    help="drop comparisons where the bracketing GNSS samples are further apart "
                         "than this (seconds) — avoids interpolating across RTK dropouts")
    ap.add_argument("--csv", default="", help="optional output CSV of per-sample errors")
    args = ap.parse_args()

    traj = load_tum(os.path.join(args.dump, "traj_imu.txt"))
    T = load_t_world_utm(os.path.join(args.dump, "T_world_utm.txt"))
    R, t = T[:3, :3], T[:3, 3]
    gnss = read_odom(args.bag, args.gnss_topic)
    gnss_map = (R @ gnss[:, 1:4].T).T + t  # GNSS track in map frame

    # interpolate GNSS to trajectory stamps (inside the overlapping window)
    t0, t1 = gnss[0, 0], gnss[-1, 0]
    sel = (traj[:, 0] >= t0) & (traj[:, 0] <= t1)
    if sel.sum() < 10:
        sys.exit("trajectory and GNSS stamps barely overlap — check inputs")
    ts = traj[sel, 0]
    p_glim = traj[sel, 1:4]
    p_gnss = np.column_stack([np.interp(ts, gnss[:, 0], gnss_map[:, i]) for i in range(3)])

    # drop samples whose bracketing GNSS gap exceeds --max-gap (RTK dropouts)
    idx = np.searchsorted(gnss[:, 0], ts).clip(1, len(gnss) - 1)
    gap = gnss[idx, 0] - gnss[idx - 1, 0]
    ok = gap <= args.max_gap
    if (~ok).any():
        print(f"dropping {(~ok).sum()} samples inside GNSS gaps > {args.max_gap}s")
    ts, p_glim, p_gnss = ts[ok], p_glim[ok], p_gnss[ok]
    if len(ts) < 10:
        sys.exit("too few samples after gap filtering")

    err = p_glim - p_gnss
    e2d = np.linalg.norm(err[:, :2], axis=1)
    e3d = np.linalg.norm(err, axis=1)

    def stats(e):
        return (f"rms={np.sqrt(np.mean(e ** 2)):.3f}  median={np.median(e):.3f}  "
                f"p95={np.percentile(e, 95):.3f}  max={np.max(e):.3f}")

    print(f"frames compared: {len(ts)}  (window {ts[0]:.1f} .. {ts[-1]:.1f}, {ts[-1] - ts[0]:.1f}s)")
    print(f"horizontal error [m]: {stats(e2d)}")
    print(f"3D error         [m]: {stats(e3d)}")
    print(f"z error          [m]: rms={np.sqrt(np.mean(err[:, 2] ** 2)):.3f}  "
          f"mean={np.mean(err[:, 2]):.3f}")

    if args.csv:
        np.savetxt(args.csv, np.column_stack([ts, err, e2d, e3d]),
                   header="stamp ex ey ez e2d e3d", comments="")
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
