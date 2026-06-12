#!/usr/bin/env python3
"""Prepare a raw AV-24 recording for the DLIO++ mapping/localization pipeline.

The raw bags (e.g. <run>/filtered/all) carry the Point One Atlas driver's
native topics, which none of the pipeline configs subscribe to:

    /atlas/imu_calibrated   sensor_msgs/msg/Imu        (frame_id "pointonenav",
                                                        arrival-time stamped, bursty)
    /atlas/pose_filtered    fusion_engine_msgs/msg/Pose (LLA + ENU rpy[deg] +
                                                        body-FLU velocity)
    /luminar_*/points       sensor_msgs/msg/PointCloud2 (clean sensor-clocked stamps)

This script writes a new rosbag2 (mcap) directory containing:

    /luminar_front|left|right/points   byte-for-byte passthrough
    /gps_p1/imu                        re-stamped IMU, frame_id "gps_antenna_top"
    /gps_p1/filtered_odom              nav_msgs/Odometry in the fixed "utm" frame
    /gps_p1/filtered_odom_rtk_fixed    same, gated to RTK-FIXED quality only

IMU re-stamping: the Atlas driver stamps messages with arrival time. Samples
arrive in bursts (median gap 1.6 ms at a nominal ~100 Hz; occasionally the
driver buffers >10 s and flushes everything at once). Header stamps are
upper bounds of the true sample times, so a backward-min filter
    t[i] = min(arrival[i], t[i+1] - T)
recovers a uniform sampling grid (T estimated from the data). Without this,
GTSAM IMU preintegration weights every sample with a wrong dt and the mapping
trajectory degrades badly.

Odometry conversion: latitude/longitude/altitude -> UTM via pyproj using a
FIXED zone (auto-derived from the first fix, printable/overridable) so that
maps and bags from different sessions at the same track share one world
frame. A fixed local origin (auto: first fix rounded down to a 10 km grid;
override with --utm-origin to match a previous session) is subtracted so
coordinates stay small enough for float32 consumers (PCD export, viewers);
the origin is written to <output>/utm_origin.txt. P1 rpy is measured ENU yaw
(validated against UTM velocity heading: median error 0.13 deg on run_5), so
orientation = Rz(yaw)Ry(pitch)Rx(roll). Twist is left in the body (FLU =
child) frame per ROS convention.

Odometry re-stamping: the pose stream's header stamps are arrival times with
the same burst pathology as the IMU (measured: median gap 1.75 ms at a 10 ms
fix interval, plus the multi-second buffer flushes). Unlike the IMU, the
fusion_engine Pose message carries p1_time (the solution's true time of
validity on the P1 monotonic clock), so stamps are rebuilt exactly as
p1_time + OFF, where OFF is the lower envelope of (arrival - p1_time)
(arrival can only be late, never early).

Run from a local ROS 2 Jazzy shell. Dependencies:
    pip install --user --break-system-packages mcap mcap-ros2-support pyproj numpy

Usage:
    python3 scripts/prep_bag.py \
        --input  "../rosbags/putnam/may_26/run_5/filtered/all" \
        --output "./dlio_data/run_5_prepped" \
        [--utm-zone 16] [--rtk-max-var-xy 1e-3] [--rtk-max-var-z 5e-3]
"""

import argparse
import glob
import math
import os
import sys

import numpy as np

try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError:
    sys.exit("missing python deps: pip install --user --break-system-packages mcap mcap-ros2-support")

try:
    from pyproj import Proj
except ImportError:
    sys.exit("missing python dep: pip install --user --break-system-packages pyproj")

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

LIDAR_TOPICS = ["/luminar_front/points", "/luminar_left/points", "/luminar_right/points"]
IMU_IN = "/atlas/imu_calibrated"
POSE_IN = "/atlas/pose_filtered"
IMU_OUT = "/gps_p1/imu"
ODOM_OUT = "/gps_p1/filtered_odom"
ODOM_RTK_OUT = "/gps_p1/filtered_odom_rtk_fixed"
OUT_FRAME = "utm"
BODY_FRAME = "gps_antenna_top"
SOLUTION_TYPE_RTK_FIXED = 4  # fusion_engine SolutionType enum


def input_mcaps(path):
    if os.path.isfile(path) and path.endswith(".mcap"):
        return [path]
    files = glob.glob(os.path.join(path, "*.mcap"))
    if not files:
        sys.exit(f"no .mcap files under {path}")

    def key(p):
        stem = os.path.splitext(os.path.basename(p))[0]
        tail = stem.rsplit("_", 1)[-1]
        return (0, int(tail)) if tail.isdigit() else (1, stem)

    return sorted(files, key=key)


def collect_stamps(files):
    """Pass 1: IMU arrival stamps + pose (arrival, p1_time) pairs."""
    imu_stamps = []
    pose_pairs = []
    for path in files:
        with open(path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, channel, _, msg in reader.iter_decoded_messages(topics=[IMU_IN, POSE_IN]):
                if channel.topic == IMU_IN:
                    imu_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                else:
                    pose_pairs.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                       msg.p1_time.seconds + msg.p1_time.fraction_ns * 1e-9))
    return np.asarray(imu_stamps), np.asarray(pose_pairs)


def fit_p1_clock_offset(pose_pairs):
    """ROS time of validity = p1_time + OFF. Arrival is always late, so OFF is
    the lower envelope of (arrival - p1_time), tracked per minute to absorb
    slow clock drift between the P1 clock and the recorder clock."""
    arrival, p1 = pose_pairs[:, 0], pose_pairs[:, 1]
    lag = arrival - p1
    bins = np.floor((p1 - p1[0]) / 60.0).astype(int)
    env_t, env_off = [], []
    for b in np.unique(bins):
        sel = bins == b
        env_t.append(np.median(p1[sel]))
        env_off.append(np.percentile(lag[sel], 1))
    env_t, env_off = np.asarray(env_t), np.asarray(env_off)
    drift_ms = (env_off.max() - env_off.min()) * 1e3
    print(f"[prep_bag] P1->ROS clock offset: median={np.median(env_off):.6f}s "
          f"envelope drift across run={drift_ms:.2f} ms")
    if drift_ms < 5.0:
        const = float(np.median(env_off))
        return lambda t: t + const
    print("[prep_bag] WARNING: P1->ROS offset drifts; using piecewise-linear envelope")
    return lambda t: t + np.interp(t, env_t, env_off)


def dejitter(stamps, nominal_period=0.01):
    """Backward-min de-jitter of arrival-stamped samples onto a uniform grid."""
    if len(stamps) < 100:
        return stamps.copy()
    period = nominal_period
    for _ in range(2):  # refine the period estimate once
        dt = np.diff(stamps)
        sane = dt[(dt > 0.5 * period) & (dt < 1.5 * period)]
        if len(sane) > 100:
            period = float(np.median(sane))
    out = stamps.copy()
    for i in range(len(out) - 2, -1, -1):
        ceil = out[i + 1] - period
        if out[i] > ceil:
            out[i] = ceil
    # guard: enforce strictly increasing in pathological cases
    for i in range(1, len(out)):
        if out[i] <= out[i - 1]:
            out[i] = out[i - 1] + 1e-6
    print(f"[prep_bag] IMU de-jitter: n={len(out)} period={period * 1e3:.3f} ms "
          f"max_shift={np.max(stamps - out) * 1e3:.1f} ms")
    return out


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    """ENU-frame intrinsic ZYX (yaw CCW-from-East) -> quaternion (x, y, z, w)."""
    r = math.radians(roll_deg) / 2.0
    p = math.radians(pitch_deg) / 2.0
    y = math.radians(yaw_deg) / 2.0
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def to_ros_time(t):
    ns = int(round(t * 1e9))
    return ns // 1_000_000_000, ns % 1_000_000_000


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True, help="bag dir (or single .mcap) with /atlas + /luminar topics")
    ap.add_argument("--output", required=True, help="output rosbag2 directory (must not exist)")
    ap.add_argument("--utm-zone", type=int, default=0, help="UTM zone; 0 = auto from first fix")
    ap.add_argument("--utm-origin", default="",
                    help="'E,N' meters subtracted from UTM coords. Default: first fix rounded "
                         "down to a 10 km grid. Pass the utm_origin.txt values of a previous "
                         "session to share its frame.")
    ap.add_argument("--rtk-max-var-xy", type=float, default=1e-3, help="m^2 gate for the rtk_fixed topic")
    ap.add_argument("--rtk-max-var-z", type=float, default=5e-3, help="m^2 gate for the rtk_fixed topic")
    args = ap.parse_args()

    if os.path.exists(args.output):
        sys.exit(f"output already exists: {args.output}")

    files = input_mcaps(args.input)
    print(f"[prep_bag] {len(files)} input mcap file(s)")

    print("[prep_bag] pass 1/2: collecting IMU/pose stamps ...")
    imu_stamps, pose_pairs = collect_stamps(files)
    if len(imu_stamps) == 0:
        sys.exit(f"no messages on {IMU_IN} — wrong input bag?")
    if len(pose_pairs) == 0:
        sys.exit(f"no messages on {POSE_IN} — wrong input bag?")
    new_imu_stamps = dejitter(imu_stamps)
    p1_to_ros = fit_p1_clock_offset(pose_pairs)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_id = 0
    for topic, msg_type in (
        [(t, "sensor_msgs/msg/PointCloud2") for t in LIDAR_TOPICS]
        + [(IMU_OUT, "sensor_msgs/msg/Imu")]
        + [(ODOM_OUT, "nav_msgs/msg/Odometry"), (ODOM_RTK_OUT, "nav_msgs/msg/Odometry")]
    ):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=topic_id, name=topic, type=msg_type, serialization_format="cdr"))
        topic_id += 1

    proj = None
    utm_zone = args.utm_zone
    utm_origin = None
    if args.utm_origin:
        utm_origin = tuple(float(v) for v in args.utm_origin.split(","))
    imu_idx = 0
    counts = {"lidar": 0, "imu": 0, "odom": 0, "rtk": 0, "rtk_rejected": 0}
    deg2rad_sq = (math.pi / 180.0) ** 2

    print("[prep_bag] pass 2/2: writing output bag ...")
    wanted = LIDAR_TOPICS + [IMU_IN, POSE_IN]
    decoder_factory = DecoderFactory()
    for fidx, path in enumerate(files):
        print(f"[prep_bag]   file {fidx + 1}/{len(files)}: {os.path.basename(path)}")
        decoders = {}  # schema id -> decode fn; ids are file-local, reset per file
        with open(path, "rb") as f:
            reader = make_reader(f)
            for schema, channel, message in reader.iter_messages(topics=wanted):
                topic = channel.topic
                if topic in LIDAR_TOPICS:
                    # passthrough: mcap payload is already CDR — do NOT decode
                    writer.write(topic, message.data, message.log_time)
                    counts["lidar"] += 1
                    continue
                decode = decoders.get(schema.id)
                if decode is None:
                    decode = decoder_factory.decoder_for("cdr", schema)
                    decoders[schema.id] = decode
                msg = decode(message.data)
                if topic == IMU_IN:
                    t = new_imu_stamps[imu_idx]
                    imu_idx += 1
                    out = Imu()
                    out.header.stamp.sec, out.header.stamp.nanosec = to_ros_time(t)
                    out.header.frame_id = BODY_FRAME
                    out.orientation.x = msg.orientation.x
                    out.orientation.y = msg.orientation.y
                    out.orientation.z = msg.orientation.z
                    out.orientation.w = msg.orientation.w
                    out.angular_velocity.x = msg.angular_velocity.x
                    out.angular_velocity.y = msg.angular_velocity.y
                    out.angular_velocity.z = msg.angular_velocity.z
                    out.linear_acceleration.x = msg.linear_acceleration.x
                    out.linear_acceleration.y = msg.linear_acceleration.y
                    out.linear_acceleration.z = msg.linear_acceleration.z
                    writer.write(IMU_OUT, serialize_message(out), int(t * 1e9))
                    counts["imu"] += 1
                elif topic == POSE_IN:
                    if proj is None:
                        if utm_zone == 0:
                            utm_zone = int((msg.longitude + 180.0) // 6.0) + 1
                        proj = Proj(proj="utm", zone=utm_zone, ellps="WGS84",
                                    south=msg.latitude < 0.0)
                        e0, n0 = proj(msg.longitude, msg.latitude)
                        if utm_origin is None:
                            utm_origin = (math.floor(e0 / 10000.0) * 10000.0,
                                          math.floor(n0 / 10000.0) * 10000.0)
                        print(f"[prep_bag] UTM zone {utm_zone}{'S' if msg.latitude < 0 else 'N'} "
                              f"(first fix lat={msg.latitude:.6f} lon={msg.longitude:.6f})")
                        print(f"[prep_bag] UTM local origin: {utm_origin[0]:.1f},{utm_origin[1]:.1f} "
                              f"(reuse via --utm-origin for other sessions at this track)")
                        with open(os.path.join(args.output, "utm_origin.txt"), "w") as of:
                            of.write(f"# UTM zone {utm_zone}; subtract this origin from raw UTM\n"
                                     f"{utm_origin[0]:.3f},{utm_origin[1]:.3f}\n")
                    e, n = proj(msg.longitude, msg.latitude)
                    t_valid = p1_to_ros(msg.p1_time.seconds + msg.p1_time.fraction_ns * 1e-9)
                    out = Odometry()
                    out.header.stamp.sec, out.header.stamp.nanosec = to_ros_time(t_valid)
                    out.header.frame_id = OUT_FRAME
                    out.child_frame_id = BODY_FRAME
                    out.pose.pose.position.x = e - utm_origin[0]
                    out.pose.pose.position.y = n - utm_origin[1]
                    out.pose.pose.position.z = msg.altitude
                    qx, qy, qz, qw = rpy_to_quat(msg.rpy.roll, msg.rpy.pitch, msg.rpy.yaw)
                    out.pose.pose.orientation.x = qx
                    out.pose.pose.orientation.y = qy
                    out.pose.pose.orientation.z = qz
                    out.pose.pose.orientation.w = qw
                    pc = msg.position_covariance  # ENU 3x3 row-major
                    rc = msg.rpy_covariance       # deg^2 3x3 row-major
                    cov = out.pose.covariance
                    cov[0], cov[7], cov[14] = float(pc[0]), float(pc[4]), float(pc[8])
                    cov[21] = float(rc[0]) * deg2rad_sq
                    cov[28] = float(rc[4]) * deg2rad_sq
                    cov[35] = float(rc[8]) * deg2rad_sq
                    out.twist.twist.linear.x = msg.velflu.x
                    out.twist.twist.linear.y = msg.velflu.y
                    out.twist.twist.linear.z = msg.velflu.z
                    vc = msg.velflu_covariance
                    tcov = out.twist.covariance
                    tcov[0], tcov[7], tcov[14] = float(vc[0]), float(vc[4]), float(vc[8])
                    data = serialize_message(out)
                    stamp_ns = int(round(t_valid * 1e9))
                    writer.write(ODOM_OUT, data, stamp_ns)
                    counts["odom"] += 1
                    rtk_ok = (msg.solution_type == SOLUTION_TYPE_RTK_FIXED
                              and pc[0] <= args.rtk_max_var_xy
                              and pc[4] <= args.rtk_max_var_xy
                              and pc[8] <= args.rtk_max_var_z)
                    if rtk_ok:
                        writer.write(ODOM_RTK_OUT, data, stamp_ns)
                        counts["rtk"] += 1
                    else:
                        counts["rtk_rejected"] += 1
    del writer  # flush + write metadata.yaml

    print(f"[prep_bag] done: {counts['lidar']} lidar, {counts['imu']} imu, "
          f"{counts['odom']} odom ({counts['rtk']} rtk-fixed, {counts['rtk_rejected']} rejected)")
    print(f"[prep_bag] output: {args.output}")
    if counts["rtk"] == 0:
        print("[prep_bag] WARNING: no RTK-FIXED samples passed the gate — GLIM will map "
              "without GNSS anchoring and no T_world_utm.txt will be produced.")


if __name__ == "__main__":
    main()
