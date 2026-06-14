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

    /luminar_front|left|right/points   passthrough by default; optionally shifted
                                       by --lidar-time-offset
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

LiDAR time alignment: LiDAR PointCloud2 header stamps are left unchanged by
default and are treated as the scan reference time on the ROS/INS time axis.
Per-point Luminar timestamps are not used as an absolute INS-time source here;
they are only checked as an intra-scan clock (timestamp_i - min_timestamp).
The script prints a LiDAR-vs-odom timing report and writes
time_alignment_report.txt. If an external calibration finds a fixed LiDAR-INS
time delay, pass --lidar-time-offset SECONDS to add that offset to LiDAR
PointCloud2 header.stamp and rosbag log_time. Point-level timestamps are left
untouched because GLIM uses them only through their per-scan relative offsets.

Run from a local ROS 2 Jazzy shell. Dependencies:
    pip install --user --break-system-packages mcap mcap-ros2-support pyproj numpy

Usage:
    python3 scripts/prep_bag.py \
        --input  "../rosbags/putnam/may_26/run_5/filtered/all" \
        --output "./dlio_data/run_5_prepped" \
        [--utm-zone 16] [--rtk-max-var-xy 1e-3] [--rtk-max-var-z 5e-3]
"""

import argparse
import bisect
import glob
import math
import os
import struct
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
TIME_FIELD_NAMES = ("t", "time", "time_stamp", "timestamp")

# sensor_msgs/msg/PointField constants. Importing PointField directly is not
# needed and keeps this script compatible with mcap's dynamic message classes.
PF_UINT8 = 2
PF_UINT32 = 6
PF_FLOAT32 = 7
PF_FLOAT64 = 8


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


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def set_stamp(stamp, t):
    stamp.sec, stamp.nanosec = to_ros_time(t)


def nearest_abs_delta(stamps, t):
    if len(stamps) == 0:
        return None
    idx = bisect.bisect_left(stamps, t)
    candidates = []
    if idx < len(stamps):
        candidates.append(abs(stamps[idx] - t))
    if idx > 0:
        candidates.append(abs(stamps[idx - 1] - t))
    return min(candidates) if candidates else None


def point_time_field(msg):
    for f in msg.fields:
        if f.name in TIME_FIELD_NAMES:
            return f
    return None


def unpack_point_time(data, offset, datatype, count):
    if datatype == PF_UINT8 and count == 8:
        return struct.unpack_from("<Q", data, offset)[0] * 1e-9
    if datatype == PF_UINT32:
        return struct.unpack_from("<I", data, offset)[0] * 1e-9
    if datatype == PF_FLOAT32:
        return float(struct.unpack_from("<f", data, offset)[0])
    if datatype == PF_FLOAT64:
        return float(struct.unpack_from("<d", data, offset)[0])
    return None


def point_time_minmax(msg):
    field = point_time_field(msg)
    if field is None:
        return None
    count = int(msg.width) * int(msg.height)
    if count == 0:
        return None

    tmin = None
    tmax = None
    offset = int(field.offset)
    point_step = int(msg.point_step)
    data = msg.data
    for i in range(count):
        t = unpack_point_time(data, i * point_step + offset, field.datatype, int(field.count))
        if t is None:
            return None
        if tmin is None or t < tmin:
            tmin = t
        if tmax is None or t > tmax:
            tmax = t

    return {
        "name": field.name,
        "datatype": int(field.datatype),
        "count": int(field.count),
        "min": tmin,
        "max": tmax,
        "span": max(0.0, tmax - tmin),
        "absolute_like": tmax > 1.0,
    }


def ms_summary(values):
    if not values:
        return "n=0"
    arr = np.asarray(values, dtype=float) * 1e3
    return (f"n={len(arr)} median={np.median(arr):.3f} ms "
            f"p95={np.percentile(arr, 95):.3f} ms "
            f"min={arr.min():.3f} ms max={arr.max():.3f} ms")


def signed_ms_summary(values):
    if not values:
        return "n=0"
    arr = np.asarray(values, dtype=float) * 1e3
    return (f"n={len(arr)} median={np.median(arr):.3f} ms "
            f"min={arr.min():.3f} ms max={arr.max():.3f} ms "
            f"range={(arr.max() - arr.min()):.3f} ms")


def init_lidar_time_diagnostics(p1_to_ros, pose_pairs, samples_per_topic,
                                lidar_time_offset, nearest_warn_ms,
                                clock_drift_warn_ms):
    return {
        "samples_per_topic": samples_per_topic,
        "lidar_time_offset": lidar_time_offset,
        "nearest_warn_ms": nearest_warn_ms,
        "clock_drift_warn_ms": clock_drift_warn_ms,
        "odom_stamps": sorted(float(x) for x in p1_to_ros(pose_pairs[:, 1])),
        "topics": {
            topic: {
                "samples": 0,
                "missing_time": 0,
                "unsupported_time": 0,
                "header_log": [],
                "scan_span": [],
                "header_min_point": [],
                "header_dt": [],
                "point_min_dt": [],
                "nearest_odom_start": [],
                "nearest_odom_end": [],
                "last_header": None,
                "last_point_min": None,
                "field_desc": None,
            }
            for topic in LIDAR_TOPICS
        },
    }


def update_lidar_time_diagnostics(diag, topic, msg, log_time_ns):
    samples_per_topic = diag["samples_per_topic"]
    if samples_per_topic <= 0 or topic not in diag["topics"]:
        return

    st = diag["topics"][topic]
    if st["samples"] >= samples_per_topic:
        return

    header_t = stamp_to_sec(msg.header.stamp)
    log_t = log_time_ns * 1e-9
    st["samples"] += 1
    st["header_log"].append(log_t - header_t)

    if st["last_header"] is not None:
        st["header_dt"].append(header_t - st["last_header"])
    st["last_header"] = header_t

    nearest_start = nearest_abs_delta(diag["odom_stamps"], header_t)
    if nearest_start is not None:
        st["nearest_odom_start"].append(nearest_start)

    point_stats = point_time_minmax(msg)
    if point_stats is None:
        if point_time_field(msg) is None:
            st["missing_time"] += 1
        else:
            st["unsupported_time"] += 1
        return

    st["field_desc"] = (f"{point_stats['name']} datatype={point_stats['datatype']} "
                        f"count={point_stats['count']}")
    st["scan_span"].append(point_stats["span"])
    scan_end_t = header_t + point_stats["span"]
    nearest_end = nearest_abs_delta(diag["odom_stamps"], scan_end_t)
    if nearest_end is not None:
        st["nearest_odom_end"].append(nearest_end)
    if point_stats["absolute_like"]:
        st["header_min_point"].append(header_t - point_stats["min"])
        if st["last_point_min"] is not None:
            st["point_min_dt"].append(point_stats["min"] - st["last_point_min"])
        st["last_point_min"] = point_stats["min"]


def format_lidar_time_diagnostics(diag):
    """Report LiDAR scan reference stamps against the re-stamped INS/odom axis.

    This intentionally does not estimate physical LiDAR-INS latency from motion.
    Timestamps can prove that the bag's clocks are mutually usable, but a
    constant sensor latency needs external calibration or a trajectory residual
    sweep.  --lidar-time-offset is the explicit hook for applying that result.
    """
    lines = []
    lidar_time_offset = diag["lidar_time_offset"]
    nearest_warn_ms = diag["nearest_warn_ms"]
    clock_drift_warn_ms = diag["clock_drift_warn_ms"]

    lines.append("[prep_bag] LiDAR/INS time alignment report")
    lines.append(f"[prep_bag]   applied lidar_time_offset={lidar_time_offset:+.9f}s "
                 "(positive moves LiDAR later on the ROS/INS axis)")

    if diag["samples_per_topic"] <= 0:
        lines.append("[prep_bag]   skipped (--lidar-time-check-samples <= 0)")
        return lines

    for topic, st in diag["topics"].items():
        lines.append(f"[prep_bag]   {topic}: samples={st['samples']}")
        if st["field_desc"]:
            lines.append(f"[prep_bag]     point time field: {st['field_desc']}")
        if st["missing_time"]:
            lines.append(f"[prep_bag]     WARNING: {st['missing_time']} sampled clouds had no per-point time field")
        if st["unsupported_time"]:
            lines.append(f"[prep_bag]     WARNING: {st['unsupported_time']} sampled clouds had an unsupported time field")
        lines.append(f"[prep_bag]     bag_log_time - header.stamp: {signed_ms_summary(st['header_log'])}")
        lines.append(f"[prep_bag]     LiDAR header dt: {ms_summary(st['header_dt'])}")
        if st["scan_span"]:
            lines.append(f"[prep_bag]     LiDAR scan span from per-point time: {ms_summary(st['scan_span'])}")
        if st["header_min_point"]:
            lines.append(f"[prep_bag]     header.stamp - min(point_time): {signed_ms_summary(st['header_min_point'])}")
            spread_ms = (max(st["header_min_point"]) - min(st["header_min_point"])) * 1e3
            if spread_ms > clock_drift_warn_ms:
                lines.append(f"[prep_bag]     WARNING: header-to-point clock offset spread {spread_ms:.3f} ms "
                             f"> {clock_drift_warn_ms:.3f} ms")
        if st["point_min_dt"]:
            lines.append(f"[prep_bag]     point-min dt: {ms_summary(st['point_min_dt'])}")
        lines.append(f"[prep_bag]     nearest odom to scan start: {ms_summary(st['nearest_odom_start'])}")
        if st["nearest_odom_end"]:
            lines.append(f"[prep_bag]     nearest odom to scan end: {ms_summary(st['nearest_odom_end'])}")

        all_nearest = st["nearest_odom_start"] + st["nearest_odom_end"]
        if all_nearest and max(all_nearest) * 1e3 > nearest_warn_ms:
            lines.append(f"[prep_bag]     WARNING: LiDAR scan window is farther than "
                         f"{nearest_warn_ms:.1f} ms from nearest odom sample")

    lines.append("[prep_bag]   NOTE: this check validates timestamp-axis compatibility. "
                 "It cannot infer true physical LiDAR-INS latency from stamps alone; "
                 "apply measured latency with --lidar-time-offset.")
    return lines


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
    ap.add_argument("--lidar-time-offset", type=float, default=0.0,
                    help="Seconds added to /luminar_* PointCloud2 header.stamp and bag log_time. "
                         "Positive moves LiDAR later on the ROS/INS time axis. Point-level "
                         "timestamps are left untouched.")
    ap.add_argument("--lidar-time-check-samples", type=int, default=200,
                    help="Number of LiDAR clouds per topic to sample for the timing report; "
                         "set 0 to skip.")
    ap.add_argument("--lidar-odom-warn-ms", type=float, default=20.0,
                    help="Warn when scan start/end is farther than this from the nearest "
                         "re-stamped odom sample.")
    ap.add_argument("--lidar-clock-drift-warn-ms", type=float, default=2.0,
                    help="Warn when header.stamp - min(point timestamp) varies by more than "
                         "this across sampled LiDAR clouds.")
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
    lidar_diag = init_lidar_time_diagnostics(
        p1_to_ros, pose_pairs,
        args.lidar_time_check_samples,
        args.lidar_time_offset,
        args.lidar_odom_warn_ms,
        args.lidar_clock_drift_warn_ms,
    )
    if abs(args.lidar_time_offset) > 1e-12:
        print(f"[prep_bag] applying fixed LiDAR time offset {args.lidar_time_offset:+.9f}s "
              "to PointCloud2 header.stamp and bag log_time")

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
    lidar_offset_ns = int(round(args.lidar_time_offset * 1e9))

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
                    need_lidar_diag = (
                        args.lidar_time_check_samples > 0
                        and lidar_diag["topics"][topic]["samples"] < args.lidar_time_check_samples
                    )
                    lidar_msg = None
                    if lidar_offset_ns != 0 or need_lidar_diag:
                        decode = decoders.get(schema.id)
                        if decode is None:
                            decode = decoder_factory.decoder_for("cdr", schema)
                            decoders[schema.id] = decode
                        lidar_msg = decode(message.data)

                    if lidar_offset_ns == 0:
                        if need_lidar_diag:
                            update_lidar_time_diagnostics(lidar_diag, topic, lidar_msg, message.log_time)
                        # passthrough: mcap payload is already CDR — do NOT decode
                        writer.write(topic, message.data, message.log_time)
                    else:
                        shifted_stamp = stamp_to_sec(lidar_msg.header.stamp) + args.lidar_time_offset
                        if shifted_stamp < 0.0:
                            sys.exit(f"LiDAR time offset shifts {topic} header.stamp negative")
                        set_stamp(lidar_msg.header.stamp, shifted_stamp)
                        shifted_log_time = message.log_time + lidar_offset_ns
                        if shifted_log_time < 0:
                            sys.exit(f"LiDAR time offset shifts {topic} bag log_time negative")
                        if need_lidar_diag:
                            update_lidar_time_diagnostics(lidar_diag, topic, lidar_msg, shifted_log_time)
                        writer.write(topic, serialize_message(lidar_msg), shifted_log_time)
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

    time_report_lines = format_lidar_time_diagnostics(lidar_diag)
    with open(os.path.join(args.output, "time_alignment_report.txt"), "w") as tf:
        tf.write("\n".join(time_report_lines))
        tf.write("\n")
    for line in time_report_lines:
        print(line)

    print(f"[prep_bag] done: {counts['lidar']} lidar, {counts['imu']} imu, "
          f"{counts['odom']} odom ({counts['rtk']} rtk-fixed, {counts['rtk_rejected']} rejected)")
    print(f"[prep_bag] output: {args.output}")
    if counts["rtk"] == 0:
        print("[prep_bag] WARNING: no RTK-FIXED samples passed the gate — GLIM will map "
              "without GNSS anchoring and no T_world_utm.txt will be produced.")


if __name__ == "__main__":
    main()
