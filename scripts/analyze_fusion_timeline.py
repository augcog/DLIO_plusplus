#!/usr/bin/env python3
"""Unified-timeline joint analysis of localization error vs sensor status.

Aligns the localizer's estimated trajectory with the RTK/INS reference on one
time axis and overlays, per instant: position error, LiDAR matching quality
(GICP fitness / acceptance / rejection reason / processing time), IMU
propagation burden (age since the last non-IMU correction, dead-reckoning
streaks), GNSS/RTK quality (reported pose sigma, RTK-FIXED gating state,
gate drops), and the node's decision events (accept / reject->dead-reckon /
snap-to-GT / deferred snap). High-error segments are detected, attributed to
causes, and reported with recovery mechanisms.

Inputs (produced by scripts/run_localization_replay.sh):
  --loc-dir       dir containing loc_eval/ (mcap bag) and localization.log
  --prepped-bag   the prepped input bag (RTK-FIXED gating stream, IMU stream)
  --out           output directory (default <loc-dir>/fusion_analysis)

Run INSIDE the ros2-jazzy distrobox (needs rosbag2_py + matplotlib):
  distrobox enter ros2-jazzy -- bash -lc '
    source /opt/ros/jazzy/setup.bash &&
    python3 scripts/analyze_fusion_timeline.py \
        --loc-dir /home/dongc1/dlio_data/run_5_loc \
        --prepped-bag /home/dongc1/dlio_data/run_5_prepped \
        --name run_5'

Outputs: timeline_<name>.png (the unified multi-panel figure),
track_map_<name>.png, distributions_<name>.png, segments_<name>/seg*.png,
error_<name>.csv, scans_<name>.csv, segments_<name>.csv, report_<name>.md.
"""

import argparse
import os
import re
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool

C_ACC, C_DR, C_SNAP, C_NOSCAN = "tab:green", "tab:orange", "tab:blue", "0.82"
GATE_VAR_XY = 0.25   # node rtk_gate/max_pose_var_xy (m^2) -> sigma 0.5 m
SCAN_PERIOD = 0.05   # Luminar front period (s)


# --------------------------------------------------------------------------
# bag reading
# --------------------------------------------------------------------------

def detect_storage(bag_dir):
    meta = os.path.join(bag_dir, "metadata.yaml")
    if os.path.exists(meta):
        for line in open(meta):
            if "storage_identifier" in line:
                return line.split(":")[1].strip()
    return "mcap"


def read_bag(bag_dir, want):
    """want: {topic: kind}; kind in odom|pose|f64|bool.
    Returns {topic: np.ndarray} (see column comments below)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=detect_storage(bag_dir)),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    avail = {t.name for t in reader.get_all_topics_and_types()}
    want = {t: k for t, k in want.items() if t in avail}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(want)))
    rows = {t: [] for t in want}
    while reader.has_next():
        topic, data, log_t = reader.read_next()
        kind = want.get(topic)
        if kind is None:
            continue
        recv = log_t * 1e-9
        if kind == "odom":   # t x y z covxx covyy covzz speed recv
            m = deserialize_message(data, Odometry)
            p, c, tw = m.pose.pose.position, m.pose.covariance, m.twist.twist.linear
            rows[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                p.x, p.y, p.z, c[0], c[7], c[14],
                                (tw.x**2 + tw.y**2 + tw.z**2) ** 0.5, recv))
        elif kind == "pose":  # t x y z recv
            m = deserialize_message(data, PoseStamped)
            p = m.pose.position
            rows[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                p.x, p.y, p.z, recv))
        elif kind == "f64":   # recv value
            m = deserialize_message(data, Float64)
            rows[topic].append((recv, m.data))
        elif kind == "bool":
            m = deserialize_message(data, Bool)
            rows[topic].append((recv, float(m.data)))
    return {t: np.array(v) for t, v in rows.items() if v}


def read_stamps_only(bag_dir, topics):
    """Header stamps for Odometry/Imu topics (cheap-ish full deserialize)."""
    from sensor_msgs.msg import Imu
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=detect_storage(bag_dir)),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in out:
            continue
        cls = Imu if "Imu" in types[topic] else Odometry
        m = deserialize_message(data, cls)
        out[topic].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return {t: np.array(v) for t, v in out.items()}


# --------------------------------------------------------------------------
# localization.log parsing
# --------------------------------------------------------------------------

WALL_RE = re.compile(r"\[(?:WARN|INFO|ERROR|DEBUG)\] \[(\d+\.\d+)\]")
NUM = r"([-\d.eE+na]+)"
F_RE = {k: re.compile(p) for k, p in {
    "stamp": r"\bstamp=(\d+\.\d+)",
    "status": r"\bstatus=(\w+)",
    "fitness": r"\bfitness=" + NUM,
    "gicp_ms": r"\bgicp_ms=" + NUM,
    "ratio": r"\bratio=" + NUM,
    "hessian": r"\bhessian_cond=" + NUM,
    "raw": r"\braw=(\d+)",
    "pre": r"\bpre=(\d+)",
    "imu_lag": r"scan_to_latest_imu_lag=" + NUM + "s",
    "g2s": r"guess_to_solution=\[" + NUM + r"m,",
    "g2s_rot": r"guess_to_solution=\[[^,]+," + NUM + r"deg\]",
    "gt_et": r"gt_err=\[" + NUM + "m",
}.items()}


def parse_log(path):
    scans, snaps, defers, gates, events = [], [], [], [], []
    if not os.path.exists(path):
        print(f"warning: no localization.log at {path}", file=sys.stderr)
        return scans, snaps, defers, gates, events
    last_scan_wall, last_scan_stamp = None, None
    for line in open(path, errors="replace"):
        mw = WALL_RE.search(line)
        if not mw:
            continue
        wall = float(mw.group(1))
        if "SCAN DEBUG |" in line:
            d = {"wall": wall}
            for key, rx in F_RE.items():
                m = rx.search(line)
                if m:
                    try:
                        d[key] = float(m.group(1)) if key != "status" else m.group(1)
                    except ValueError:
                        pass
            if "stamp" in d:
                scans.append(d)
                last_scan_wall, last_scan_stamp = wall, d["stamp"]
        elif "snapped pose to GT" in line:
            m = re.search(r"\(.* after (\d+) consecutive", line)
            stamp = last_scan_stamp if (last_scan_wall and wall - last_scan_wall < 0.5) else None
            snaps.append({"wall": wall, "stamp": stamp,
                          "streak": int(m.group(1)) if m else -1})
        elif "deferring snap" in line:
            reason = ("gt_stale" if "no GT sample within" in line
                      else "no_gt_yet" if "no GT odom received" in line else "tf")
            stamp = last_scan_stamp if (last_scan_wall and wall - last_scan_wall < 0.5) else None
            defers.append({"wall": wall, "stamp": stamp, "reason": reason})
        elif "RTK gate: dropping gt_odom" in line:
            gates.append(wall)
        elif "Geometric observer initialized" in line or "RTK-quality init" in line \
                or "IMU calibration complete" in line:
            events.append((wall, line.rsplit("]:", 1)[-1].strip()[:70]))
    return scans, snaps, defers, gates, events


# --------------------------------------------------------------------------
# timeline assembly
# --------------------------------------------------------------------------

def build(args):
    eval_bag = os.path.join(args.loc_dir, "loc_eval")
    dbg = "/gicp/localization/debug/"
    data = read_bag(eval_bag, {
        args.est_topic: "odom", args.gt_topic: "odom",
        "/gicp/localization/pose": "pose",
        dbg + "fitness": "f64", dbg + "gicp_elapsed_ms": "f64",
        dbg + "hessian_condition_proxy": "f64", dbg + "correspondence_ratio": "f64",
        dbg + "num_correspondences": "f64", dbg + "converged": "bool",
        dbg + "imu_age": "f64", dbg + "scan_dt": "f64",
        "/gicp/localization/gt_snap": "pose",
    })
    if args.est_topic not in data or args.gt_topic not in data:
        sys.exit(f"est/gt topics missing from {eval_bag}")
    est, gt = data[args.est_topic], data[args.gt_topic]
    pose = data.get("/gicp/localization/pose")
    if pose is None:
        sys.exit("no /gicp/localization/pose in eval bag (needed as the per-scan record)")

    T = {}
    T["t0"] = gt[0, 0]

    def rel(t):
        return np.asarray(t) - T["t0"]

    # ---- denial windows (annotation; seconds relative to bag start) ----
    denial = []
    if args.denial_windows:
        for w in args.denial_windows.split(","):
            a, b = w.split(":")
            denial.append((float(a), float(b)))
    T["denial"] = denial

    def in_denial(tr):
        m = np.zeros(np.shape(tr), bool)
        for a, b in denial:
            m |= (tr >= a) & (tr <= b)
        return m

    # ---- error series (est vs interpolated GT) ----
    # GT samples used as reference truth: optionally gated by reported
    # covariance, except inside synthetic-denial windows (covariance there is
    # artificially inflated by the bridge while positions stay RTK-true).
    g_ok = np.ones(len(gt), bool)
    if args.gt_var_gate > 0:
        g_ok = np.maximum(gt[:, 4], gt[:, 5]) <= args.gt_var_gate
        g_ok |= in_denial(rel(gt[:, 0]))
    gtu = gt[g_ok]
    ts = est[:, 0]
    sel = (ts >= gtu[0, 0]) & (ts <= gtu[-1, 0])
    ts, p_est = ts[sel], est[sel, 1:4]
    p_gt = np.column_stack([np.interp(ts, gtu[:, 0], gtu[:, i]) for i in (1, 2, 3)])
    idx = np.searchsorted(gtu[:, 0], ts).clip(1, len(gtu) - 1)
    valid = (gtu[idx, 0] - gtu[idx - 1, 0]) <= args.max_gt_gap
    err = p_est - p_gt
    e2d = np.linalg.norm(err[:, :2], axis=1)
    # along-track / cross-track split using the GT path tangent
    tang = np.gradient(p_gt[:, :2], axis=0)
    n = np.linalg.norm(tang, axis=1)
    tang[n > 1e-6] /= n[n > 1e-6, None]
    e_along = np.einsum("ij,ij->i", err[:, :2], tang)
    e_cross = err[:, 0] * -tang[:, 1] + err[:, 1] * tang[:, 0]
    T["err"] = dict(t=rel(ts), e2d=e2d, ez=err[:, 2], valid=valid,
                    along=e_along, cross=e_cross,
                    speed=np.interp(ts, gt[:, 0], gt[:, 7]),
                    x=p_est[:, 0], y=p_est[:, 1],
                    gx=p_gt[:, 0], gy=p_gt[:, 1])

    # ---- wall<->bag clock offset (lower envelope over GT messages; the
    # bridge hop is sub-ms, so min(recv-stamp) over a window ~= clock offset)
    off_raw = gt[:, 8] - gt[:, 0]
    k = max(1, len(gt) // 400)
    env_t, env_v = gt[::k, 0], np.array([off_raw[max(0, i - 200):i + 200].min()
                                         for i in range(0, len(gt), k)])

    # ---- per-scan table ----
    st = pose[:, 0]
    scan = dict(t=rel(st), stamp=st, recv=pose[:, 4])
    # age of the scan's result when its pose was published = end-to-end
    # processing latency for that scan (queue wait + preprocess + GICP)
    scan["result_age_ms"] = (pose[:, 4] - np.interp(st, env_t, env_v) - st) * 1e3
    for key, topic in [("fitness", dbg + "fitness"), ("ms", dbg + "gicp_elapsed_ms"),
                       ("hessian", dbg + "hessian_condition_proxy"),
                       ("ratio", dbg + "correspondence_ratio"),
                       ("ncorr", dbg + "num_correspondences"),
                       ("converged", dbg + "converged"), ("imu_age", dbg + "imu_age")]:
        arr = data.get(topic)
        if arr is None:
            scan[key] = np.full(len(st), np.nan)
        elif len(arr) == len(st):                    # same per-scan cadence
            scan[key] = arr[:, 1]
        else:                                        # nearest-receive fallback
            j = np.searchsorted(arr[:, 0], pose[:, 4]).clip(1, len(arr) - 1)
            j -= (pose[:, 4] - arr[j - 1, 0]) < (arr[j, 0] - pose[:, 4])
            v = arr[j, 1]
            v[np.abs(arr[j, 0] - pose[:, 4]) > 0.04] = np.nan
            scan[key] = v

    # merge the log records (status + extras) by millisecond-rounded stamp
    scans_log, snaps, defers, gates, events = parse_log(
        os.path.join(args.loc_dir, "localization.log"))
    by_ms = {round(d["stamp"] * 1000): d for d in scans_log}
    status = np.array(["accepted"] * len(st), dtype=object)
    log_hess = np.full(len(st), np.nan)
    log_g2s = np.full(len(st), np.nan)
    log_g2s_rot = np.full(len(st), np.nan)
    log_imu_lag = np.full(len(st), np.nan)
    log_pre = np.full(len(st), np.nan)
    for i, s in enumerate(st):
        d = by_ms.get(round(s * 1000))
        if d:
            status[i] = d.get("status", "accepted")
            log_hess[i] = d.get("hessian", np.nan)
            log_g2s[i] = d.get("g2s", np.nan)
            log_g2s_rot[i] = d.get("g2s_rot", np.nan)
            log_imu_lag[i] = d.get("imu_lag", np.nan)
            log_pre[i] = d.get("pre", np.nan)
    if np.all(np.isnan(scan["hessian"])):
        scan["hessian"] = log_hess
    scan["g2s"], scan["g2s_rot"] = log_g2s, log_g2s_rot
    scan["imu_lag"], scan["pre"] = log_imu_lag, log_pre
    accepted = ~np.isin(status, ["rejected_hessian", "rejected_fitness", "rejected_jump",
                                 "rejected_gt_veto", "failed_to_converge", "invalid_solution"])
    scan["status"], scan["accepted"] = status, accepted

    # snap / defer events -> scan stamps
    def ev_stamps(evs):
        out = []
        wall2bag_w, wall2bag_b = pose[:, 4], pose[:, 0]
        for e in evs:
            if e.get("stamp") is not None:
                out.append(e["stamp"])
            else:
                out.append(np.interp(e["wall"], wall2bag_w, wall2bag_b))
        return np.array(out) if out else np.empty(0)

    snap_t, defer_t = ev_stamps(snaps), ev_stamps(defers)
    gs = data.get("/gicp/localization/gt_snap")
    if gs is not None and len(gs) > len(snap_t):     # exact topic if recorded
        snap_t = gs[:, 0]
    snapped = np.isin(np.round(st * 1000), np.round(snap_t * 1000))
    scan["snapped"] = snapped
    T["snap_t"], T["defer_t"] = rel(snap_t), rel(defer_t)
    T["defer_reasons"] = [d["reason"] for d in defers]
    T["gate_t"] = rel(np.interp(np.array(gates), pose[:, 4], pose[:, 0])) if gates else np.empty(0)

    # consecutive-failure streak + IMU correction age
    streak, c = np.zeros(len(st), int), 0
    for i in range(len(st)):
        c = 0 if (accepted[i] or snapped[i]) else c + 1
        streak[i] = c
    scan["streak"] = streak
    corr_t = st[accepted | snapped]
    j = np.searchsorted(corr_t, ts, side="right") - 1
    age = np.where(j >= 0, ts - corr_t[j.clip(0)], ts - ts[0])
    T["err"]["corr_age"] = age
    scan["dt_proc"] = np.diff(st, prepend=st[0] - SCAN_PERIOD)
    T["scan"] = scan

    # per-error-sample decision state: 0 noscan, 1 accept, 2 DR, 3 snap
    j = np.searchsorted(st, ts, side="right") - 1
    state = np.zeros(len(ts), int)
    has = j >= 0
    jc = j.clip(0)
    near = has & (ts - st[jc] <= 0.6)
    state[near & accepted[jc]] = 1
    state[near & ~accepted[jc] & ~snapped[jc]] = 2
    state[near & ~accepted[jc] & snapped[jc]] = 3
    T["err"]["state"] = state

    # ---- GNSS quality ----
    T["gnss"] = dict(t=rel(gt[:, 0]),
                     sigma_xy=np.sqrt(np.maximum(gt[:, 4], gt[:, 5])),
                     sigma_z=np.sqrt(gt[:, 6]))
    if args.prepped_bag:
        pp = read_stamps_only(args.prepped_bag,
                              ["/gps_p1/filtered_odom", "/gps_p1/filtered_odom_rtk_fixed",
                               "/gps_p1/imu"])
        od = pp.get("/gps_p1/filtered_odom", np.empty(0))
        fx = pp.get("/gps_p1/filtered_odom_rtk_fixed", np.empty(0))
        if len(od):
            fixed = np.isin(np.round(od * 1000), np.round(fx * 1000))
            T["rtk"] = dict(t=rel(od), fixed=fixed)
        imu = pp.get("/gps_p1/imu", np.empty(0))
        if len(imu) > 1:
            T["imu"] = dict(t=rel(imu[1:]), gap=np.diff(imu))
    T["events"] = [(np.interp(w, pose[:, 4], pose[:, 0]) - T["t0"], lbl) for w, lbl in events]
    T["meta"] = dict(name=args.name, n_front_scans=None, est_rate=1.0 / np.median(np.diff(ts)))

    # optional clipping (e.g. to cut a known-broken replay tail)
    if args.t_max > 0:
        m = T["err"]["t"] <= args.t_max
        T["err"] = {k: (v[m] if isinstance(v, np.ndarray) and len(v) == len(m) else v)
                    for k, v in T["err"].items()}
        ms_ = T["scan"]["t"] <= args.t_max
        T["scan"] = {k: (v[ms_] if isinstance(v, np.ndarray) and len(v) == len(ms_) else v)
                     for k, v in T["scan"].items()}
        T["snap_t"] = T["snap_t"][T["snap_t"] <= args.t_max]
        T["defer_t"] = T["defer_t"][T["defer_t"] <= args.t_max]
        T["gate_t"] = T["gate_t"][T["gate_t"] <= args.t_max]
        for key in ("gnss", "rtk", "imu"):
            if key in T:
                mk = T[key]["t"] <= args.t_max
                T[key] = {k: v[mk] for k, v in T[key].items()}
    return T


# --------------------------------------------------------------------------
# segmentation + cause attribution
# --------------------------------------------------------------------------

def find_segments(T, thr, min_dur, merge_gap):
    t, e, valid = T["err"]["t"], T["err"]["e2d"], T["err"]["valid"]
    hot = (e > thr) & valid
    if not hot.any():
        return []
    d = np.diff(hot.astype(int))
    starts, ends = list(np.where(d == 1)[0] + 1), list(np.where(d == -1)[0] + 1)
    if hot[0]:
        starts.insert(0, 0)
    if hot[-1]:
        ends.append(len(hot))
    segs = [[s, e_] for s, e_ in zip(starts, ends)]
    merged = [segs[0]]
    for s, e_ in segs[1:]:
        if t[s] - t[merged[-1][1] - 1] < merge_gap:
            merged[-1][1] = e_
        else:
            merged.append([s, e_])
    return [(s, e_) for s, e_ in merged if t[e_ - 1] - t[s] >= min_dur]


def classify_segment(T, i0, i1, thr):
    E, S = T["err"], T["scan"]
    t0, t1 = E["t"][i0], E["t"][i1 - 1]
    m = (S["t"] >= t0 - 1) & (S["t"] <= t1 + 1)
    n_sc = int(m.sum())
    rej = (~S["accepted"][m])
    seg = dict(t0=t0, t1=t1, dur=t1 - t0,
               peak=float(E["e2d"][i0:i1].max()),
               t_peak=float(E["t"][i0:i1][E["e2d"][i0:i1].argmax()]),
               mean=float(E["e2d"][i0:i1].mean()),
               ez_peak=float(np.abs(E["ez"][i0:i1]).max()),
               n_scans=n_sc, rej_frac=float(rej.mean()) if n_sc else 1.0,
               n_snap=int(S["snapped"][m].sum()),
               n_defer=int(((T["defer_t"] >= t0 - 1) & (T["defer_t"] <= t1 + 1)).sum()),
               max_streak=int(S["streak"][m].max()) if n_sc else 0,
               fit_med=float(np.nanmedian(S["fitness"][m])) if n_sc else np.nan,
               ms_p95=float(np.nanpercentile(S["ms"][m], 95)) if n_sc else np.nan,
               max_proc_gap=float(S["dt_proc"][m].max()) if n_sc else np.nan,
               speed=float(E["speed"][i0:i1].mean()),
               speed_max=float(E["speed"][i0:i1].max()),
               x=float(E["gx"][i0:i1].mean()), y=float(E["gy"][i0:i1].mean()),
               max_age=float(E["corr_age"][i0:i1].max()),
               along=float(np.median(E["along"][i0:i1])),
               cross=float(np.median(E["cross"][i0:i1])))
    if n_sc:
        sts, cnt = np.unique(S["status"][m][rej], return_counts=True)
        seg["dom_reason"] = str(sts[cnt.argmax()]) if len(sts) else "-"
    else:
        seg["dom_reason"] = "no_scans_processed"
    denial = any(not (b < seg["t0"] or a > seg["t1"]) for a, b in T["denial"])
    seg["rtk_denied"] = denial or \
        (len(T["gate_t"]) and ((T["gate_t"] >= t0) & (T["gate_t"] <= t1)).any())

    causes, reason_txt = [], {
        "rejected_hessian": "GICP degeneracy gate vetoed the matches (feature-poor geometry / large correction)",
        "failed_to_converge": "GICP failed to converge (poor initial alignment or map mismatch)",
        "rejected_fitness": "poor match quality (scene differs from map)",
        "rejected_gt_veto": "GNSS integrity veto: confident LiDAR matches disagreed with RTK (track aliasing)",
        "no_scans_processed": "no LiDAR scans processed at all",
    }
    drift_consistent = seg["max_age"] > 1.0 and seg["speed_max"] > 2.0 and \
        seg["peak"] < 3.0 * seg["max_age"] * seg["speed_max"] and \
        seg["peak"] > 0.10 * seg["max_age"] * seg["speed_max"]
    if seg["rtk_denied"]:
        causes.append("GNSS denied (RTK gate active) -> snap-to-GT unavailable, IMU dead-reckoning carried the pose"
                      + (f"; drift accumulated over {seg['max_age']:.1f}s without corrections"
                         if seg["max_age"] > 1 else ""))
    if seg["max_age"] > 1.0 and not seg["rtk_denied"]:
        causes.append(f"no corrections for up to {seg['max_age']:.1f}s -> pure-IMU drift"
                      + (f" (peak {seg['peak']:.1f}m ~ speed x gap at up to {seg['speed_max']:.0f}m/s)"
                         if drift_consistent else ""))
    if seg["max_proc_gap"] > 0.5:
        causes.append(f"scan starvation: largest no-scan stretch {seg['max_proc_gap']:.1f}s "
                      f"(callback over budget -> QoS dropped scans)")
    if seg["rej_frac"] > 0.25 and seg["dom_reason"] in reason_txt:
        causes.append(f"{reason_txt[seg['dom_reason']]} - {seg['rej_frac'] * 100:.0f}% of processed scans non-accepted")
    if not np.isnan(seg["ms_p95"]) and seg["ms_p95"] > 1e3 * SCAN_PERIOD and seg["max_proc_gap"] <= 0.5:
        causes.append(f"GICP over the {SCAN_PERIOD * 1e3:.0f}ms budget (p95 {seg['ms_p95']:.0f}ms) -> stale corrections")
    if abs(seg["along"]) > 2 * abs(seg["cross"]) and seg["speed"] > 10 and seg["peak"] < 3.0:
        causes.append(f"residual is along-track ({seg['along']:+.2f}m at {seg['speed']:.0f}m/s = latency floor, "
                      f"not a matching failure)")
    if not causes:
        causes.append("LiDAR matches accepted yet offset persisted (map/reference disagreement)")
    seg["causes"] = causes

    # recovery attribution: what was active in the last second of the segment
    tail = (S["t"] >= t1 - 1.0) & (S["t"] <= t1 + 0.5)
    rec = []
    if S["snapped"][tail].any():
        rec.append("GNSS snap-to-GT pulled the pose back")
    if (S["accepted"][tail]).any():
        rec.append("LiDAR re-acquired (accepted matches resumed)")
    for a, b in T["denial"]:
        if t1 - 3 <= b <= t1 + 3:
            rec.insert(0, f"RTK readmitted at t={b:.0f}s")
    seg["recovery"] = "; ".join(rec) if rec else "error decayed gradually"
    return seg


# --------------------------------------------------------------------------
# statistics + verdicts
# --------------------------------------------------------------------------

def estats(e):
    return dict(rms=float(np.sqrt(np.mean(e ** 2))), med=float(np.median(e)),
                p95=float(np.percentile(e, 95)), mx=float(e.max()), n=int(len(e)))


def compute_stats(T, segs, thr):
    E, S = T["err"], T["scan"]
    v = E["valid"]
    out = dict(overall=estats(E["e2d"][v]), z=estats(np.abs(E["ez"][v])))
    names = {0: "no-scan (IMU-only ride)", 1: "LiDAR accepted", 2: "rejected->dead-reckon", 3: "rejected->GT snap"}
    out["by_state"] = {names[s]: estats(E["e2d"][v & (E["state"] == s)])
                       for s in (1, 2, 3, 0) if (v & (E["state"] == s)).any()}
    out["state_share"] = {names[s]: float((E["state"] == s).mean()) for s in (0, 1, 2, 3)}
    span = S["t"][-1] - S["t"][0]
    out["scan"] = dict(
        n=len(S["t"]), accepted_frac=float(S["accepted"].mean()),
        snap_frac=float(S["snapped"].mean()),
        reasons={str(k): int(c) for k, c in
                 zip(*np.unique(S["status"][~S["accepted"]], return_counts=True))} if (~S["accepted"]).any() else {},
        fit_acc=float(np.nanmedian(S["fitness"][S["accepted"]])) if S["accepted"].any() else np.nan,
        fit_rej=float(np.nanmedian(S["fitness"][~S["accepted"]])) if (~S["accepted"]).any() else np.nan,
        ms_med=float(np.nanmedian(S["ms"])), ms_p95=float(np.nanpercentile(S["ms"], 95)),
        over_budget=float(np.nanmean(S["ms"] > SCAN_PERIOD * 1e3)),
        proc_rate=float(len(S["t"]) / max(span, 1e-9)),
        proc_share=float(len(S["t"]) / max(span / SCAN_PERIOD, 1.0)),
        acc_rate=float(S["accepted"].sum() / max(span, 1e-9)),
        age_med=float(np.nanmedian(S["result_age_ms"])),
        age_p95=float(np.nanpercentile(S["result_age_ms"], 95)),
        max_proc_gap=float(S["dt_proc"].max()),
        max_streak=int(S["streak"].max()))
    # effective output lag: with fresh corrections at speed, the residual error
    # is almost purely along-track; its scale per unit speed is a time lag.
    fresh = (E["corr_age"] < 0.3) & (E["speed"] > 5.0) & v
    out["lag_ms"] = float(-np.median(E["along"][fresh] / E["speed"][fresh]) * 1e3) \
        if fresh.sum() > 100 else np.nan
    still = (E["speed"] < 1.0) & v & (E["corr_age"] < 0.5)
    out["still_err"] = float(np.median(E["e2d"][still])) if still.sum() > 50 else np.nan
    out["imu"] = dict(age_med=float(np.median(E["corr_age"])),
                      age_p95=float(np.percentile(E["corr_age"], 95)),
                      age_max=float(E["corr_age"].max()))
    if "imu" in T:
        out["imu"]["stream_max_gap"] = float(T["imu"]["gap"].max())
    if "rtk" in T:
        out["gnss"] = dict(fixed_frac=float(T["rtk"]["fixed"].mean()),
                           gate_drops=int(len(T["gate_t"])),
                           snaps=int(len(T["snap_t"])), defers=int(len(T["defer_t"])))
    # snap effectiveness: error in tight +-150 ms windows around each snap
    # (isolates the position reset from subsequent drift / neighboring snaps)
    drops = []
    for s in T["snap_t"]:
        b = E["e2d"][(E["t"] >= s - 0.15) & (E["t"] < s - 0.005)]
        a = E["e2d"][(E["t"] > s + 0.005) & (E["t"] <= s + 0.15)]
        if len(b) and len(a):
            drops.append((np.median(b), np.median(a)))
    if drops:
        d = np.array(drops)
        out["snap_effect"] = dict(n=len(d), before=float(np.median(d[:, 0])),
                                  after=float(np.median(d[:, 1])),
                                  helped=float(np.mean(d[:, 1] < 0.8 * d[:, 0])))
    # denial-window recovery: error at readmission, 1 s later, and the first
    # sustained (>=2 s) return below threshold
    recov = []
    for a, b in T["denial"]:
        m = (E["t"] > b) & E["valid"]
        if not m.any():
            continue
        tt, ee = E["t"][m], E["e2d"][m]
        e1 = ee[(tt > b + 0.8) & (tt < b + 1.3)]
        t_sus = np.inf
        below = ee < thr
        for i in np.where(below)[0]:
            j = np.searchsorted(tt, tt[i] + 2.0)
            if below[i:j].all() and j > i:
                t_sus = tt[i] - b
                break
        recov.append(dict(window=(a, b), err_at_end=float(np.interp(b, E["t"], E["e2d"])),
                          err_1s=float(np.median(e1)) if len(e1) else np.nan,
                          t_recover=float(t_sus)))
    out["denial_recovery"] = recov
    # IMU verdict stats restricted to the in-design regime (outside denial)
    nd = ~np.zeros(len(E["t"]), bool)
    for a, b in T["denial"]:
        nd &= ~((E["t"] >= a) & (E["t"] <= b + 2))
    out["imu_nd"] = dict(age_med=float(np.median(E["corr_age"][nd])),
                         age_p95=float(np.percentile(E["corr_age"][nd], 95)),
                         age_max=float(E["corr_age"][nd].max()))
    return out


def verdicts(T, st, segs_c):
    V = []
    sc = st["scan"]
    acc_t = st["state_share"].get("LiDAR accepted", 0)
    # V1: GNSS corrects when RTK good
    if "snap_effect" in st and st["snap_effect"]["n"] > 5:
        se = st["snap_effect"]
        n_rec = sum(1 for s in segs_c if "snap" in s["recovery"] or "RTK" in s["recovery"])
        capping = se["after"] <= se["before"] * 1.05
        grade = "pass" if capping and (not segs_c or n_rec >= len(segs_c) * 0.5) else "partial"
        V.append(("Use GNSS to correct globally when RTK is good",
                  f"{grade.upper()} - {len(T['snap_t'])} snap-to-GT events; across snaps median error "
                  f"{se['before']:.2f}->{se['after']:.2f} m (+-150 ms windows), {se['helped'] * 100:.0f}% drop >20%; "
                  f"{n_rec}/{len(segs_c)} high-error segments ended by GNSS pull-back. Snaps cap drift at the "
                  f"latency floor rather than zeroing it (snapped state is one callback old).", grade))
    else:
        V.append(("Use GNSS to correct globally when RTK is good",
                  "NOT EXERCISED - no snap events", "na"))
    # V2: GNSS disabled when RTK poor
    if T["denial"] or len(T["gate_t"]):
        n_def_stale = sum(1 for r in T["defer_reasons"] if r == "gt_stale")
        in_d = np.zeros(len(T["snap_t"]), bool)
        for a, b in T["denial"]:
            in_d |= (T["snap_t"] >= a + 0.3) & (T["snap_t"] <= b - 0.3)
        V.append(("Disable GNSS when RTK quality is poor",
                  f"{'PASS' if in_d.sum() == 0 else 'FAIL'} - {int(in_d.sum())} snaps inside denied windows "
                  f"(expect 0); {n_def_stale} snap attempts correctly deferred on stale GT; "
                  f"{len(T['gate_t'])} gate-drop log events", "pass" if in_d.sum() == 0 else "fail"))
    else:
        V.append(("Disable GNSS when RTK quality is poor",
                  "NOT EXERCISED - RTK was FIXED for the whole run (gate never tripped). "
                  "Use the synthetic-denial replay to test this leg.", "na"))
    # V3: LiDAR primary when reliable
    grade = "pass" if (sc["accepted_frac"] > 0.7 and sc["proc_share"] > 0.7) else \
            ("partial" if sc["accepted_frac"] > 0.4 else "fail")
    V.append(("Rely primarily on LiDAR when matching is reliable",
              f"{grade.upper()} - node processed {sc['proc_share'] * 100:.0f}% of incoming scans "
              f"({sc['proc_rate']:.1f} of {1 / SCAN_PERIOD:.0f} Hz; GICP {sc['ms_med']:.0f} ms median vs "
              f"{SCAN_PERIOD * 1e3:.0f} ms budget), accepted {sc['accepted_frac'] * 100:.0f}% of those -> net LiDAR "
              f"correction rate {sc['acc_rate']:.1f} Hz; {acc_t * 100:.0f}% of time in LiDAR-accepted state", grade))
    # V4: IMU short-term continuity (judged outside denial windows — bridging a
    # 40 s outage is not what "short-term continuity" promises)
    a_im = st.get("imu_nd", st["imu"])
    noscan = st["by_state"].get("no-scan (IMU-only ride)")
    dr = st["by_state"].get("rejected->dead-reckon")
    imu_err = noscan or dr
    grade = "pass" if (a_im["age_p95"] < 1.0 and (not imu_err or imu_err["p95"] < 3)) else \
            ("partial" if a_im["age_p95"] < 5.0 else "fail")
    V.append(("Use IMU for short-term continuity when LiDAR degrades",
              f"{grade.upper()} - IMU-only span (outside denial) median {a_im['age_med']:.2f}s / p95 "
              f"{a_im['age_p95']:.2f}s / max {a_im['age_max']:.1f}s"
              + (f"; error during IMU-only stretches median {imu_err['med']:.2f} m, p95 {imu_err['p95']:.2f} m"
                 if imu_err else "")
              + " (IMU holds ~1-2 s gaps to a few m; drift grows with speed x age beyond that)", grade))
    # V5: RTK recovery pulls back
    if st["denial_recovery"]:
        rr = st["denial_recovery"]
        txt = "; ".join(f"window {d['window'][0]:.0f}-{d['window'][1]:.0f}s: {d['err_at_end']:.0f}m at readmit "
                        f"-> {d['err_1s']:.1f}m 1s later"
                        + (f", sustained<thr at +{d['t_recover']:.0f}s" if np.isfinite(d['t_recover']) else "")
                        for d in rr)
        ok = all(np.isfinite(d["err_1s"]) and
                 (d["err_1s"] < 0.1 * max(d["err_at_end"], 1e-9) or d["err_1s"] < 3.0) for d in rr)
        V.append(("Once RTK recovers, GNSS pulls the trajectory back",
                  f"{'PASS' if ok else 'PARTIAL'} - {txt}", "pass" if ok else "partial"))
    else:
        V.append(("Once RTK recovers, GNSS pulls the trajectory back",
                  "NOT EXERCISED - no RTK loss/recovery transition in this run", "na"))
    return V


def findings(T, st):
    """Run-specific, data-driven observations beyond the five design rules."""
    F = []
    sc = st["scan"]
    if sc["proc_share"] < 0.8:
        F.append(f"Scan-processing starvation: only {sc['proc_share'] * 100:.0f}% of incoming scans were "
                 f"processed ({sc['proc_rate']:.1f} of {1 / SCAN_PERIOD:.0f} Hz). GICP median "
                 f"{sc['ms_med']:.0f} ms (p95 {sc['ms_p95']:.0f} ms) vs the {SCAN_PERIOD * 1e3:.0f} ms scan "
                 f"period, so the callback can't keep up and SensorDataQoS drops the backlog; the largest "
                 f"no-scan stretch was {sc['max_proc_gap']:.1f} s, during which the pose rode IMU alone.")
    if np.isfinite(st.get("lag_ms", np.nan)) and st["lag_ms"] > 20:
        F.append(f"Latency floor: with fresh corrections at speed the residual error is almost purely "
                 f"along-track and scales with speed as an effective output lag of ~{st['lag_ms']:.0f} ms "
                 f"(compare GICP median {sc['ms_med']:.0f} ms + result age median {sc['age_med']:.0f} ms). "
                 f"Corrections (GICP pose or GT snap) reset the state to the scan-time pose without "
                 f"replaying IMU forward, so the published pose trails ground truth by about one "
                 f"callback latency: error ~ speed x lag (e.g. {st['lag_ms'] / 1000 * 15:.1f} m at 15 m/s).")
    if np.isfinite(st.get("still_err", np.nan)):
        F.append(f"Stationary sanity: with the car stopped the median error is {st['still_err'] * 1e3:.0f} mm "
                 f"- map alignment and reference agree; all larger error is dynamic in origin.")
    if sc["accepted_frac"] < 0.5 and sc["reasons"]:
        top = max(sc["reasons"].items(), key=lambda kv: kv[1])
        F.append(f"LiDAR vetoed: only {sc['accepted_frac'] * 100:.0f}% of processed scans were accepted; "
                 f"dominant rejection '{top[0]}' ({top[1]} scans). With gt_recovery min_consecutive_failures=1, "
                 f"nearly every rejection immediately snapped to GT - the estimator effectively ran "
                 f"GNSS+IMU with sparse LiDAR confirmation ({sc['acc_rate']:.1f} Hz accepted corrections).")
    if "gnss" in st and st["gnss"]["fixed_frac"] > 0.999 and not T["denial"]:
        F.append("RTK was FIXED for 100% of this bag - the GNSS-degraded branches (rtk_gate drop, snap "
                 "deferral, re-acquisition) are untested by this data; see the synthetic-denial run.")
    if T["denial"]:
        S = T["scan"]
        in_d = np.zeros(len(S["t"]), bool)
        for a, b in T["denial"]:
            in_d |= (S["t"] >= a) & (S["t"] <= b)
        n_acc_d = int(S["accepted"][in_d].sum())
        fit_late = np.nanmedian(S["fitness"][in_d][-max(int(in_d.sum() * 0.25), 1):]) if in_d.any() else np.nan
        F.append(f"No re-acquisition without GNSS: during the denied windows only {n_acc_d} of "
                 f"{int(in_d.sum())} scans were accepted; as the IMU prior drifted past GICP's "
                 f"correspondence radius the fitness exploded (late-window median {fit_late:.0f}), so "
                 f"LiDAR could not pull the pose back on its own - drift was unbounded until RTK "
                 f"readmission. A relocalization mechanism (wider search / multi-hypothesis) is absent.")
    return F


# --------------------------------------------------------------------------
# plotting
# --------------------------------------------------------------------------

def bin_env(t, v, dt=0.2):
    if len(t) == 0:
        return t, v, v
    b = ((t - t[0]) / dt).astype(int)
    u, inv = np.unique(b, return_inverse=True)
    med = np.full(len(u), np.nan)
    mx = np.full(len(u), np.nan)
    for k in range(len(u)):
        m = inv == k
        med[k], mx[k] = np.median(v[m]), v[m].max()
    return t[0] + u * dt + dt / 2, med, mx


def state_runs(t, dt, flags):
    """merge consecutive same-flag scans into (start, width) bars."""
    runs = {}
    i = 0
    while i < len(t):
        j = i
        while j + 1 < len(t) and flags[j + 1] == flags[i] and t[j + 1] - t[j] < 1.0:
            j += 1
        runs.setdefault(flags[i], []).append((t[i] - dt / 2, t[j] - t[i] + dt))
        i = j + 1
    return runs


def shade_common(ax, T, segs):
    for k, (i0, i1) in enumerate(segs):
        ax.axvspan(T["err"]["t"][i0], T["err"]["t"][i1 - 1], color="red", alpha=0.08, zorder=0)
    for a, b in T["denial"]:
        ax.axvspan(a, b, color="purple", alpha=0.10, zorder=0, hatch="//", lw=0)


def plot_timeline(T, segs, st, thr, out_png):
    E, S = T["err"], T["scan"]
    fig, axs = plt.subplots(6, 1, figsize=(17, 15), sharex=True,
                            gridspec_kw=dict(height_ratios=[2.2, 1.5, 1.1, 1.2, 1.3, 1.0], hspace=0.07))
    fig.suptitle(f"Unified sensor-fusion timeline — {T['meta']['name']}   "
                 f"(err median {st['overall']['med']:.2f} m, p95 {st['overall']['p95']:.2f} m; processed "
                 f"{st['scan']['proc_share'] * 100:.0f}% of scans, accepted {st['scan']['accepted_frac'] * 100:.0f}% "
                 f"of processed; {len(T['snap_t'])} GT snaps)", fontsize=13, y=0.995)

    # A: error
    ax = axs[0]
    tb, med, mx = bin_env(E["t"][E["valid"]], E["e2d"][E["valid"]])
    ax.fill_between(tb, med, mx, color="tab:red", alpha=0.25, lw=0, label="0.2s max envelope")
    ax.plot(tb, med, color="tab:red", lw=1.0, label="horizontal error")
    tzb, zmed, zmx = bin_env(E["t"][E["valid"]], np.abs(E["ez"][E["valid"]]))
    ax.plot(tzb, zmed, color="tab:brown", lw=0.7, alpha=0.7, label="|z| error")
    ax.axhline(thr, color="k", ls="--", lw=0.8, label=f"segment thr {thr} m")
    if len(T["snap_t"]):
        ax.plot(T["snap_t"], np.full(len(T["snap_t"]), ax.get_ylim()[0]), "|",
                color=C_SNAP, ms=10, mew=1.2, label="snap-to-GT")
    ax.set_yscale("log")
    ax.set_ylim(max(5e-3, med[np.isfinite(med)].min() * 0.5), max(mx.max() * 2, thr * 4))
    for k, (i0, i1) in enumerate(segs):
        ax.annotate(f"S{k + 1}", (E["t"][i0:i1][E["e2d"][i0:i1].argmax()], E["e2d"][i0:i1].max()),
                    xytext=(0, 6), textcoords="offset points", ha="center",
                    fontsize=9, fontweight="bold", color="darkred")
    ax.set_ylabel("position error [m]")
    ax.legend(loc="upper left", ncol=5, fontsize=8)
    shade_common(ax, T, segs)

    # B: LiDAR matching quality
    ax = axs[1]
    a = S["accepted"]
    ax.scatter(S["t"][a], S["fitness"][a], s=4, c=C_ACC, label="fitness (accepted)", rasterized=True)
    ax.scatter(S["t"][~a], S["fitness"][~a], s=4, c="tab:red", label="fitness (rejected)", rasterized=True)
    ax.axhline(1.0, color="tab:red", ls=":", lw=0.8)
    ax.axhline(0.15, color="0.4", ls=":", lw=0.8)
    ax.set_yscale("log")
    ax.set_ylabel("GICP fitness")
    h = S["hessian"]
    if np.isfinite(h).any():
        ax2 = ax.twinx()
        ax2.scatter(S["t"][np.isfinite(h)], h[np.isfinite(h)], s=2, c="tab:purple", alpha=0.5, rasterized=True)
        ax2.axhline(5e9, color="tab:purple", ls=":", lw=0.8)
        ax2.set_yscale("log")
        ax2.set_ylabel("hessian cond (purple)", color="tab:purple", fontsize=8)
        ax2.tick_params(axis="y", labelsize=7, colors="tab:purple")
    ax.legend(loc="upper left", ncol=3, fontsize=8)
    shade_common(ax, T, segs)

    # C: processing time / latency
    ax = axs[2]
    ax.scatter(S["t"], S["ms"], s=3, c="tab:cyan", label="GICP time [ms]", rasterized=True)
    ra = np.clip(S["result_age_ms"], 1.0, None)
    ax.scatter(S["t"], ra, s=2, c="0.35", alpha=0.6, label="scan result age at publish [ms]", rasterized=True)
    ax.axhline(SCAN_PERIOD * 1e3, color="k", ls="--", lw=0.8, label="50 ms scan budget")
    ax.set_ylabel("latency [ms]")
    ax.set_yscale("log")
    ax.legend(loc="upper left", fontsize=8, ncol=3)
    shade_common(ax, T, segs)

    # D: IMU burden
    ax = axs[3]
    ax.plot(E["t"], E["corr_age"], color=C_DR, lw=0.9, label="age of last LiDAR/GNSS correction [s]")
    ax.step(S["t"], S["streak"], where="post", color="tab:red", lw=0.7, alpha=0.6,
            label="consecutive non-accepted scans")
    if "imu" in T:
        tg, gmed, gmax = bin_env(T["imu"]["t"], T["imu"]["gap"] * 1e3, 1.0)
        ax.plot(tg, gmax / 1e3, color="0.5", lw=0.6, alpha=0.7, label="IMU stream max gap [s]")
    ax.set_ylabel("IMU-only span [s] / streak")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.legend(loc="upper left", fontsize=8, ncol=3)
    shade_common(ax, T, segs)

    # E: GNSS / RTK quality + events
    ax = axs[4]
    G = T["gnss"]
    ax.plot(G["t"], G["sigma_xy"], color="tab:green", lw=0.8, label="GT reported sigma_xy [m]")
    ax.axhline(np.sqrt(GATE_VAR_XY), color="tab:red", ls="--", lw=0.9,
               label=f"rtk_gate ({np.sqrt(GATE_VAR_XY):.2f} m)")
    ax.set_yscale("log")
    ax.set_ylabel("GNSS sigma [m]")
    if len(T["gate_t"]):
        ax.plot(T["gate_t"], np.full(len(T["gate_t"]), np.sqrt(GATE_VAR_XY) * 1.5), "v",
                color="tab:red", ms=4, label="gate drop")
    if len(T["snap_t"]):
        ax.plot(T["snap_t"], np.full(len(T["snap_t"]), G["sigma_xy"].min() * 1.6), "|",
                color=C_SNAP, ms=8, label="snap")
    if len(T["defer_t"]):
        ax.plot(T["defer_t"], np.full(len(T["defer_t"]), G["sigma_xy"].min() * 2.6), "x",
                color="tab:purple", ms=5, label="snap deferred (GT stale)")
    if "rtk" in T:
        R = T["rtk"]
        runs = state_runs(R["t"], 0.02, R["fixed"].astype(int))
        y0 = G["sigma_xy"].min() * 0.55
        for flag, bars in runs.items():
            ax.broken_barh(bars, (y0 * 0.8, y0 * 0.35),
                           facecolors="tab:green" if flag else "tab:red", lw=0)
    ax.legend(loc="upper left", fontsize=8, ncol=5)
    shade_common(ax, T, segs)

    # F: speed + decision-state strip
    ax = axs[5]
    ax.plot(E["t"], E["speed"], color="tab:gray", lw=0.8)
    ax.set_ylabel("speed [m/s]")
    ax.set_xlabel(f"time since bag start [s]   (t0 = {T['t0']:.3f})")
    code = np.where(S["accepted"], 1, np.where(S["snapped"], 3, 2))
    runs = state_runs(S["t"], SCAN_PERIOD, code)
    ymax = max(E["speed"].max(), 1) * 1.15
    colors = {1: C_ACC, 2: C_DR, 3: C_SNAP}
    for flag, bars in runs.items():
        ax.broken_barh(bars, (ymax, ymax * 0.08), facecolors=colors[flag], lw=0)
    # gray = stretches with no processed scan
    gaps = np.where(S["dt_proc"] > 0.3)[0]
    ax.broken_barh([(S["t"][i] - S["dt_proc"][i] + SCAN_PERIOD, S["dt_proc"][i] - SCAN_PERIOD)
                    for i in gaps], (ymax, ymax * 0.08), facecolors=C_NOSCAN, lw=0, zorder=0)
    ax.set_ylim(0, ymax * 1.12)
    ax.legend(handles=[Patch(color=C_ACC, label="LiDAR accepted"),
                       Patch(color=C_DR, label="rejected -> IMU DR"),
                       Patch(color=C_SNAP, label="rejected -> GT snap"),
                       Patch(color=C_NOSCAN, label="no scan processed"),
                       Patch(facecolor="purple", alpha=0.2, hatch="//", label="synthetic RTK denial"),
                       Patch(facecolor="red", alpha=0.15, label="high-error segment")],
              loc="upper left", ncol=6, fontsize=8)
    shade_common(ax, T, segs)
    for axx in axs:
        axx.grid(alpha=0.25, lw=0.4)
        axx.margins(x=0.01)
    fig.savefig(out_png, dpi=135, bbox_inches="tight")
    plt.close(fig)


def plot_track(T, segs, out_png):
    E = T["err"]
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.plot(E["gx"], E["gy"], color="0.85", lw=4, zorder=0, label="reference (RTK)")
    v = E["valid"]
    sc = ax.scatter(E["x"][v], E["y"][v], c=np.clip(E["e2d"][v], 1e-3, None), s=2.5,
                    cmap="turbo", norm=matplotlib.colors.LogNorm(
                        vmin=max(np.percentile(E["e2d"][v], 5), 1e-2),
                        vmax=max(E["e2d"][v].max(), 1.0)), rasterized=True)
    plt.colorbar(sc, ax=ax, label="horizontal error [m] (log)", shrink=0.8)
    for k, (i0, i1) in enumerate(segs):
        j = i0 + E["e2d"][i0:i1].argmax()
        ax.annotate(f"S{k + 1}", (E["x"][j], E["y"][j]), fontsize=8, fontweight="bold",
                    color="black", ha="center", xytext=(0, 7), textcoords="offset points",
                    bbox=dict(boxstyle="round,pad=0.15", fc="yellow", alpha=0.65, lw=0))
    # mark every 60 s of progress for orientation
    for tt in np.arange(0, E["t"][-1], 60):
        j = np.searchsorted(E["t"], tt)
        if j < len(E["t"]):
            ax.annotate(f"{tt:.0f}s", (E["gx"][j], E["gy"][j]), fontsize=6, color="0.4")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Track map colored by error — {T['meta']['name']} (segments marked)")
    ax.legend(loc="best", fontsize=8)
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
    plt.close(fig)


def plot_distributions(T, st, out_png):
    E, S = T["err"], T["scan"]
    v = E["valid"]
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    ax = axs[0, 0]
    for s, lbl, c in [(1, "LiDAR accepted", C_ACC), (2, "reject->DR", C_DR),
                      (3, "reject->snap", C_SNAP), (0, "no scan", "0.5")]:
        m = v & (E["state"] == s)
        if m.sum() > 20:
            e = np.sort(E["e2d"][m])
            ax.plot(e, np.linspace(0, 1, len(e)), color=c, label=f"{lbl} (n={m.sum()})")
    ax.set_xscale("log")
    ax.set_xlabel("horizontal error [m]")
    ax.set_ylabel("CDF")
    ax.set_title("Error conditioned on decision state")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axs[0, 1]
    bins = np.logspace(np.log10(max(np.nanmin(S["fitness"]), 1e-4)),
                       np.log10(max(np.nanmax(S["fitness"]), 1.0)), 60)
    ax.hist(S["fitness"][S["accepted"]], bins=bins, color=C_ACC, alpha=0.7, label="accepted")
    ax.hist(S["fitness"][~S["accepted"]], bins=bins, color="tab:red", alpha=0.6, label="rejected")
    ax.set_xscale("log")
    ax.set_xlabel("GICP fitness")
    ax.set_title("Fitness by outcome")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axs[1, 0]
    es = np.interp(S["stamp"], E["t"] + T["t0"], E["e2d"])
    for m, lbl, c in [(S["accepted"], "accepted", C_ACC), (~S["accepted"], "rejected", "tab:red")]:
        ax.scatter(S["fitness"][m], es[m], s=4, c=c, alpha=0.5, label=lbl, rasterized=True)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("fitness")
    ax.set_ylabel("error at scan time [m]")
    ax.set_title("Does fitness predict error?")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax = axs[1, 1]
    ms = S["ms"][np.isfinite(S["ms"]) & (S["ms"] > 0)]
    ax.hist(ms, bins=np.logspace(np.log10(max(ms.min(), 1)), np.log10(ms.max()), 60), color="tab:cyan")
    ax.axvline(SCAN_PERIOD * 1e3, color="k", ls="--", label="50 ms budget")
    ax.set_xscale("log")
    ax.set_xlabel("GICP time [ms]")
    ax.set_title(f"GICP cost (median {st['scan']['ms_med']:.0f} ms, "
                 f"{st['scan']['over_budget'] * 100:.0f}% over budget); "
                 f"processed {st['scan']['proc_share'] * 100:.0f}% of incoming scans")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.suptitle(f"Distributions — {T['meta']['name']}")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_segment_zoom(T, seg, k, out_png):
    E, S = T["err"], T["scan"]
    pad = max(8.0, seg["dur"] * 0.5)
    t0, t1 = seg["t0"] - pad, seg["t1"] + pad
    me = (E["t"] >= t0) & (E["t"] <= t1)
    ms = (S["t"] >= t0) & (S["t"] <= t1)
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    ax = axs[0]
    ax.plot(E["t"][me], E["e2d"][me], color="tab:red", lw=1.0, label="horiz err")
    ax.plot(E["t"][me], E["along"][me], color="tab:blue", lw=0.6, alpha=0.7, label="along-track")
    ax.plot(E["t"][me], E["cross"][me], color="tab:green", lw=0.6, alpha=0.7, label="cross-track")
    ax.axvspan(seg["t0"], seg["t1"], color="red", alpha=0.08)
    ax.set_ylabel("error [m]")
    ax.legend(fontsize=8, ncol=3)
    ax.set_title(f"S{k}: {seg['t0']:.0f}-{seg['t1']:.0f}s  peak {seg['peak']:.2f} m — "
                 + seg["causes"][0][:90])
    ax = axs[1]
    a = S["accepted"][ms]
    ax.scatter(S["t"][ms][a], S["fitness"][ms][a], s=8, c=C_ACC, label="accepted")
    ax.scatter(S["t"][ms][~a], S["fitness"][ms][~a], s=8, c="tab:red", label="rejected")
    for x in T["snap_t"][(T["snap_t"] >= t0) & (T["snap_t"] <= t1)]:
        ax.axvline(x, color=C_SNAP, lw=0.5, alpha=0.5)
    ax.set_yscale("log")
    ax.set_ylabel("fitness (| = snap)")
    ax.legend(fontsize=8)
    ax = axs[2]
    ax.plot(E["t"][me], E["corr_age"][me], color=C_DR, lw=0.9, label="correction age [s]")
    ax.plot(E["t"][me], E["speed"][me] / 10, color="0.6", lw=0.7, label="speed/10 [m/s]")
    for aa, bb in T["denial"]:
        if bb > t0 and aa < t1:
            ax.axvspan(max(aa, t0), min(bb, t1), color="purple", alpha=0.12, hatch="//")
    ax.set_ylabel("age [s] / speed")
    ax.set_xlabel("time [s]")
    ax.legend(fontsize=8)
    for axx in axs:
        axx.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120, bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------

def fmt_seg_md(k, s):
    c = "; ".join(s["causes"])
    return (f"| S{k} | {s['t0']:.0f}–{s['t1']:.0f} | {s['dur']:.1f} | {s['peak']:.2f} | "
            f"{s['mean']:.2f} | {s['rej_frac'] * 100:.0f}% | {s['dom_reason']} | {s['n_snap']} | "
            f"{s['max_age']:.1f} | {s['speed']:.0f} |\n"), c


def write_report(T, segs_c, st, V, args, outdir, figs):
    name = args.name
    L = [f"# Sensor-fusion timeline analysis — {name}\n",
         f"- loc dir: `{args.loc_dir}`   prepped bag: `{args.prepped_bag}`",
         f"- est `{args.est_topic}` vs reference `{args.gt_topic}` (RTK/INS); "
         f"t0 = bag start = {T['t0']:.3f}",
         f"- segment threshold {args.err_threshold} m; GT cov gate "
         f"{args.gt_var_gate} m^2" + (f"; synthetic denial windows {args.denial_windows}" if args.denial_windows else ""),
         ""]
    o, z = st["overall"], st["z"]
    L += ["## Key metrics\n",
          "| metric | value |", "|---|---|",
          f"| horizontal error rms / median / p95 / max | {o['rms']:.3f} / {o['med']:.3f} / {o['p95']:.3f} / {o['mx']:.2f} m |",
          f"| z error rms / median | {z['rms']:.3f} / {z['med']:.3f} m |",
          f"| est publish rate | {T['meta']['est_rate']:.1f} Hz |",
          f"| scans processed / accepted | {st['scan']['n']} = {st['scan']['proc_share'] * 100:.0f}% of incoming "
          f"({st['scan']['proc_rate']:.1f} Hz of 20 Hz) / {st['scan']['accepted_frac'] * 100:.1f}% of processed "
          f"(net {st['scan']['acc_rate']:.1f} Hz) |",
          f"| rejection reasons | {st['scan']['reasons']} |",
          f"| GICP time median / p95 / >50ms | {st['scan']['ms_med']:.1f} / {st['scan']['ms_p95']:.1f} ms / "
          f"{st['scan']['over_budget'] * 100:.0f}% |",
          f"| scan result age at publish median / p95 | {st['scan']['age_med']:.0f} / {st['scan']['age_p95']:.0f} ms |",
          f"| effective output lag (along-track/speed) | "
          + (f"{st['lag_ms']:.0f} ms |" if np.isfinite(st.get('lag_ms', np.nan)) else "n/a |"),
          f"| stationary error | "
          + (f"{st['still_err'] * 1e3:.0f} mm |" if np.isfinite(st.get('still_err', np.nan)) else "n/a |"),
          f"| largest no-scan gap | {st['scan']['max_proc_gap']:.2f} s |",
          f"| correction age median / p95 / max | {st['imu']['age_med']:.2f} / {st['imu']['age_p95']:.2f} / "
          f"{st['imu']['age_max']:.2f} s |"]
    if "gnss" in st:
        L.append(f"| RTK fixed share / gate drops / snaps / defers | "
                 f"{st['gnss']['fixed_frac'] * 100:.1f}% / {st['gnss']['gate_drops']} / "
                 f"{st['gnss']['snaps']} / {st['gnss']['defers']} |")
    if "snap_effect" in st:
        se = st["snap_effect"]
        L.append(f"| snap effect (median err before -> after) | {se['before']:.2f} -> {se['after']:.2f} m "
                 f"({se['n']} measurable) |")
    L += ["", "## Error conditioned on decision state\n",
          "| state | time share | median | p95 | max | n |", "|---|---|---|---|---|---|"]
    for s, d in st["by_state"].items():
        L.append(f"| {s} | {st['state_share'].get(s, 0) * 100:.1f}% | {d['med']:.3f} | {d['p95']:.3f} | "
                 f"{d['mx']:.2f} | {d['n']} |")
    L += ["", "## High-error segments\n",
          "| seg | t [s] | dur | peak [m] | mean [m] | rej% | dominant reason | snaps | max IMU-only [s] | v [m/s] |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    expl = []
    for k, s in enumerate(segs_c, 1):
        row, cause = fmt_seg_md(k, s)
        L.append(row.rstrip())
        expl.append(f"- **S{k} ({s['t0']:.0f}–{s['t1']:.0f}s, peak {s['peak']:.2f} m @ "
                    f"({s['x']:.0f},{s['y']:.0f})m):** {cause}. Recovery: {s['recovery']}.")
    L += ["", "### Segment explanations\n"] + (expl or ["- no segments above threshold"])
    if st["denial_recovery"]:
        L += ["", "## Denial-window recovery\n"]
        for d in st["denial_recovery"]:
            a, b = d["window"]
            L.append(f"- window {a:.0f}–{b:.0f}s: error at RTK readmit {d['err_at_end']:.2f} m -> "
                     f"{d['err_1s']:.2f} m one second later"
                     + (f"; sustained below {args.err_threshold} m at +{d['t_recover']:.0f} s"
                        if np.isfinite(d['t_recover']) else ""))
    L += ["", "## Findings\n"] + [f"- {f}" for f in (write_report.findings or ["-"])]
    L += ["", "## Strategy verification\n"]
    icon = {"pass": "PASS", "partial": "PARTIAL", "fail": "FAIL", "na": "n/a"}
    for rule, evidence, grade in V:
        L.append(f"- [{icon[grade]}] **{rule}** — {evidence}")
    L += ["", "## Figures\n"] + [f"- `{os.path.basename(f)}`" for f in figs] + [""]
    p = os.path.join(outdir, f"report_{name}.md")
    open(p, "w").write("\n".join(L))
    return p


def write_csvs(T, segs_c, outdir, name):
    E, S = T["err"], T["scan"]
    np.savetxt(os.path.join(outdir, f"error_{name}.csv"),
               np.column_stack([E["t"], E["e2d"], E["ez"], E["along"], E["cross"],
                                E["speed"], E["corr_age"], E["state"], E["valid"]]),
               header="t e2d ez along cross speed corr_age state valid", comments="",
               fmt="%.4f")
    cols = ["t", "fitness", "ms", "hessian", "ratio", "streak", "dt_proc"]
    arr = np.column_stack([S[c] for c in cols] +
                          [S["accepted"].astype(int), S["snapped"].astype(int)])
    hdr = " ".join(cols + ["accepted", "snapped"])
    np.savetxt(os.path.join(outdir, f"scans_{name}.csv"), arr, header=hdr,
               comments="", fmt="%.6g")
    with open(os.path.join(outdir, f"segments_{name}.csv"), "w") as f:
        f.write("seg,t0,t1,dur,peak,mean,rej_frac,dom_reason,n_snap,max_imu_age,speed,causes,recovery\n")
        for k, s in enumerate(segs_c, 1):
            f.write(f"S{k},{s['t0']:.1f},{s['t1']:.1f},{s['dur']:.1f},{s['peak']:.2f},{s['mean']:.2f},"
                    f"{s['rej_frac']:.2f},{s['dom_reason']},{s['n_snap']},{s['max_age']:.1f},"
                    f"{s['speed']:.0f},\"{'; '.join(s['causes'])}\",\"{s['recovery']}\"\n")


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--loc-dir", required=True)
    ap.add_argument("--prepped-bag", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--name", default="run")
    ap.add_argument("--est-topic", default="/gicp/localization/odom")
    ap.add_argument("--gt-topic", default="/gps_p1/filtered_odom_map")
    ap.add_argument("--err-threshold", type=float, default=1.0)
    ap.add_argument("--seg-min-dur", type=float, default=1.0)
    ap.add_argument("--seg-merge-gap", type=float, default=5.0)
    ap.add_argument("--seg-peak-factor", type=float, default=3.0,
                    help="keep only segments whose peak exceeds threshold*factor "
                         "(stats still use the plain threshold)")
    ap.add_argument("--max-gt-gap", type=float, default=0.5)
    ap.add_argument("--gt-var-gate", type=float, default=1e-3,
                    help="m^2: exclude GT above this variance from the error reference "
                         "(denial windows exempt). <=0 disables.")
    ap.add_argument("--denial-windows", default="",
                    help="'a:b,c:d' seconds from bag start where GNSS was synthetically denied")
    ap.add_argument("--max-zooms", type=int, default=8)
    ap.add_argument("--t-max", type=float, default=0.0,
                    help="analyze only the first N seconds (0 = all)")
    args = ap.parse_args()
    outdir = args.out or os.path.join(args.loc_dir, "fusion_analysis")
    os.makedirs(outdir, exist_ok=True)

    print(f"[1/5] reading bags + log for {args.name} ...")
    T = build(args)
    print(f"[2/5] segmenting (thr {args.err_threshold} m) ...")
    segs = find_segments(T, args.err_threshold, args.seg_min_dur, args.seg_merge_gap)
    peak_min = args.err_threshold * args.seg_peak_factor
    segs = [(i0, i1) for i0, i1 in segs if T["err"]["e2d"][i0:i1].max() >= peak_min]
    segs_c = [classify_segment(T, i0, i1, args.err_threshold) for i0, i1 in segs]
    st = compute_stats(T, segs_c, args.err_threshold)
    V = verdicts(T, st, segs_c)
    write_report.findings = findings(T, st)
    print(f"[3/5] plotting ...")
    figs = [os.path.join(outdir, f"timeline_{args.name}.png"),
            os.path.join(outdir, f"track_map_{args.name}.png"),
            os.path.join(outdir, f"distributions_{args.name}.png")]
    plot_timeline(T, segs, st, args.err_threshold, figs[0])
    plot_track(T, segs, figs[1])
    plot_distributions(T, st, figs[2])
    zdir = os.path.join(outdir, f"segments_{args.name}")
    os.makedirs(zdir, exist_ok=True)
    for k, seg in enumerate(segs_c[:args.max_zooms], 1):
        f = os.path.join(zdir, f"seg{k:02d}.png")
        plot_segment_zoom(T, seg, k, f)
        figs.append(f)
    print(f"[4/5] writing csv + report ...")
    write_csvs(T, segs_c, outdir, args.name)
    rp = write_report(T, segs_c, st, V, args, outdir, figs)
    print(f"[5/5] done -> {outdir}")
    o = st["overall"]
    print(f"\n=== {args.name}: err median {o['med']:.3f} m  rms {o['rms']:.3f}  p95 {o['p95']:.3f}  "
          f"max {o['mx']:.2f} | processed {st['scan']['proc_share'] * 100:.0f}% of scans, accepted "
          f"{st['scan']['accepted_frac'] * 100:.0f}% | snaps {len(T['snap_t'])} ===")
    for f in write_report.findings:
        print(f"  * {f}")
    for rule, evidence, grade in V:
        print(f"  [{grade.upper():7s}] {rule}: {evidence[:160]}")
    print(f"\nreport: {rp}")
    for s_idx, s in enumerate(segs_c, 1):
        print(f"  S{s_idx} {s['t0']:.0f}-{s['t1']:.0f}s peak {s['peak']:.2f}m: {s['causes'][0][:110]}")


if __name__ == "__main__":
    main()
