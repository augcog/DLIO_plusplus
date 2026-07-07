# GICP Turn-Error Investigation — 2026-07-04

Investigation of the large localization error during turns reported on `art-jazzy` HEAD
(`9e7dbc0`), using `runs/06 GICP 0701.webm`, run 12 (run3 data vs run5 map), and run 13
(run5 data vs run3 map). All numbers below were re-derived from `localization.log` /
`debug_bag_manual_audit.json`, not copied from the run reports.

## Evidence summary

Turn error is real and monotonic in yaw rate, **even on accepted frames** (run 12,
12,190 accepted scans, yaw rate from consecutive guess poses):

| yaw rate (deg/s) | n | gt_pos median | gt_pos p90 | gt_rot median | gt_rot p90 |
|---|---|---|---|---|---|
| 0–2   | 2,846 | 1.01 m | 5.08 m  | 1.07° | 2.77° |
| 2–10  | 4,159 | 1.14 m | 6.61 m  | 1.27° | 3.94° |
| 10–25 | 2,821 | 1.31 m | 7.31 m  | 1.47° | 4.33° |
| 25–90 | 1,394 | 2.06 m | 13.39 m | 1.75° | 7.23° |

Two distinct failure modes:

1. **Accepted-frame yaw-rate-proportional error** (table above) — a latency/deskew/observer
   problem, not a gating problem.
2. **Long dead-reckoning streaks**: 179 rejection streaks; 16 streaks ≥10 frames account for
   1,648 of 1,860 rejections; the longest is **253 consecutive `rejected_hessian` frames
   (~25 s)**. During streaks the pose is held together only by GT snap
   (`gt_recovery/min_consecutive_failures=1`), which fires every frame — production without
   GT would fully diverge here.

Rejections are *not* concentrated at turns (accept rate 86.9% turning vs 86.4% straight),
but their damage is: gt_err during streaks oscillates 0.3–12 m between snaps.

## Root causes (GICP node)

### RC1 — Degeneracy gate collapses to "hessian alone" on cross-run maps
`gicp/hessianFitnessWarnThreshold=0.15` was calibrated on a same-run map (healthy fitness
0.03–0.06). On the cross-run map the fitness **floor** is ~0.27 (p10 = 0.255, median 0.278 —
identical for accepted and rejected frames). The `fitness > 0.15` branch of the combined
gate is therefore true on essentially every frame, reducing the gate to the legacy
hessian-alone behavior that the YAML comment itself documents as over-rejecting at
feature-poor corners. Result: 1,804 hessian rejections whose GT error median was only
1.57 m — mostly good solutions thrown away, replaced by 25 s of dead reckoning.
`hessianTransWarnM=1.0` has the same problem: rejected-frame jump median is 1.47 m, so the
translation branch also fires quasi-permanently once the prior degrades.

Fix options, in order of preference:
- Normalize the fitness signal per map: warn threshold = k × rolling-median fitness
  (k ≈ 1.5–2) instead of an absolute 0.15.
- Better: stop binary-rejecting degenerate scans. Eigendecompose the 6×6 hessian and apply
  the GICP correction only along well-constrained eigen-directions (Zhang-&-Singh-style
  solution remapping / partial update), keeping the IMU prior along degenerate axes.
- At minimum raise `hessianCondMax` for cross-run maps: observed p90 is 6.9e9 vs the 5.0e9
  threshold, so the gate sits exactly on the bulk of the distribution.

### RC2 — Stale velocity seeds the prior during rejection streaks (bug)
`localization.cc` rejection branch (~line 3259) sets `prev_vel = geo.prev_vel`, but
`geo.prev_vel` is only written in `updateState()`, i.e. **on accepted scans**. During an
N-frame streak every per-scan `integrateImu(prev_scan_stamp, basePose, prev_vel, …)` restarts
from a velocity frozen at the last accepted scan. In a turn the velocity direction rotates
20–40°/s, so the prior extrapolates straight/wrong and position error grows quadratically —
this is the "cuts the corner" signature in the webm. Meanwhile `propagateState()` maintains a
perfectly good IMU-propagated `state.v.lin.w` that is never consulted.

Fix: on rejected scans seed `prev_vel` from `state.v.lin.w` (and consider seeding
`basePose` from `state.p/q` rather than raw `T_prior`), or have `propagateState()` update
`geo.prev_vel` when no fresh GICP correction has occurred.

### RC3 — Observer corrects the *current* state toward a *stale* measurement
`updateState()` pulls `state` toward the GICP candidate, which is the pose at the scan's
median point time. By the time the correction is applied, the state has advanced by
half-sweep + queueing + GICP solve time (gicp_ms median 18 ms, p99 68 ms, max 250 ms) —
roughly 0.1–0.3 s total. On straights this mostly costs along-track lag; in a 30°/s turn it
drags yaw backwards by 3–9° worth of measurement staleness every scan, with gain
`dt_eff·Kq ≈ 0.4–0.6`. This matches the accepted-frame yaw-rate-proportional error table and
is exactly "bad control of yaw gain updates."

Fix: apply the GICP result as a **delta**: `T_corr = candidate · T_prior⁻¹` (both are at
median scan time, so `T_corr` is a time-free world-frame correction), and left-apply the
proportionally-gained `T_corr` to the *current* state — or FAST-LIO style, re-propagate from
the corrected scan-time state through buffered IMU up to now. Either removes the systematic
turn lag without touching the gains.

### RC4 — Gyro bias never applied on the prior/deskew path
`callbackImu` buffers raw (frame-rotated) `ang_vel`; `propagateState()` subtracts
`state.b.gyro`, but `integrateImu()`/`integrateImuInternal()` (which build `T_prior` and the
per-point deskew frames) integrate the **uncorrected** gyro. Upstream DLIO subtracts the bias
in the IMU callback before buffering. With RTK calibration writing a one-shot `state.b.gyro`
and `Kgb=0`, the two integration paths permanently disagree by the bias. Small with the
Atlas-calibrated IMU, but it biases deskew rotation most exactly during high-yaw-rate sweeps.
Fix: subtract `state.b.{gyro,accel}` when constructing `ImuMeas` (or inside `integrateImu`).

### RC5 — Aux-lidar merge coverage is poor, starving corner geometry
Throttled merge log samples: 144 × `2/2`, 168 × `1/2`, 101 × `0/2` — only ~35% of sampled
scans got all three lidars, ~24% got front-only. The left/right Luminars supply the lateral
structure that conditions the hessian at corners; every front-only corner scan raises the
condition number and feeds RC1. GICP uses `buffer_size: 20` vs GLIM's 200 for the same
threshold (0.1 s). Fix: raise the buffer, log per-frame merged-source count into the debug
bag (the audit explicitly couldn't reconstruct it), and investigate whether the P1-clock
timestamps of left/right arrive with a constant offset that a per-aux offset could absorb.

### RC6 — Recovery dynamics after streaks
`Kq=4.0` (τ≈0.25 s) needs 3–4 scans to absorb the yaw error accumulated in a streak, while
`Kv=11.25 · err` simultaneously injects a large velocity spike from the position error —
visible as post-corner overshoot/oscillation in the webm. Consider confidence-scheduled
gains (scale Kp/Kv/Kq by fitness/hessian quality per accepted scan) instead of fixed gains,
and clamp `max_vel_correction` (currently 0 = disabled).

Also: `gt_recovery/min_consecutive_failures=1` masks divergence in replays. Keep it for
debugging, but validation runs should also be scored with it disabled, otherwise streak
damage is invisible in median stats.

## Root causes (GLIM / map side)

The cross-run fitness floor (~0.27 in both directions; run 13 is worse: 80% acceptance, 204
fitness rejections) means the run3 and run5 maps disagree systematically — this multiplies
every GICP-side issue.

- **Map density**: 1.62M points for a full track (preprocess `random_downsample_target=30000`,
  `downsample_resolution=1.0`, plus 0.3 m localization voxel) is thin. GICP fitness ≈ mean-sq
  correspondence distance; 0.27 ≈ 0.52 m mean distance is consistent with density mismatch
  against 0.5 m-voxel scans. Export a denser localization map (≈0.15–0.2 m) and re-measure the
  fitness floor before re-tuning any gate.
- **`fix_imu_bias: true`** in odometry_gpu: gyro-bias drift during mapping is uncorrected
  between GNSS factors, and it expresses as yaw error precisely at corners — baked into the
  map differently per run. With the Atlas IMU this is defensible, but worth an A/B with bias
  estimation on.
- **Dual-antenna heading is unused as a factor**: P1 Atlas provides RTK heading, but the
  GNSS global module constrains position only. Adding heading/attitude priors to global
  mapping would pin per-run map yaw and directly shrink cross-run corner disagreement.
- **IMU placement approximation**: `urdf_imu_frame: gps_antenna_top` while the LG69T sits at
  `pointonenav` (~0.63 m away) — the ω×(ω×r) term (~0.06 m/s² at 0.3 rad/s) is knowingly
  dropped in both GLIM and GICP. Second-order for now, but cheap to model exactly by
  re-pointing the frame and letting the existing lever-arm code run.

## Priority order

1. RC2 (stale `prev_vel`) — small bug fix, directly attacks corner-cutting during streaks.
2. RC1 (gate recalibration / partial updates) — eliminates the 25 s dead-reckon streaks.
3. RC3 (delta-form correction) — removes accepted-frame yaw-rate-proportional error.
4. RC5 (3-lidar coverage + per-frame source logging) — reduces degeneracy at the source.
5. Map density export + re-baseline fitness floor, then re-tune thresholds.
6. RC4, RC6, GLIM heading factors, IMU frame — follow-ups.

## Validation recipe

Replay the same run3/run5 cross pairs and compare: (a) accepted-frame gt_pos p95/p99,
(b) the yaw-rate-bucket table above, (c) streak-length distribution, (d) a run with
`gt_recovery/enable=false` to expose true divergence. The bucket/streak analysis is a
~30-line script over `SCAN DEBUG` lines; worth committing so every run report includes it.
