# GICP++ Localization

Parallel small_gicp-backed scan-to-map localization with IMU dead-reckoning and
optional ground-truth-driven recovery. This package intentionally lives beside
`gicp_localization` so the existing NanoGICP implementation remains available
for A/B replay comparisons. It locks onto a pre-built PCD map produced by GLIM
(or any compatible source) and publishes pose at IMU rate. Designed for AV-24
Cybertruck-class platforms with multi-LiDAR setups.

## Pipeline context

GICP is the online-localization stage of a four-stage pipeline:

```
adapter (Atlas WGS84 → local-ENU, publishes /gps_p1/*)
   → scripts/prep_bag.py (normalized bag)
      → GLIM (offline map, PCD)
         → gicp_plusplus (online localize)
```

The `adapter` package converts raw Atlas WGS84 fixes into a fixed local-ENU
datum (Putnam origin from the `race_metadata` TTL) and republishes `/gps_p1/*`
(including the ENU-framed seed `/gps_p1/filtered_odom`). `scripts/prep_bag.py`
produces a normalized bag. GICP consumes the GLIM-built PCD map plus that
ENU-framed seed. GICP is **frame-agnostic**: it localizes the scan against the
PCD map and reports the pose in whatever frame the map is in — with the adapter,
that frame is local ENU. See the `adapter` package and `scripts/prep_bag.py` for
how the map inputs and the seed are produced.

## Features

- **small_gicp GICP scan-to-map matching** against a single pre-built PCD map (no submap stitching at runtime).
- **IMU + LiDAR pipeline**: IMU integrates a motion prior between scans; GICP refines; a geometric observer fuses the two and propagates pose at IMU rate (~100 Hz).
- **Multi-LiDAR concatenation** (`lidar_concat`): 3x Luminar (`luminar_front` primary + `luminar_right`/`luminar_left` merged); time-aligns aux LiDARs to the primary, transforms them via offline-resolved extrinsics, and concatenates per-point timestamps onto the primary clock. A strict merge guard (`require_all_aux` / `abort_on_merge_failure`, identical semantics + defaults to GLIM) controls whether an incomplete merge degrades or skips the scan.
- **Confidence-weighted gating** (P1 rework, 2026-07 — replaces the old binary gates; see `docs/action_plan_turn_error_20260704.md` for the evidence):
  - Hard fitness reject (`gicp/fitnessRejectThreshold`) — catastrophic backstop, unchanged.
  - **Per-map fitness-ratio gates** (`gicp/fitnessBaseline/*`, `fitnessRatioRejectThreshold`): gates operate on fitness divided by a rolling median of accepted-frame fitness, so they survive cross-run maps whose absolute fitness floor differs 5–10× from the calibration map. `seedBaseline` keeps them live during warm-up.
  - **Degeneracy partial update** (`gicp/degeneracy/*`): when the hessian condition proxy trips `hessianCondMax`, the correction is projected onto well-constrained eigen-directions of the vehicle-re-centered, unit-scaled 6×6 hessian (full-6D by default — coupled rot/trans null directions included) and the IMU prior is kept along degenerate axes. Accepted-with-projection logs `status=ok_partial`; wholesale `rejected_hessian` remains only for the all-axes-degenerate case. Legacy binary gate available via `degeneracy/partialUpdate: false`.
  - **Yaw-consistency veto** (`gicp/yawGate/*`, independent of partialUpdate): a GICP yaw correction > `maxCorrDeg` vs. the IMU-integrated prior on a low-confidence match (ratio > `fitnessRatio`) keeps the IMU yaw — the wrong-basin *entry* signature the jump gate can't see.
  - Large-jump reject (compares the applied candidate to the IMU-predicted prior; speed/scan-dt-aware thresholds).
- **IMU dead-reckoning fallback**: any non-accepted scan falls back to the IMU-integrated prior instead of freezing at the last accepted pose, seeded with the *current* IMU-propagated velocity (P2 fixed a stale-velocity bug that made multi-scan rejection streaks cut corners).
- **Ground-truth divergence cross-check** (optional): subscribes to a `gt_odom` topic, computes per-scan `gt_err=[trans,rot,dt]` against the pose actually applied (post-projection), publishes deltas. Diagnostic only — never feeds back into accept/reject.
- **GT-driven pose recovery**: when GICP fails for N consecutive scans (default 5), snap pose+twist to a time-matched GT sample (composed through TF into `base_frame`) so GICP can re-acquire from a known-good state. Twist sources resolve independently (P2): angular rate backfills from the live bias-corrected gyro and linear velocity from GT pose finite-differencing when the odom twist is unpopulated — never zeroing a moving vehicle. Falls back to dead-reckoning when GT is unavailable.
- **GT-bootstrapped initial pose** (optional): take the first GT message as the initial pose so the node starts at the right location regardless of bag offset.
- **Local-ENU output** (operational contract): the primary `map_frame` pose / odom / path are already in the map's frame, which — with the adapter — is a fixed local-ENU datum (Putnam origin from the `race_metadata` TTL). GICP itself is frame-agnostic and simply reports the pose in the map's frame.
- **UTM-frame output** (optional legacy layer): only active if `localization/utm_transform_path` is set; when provided, publish pose / odom / path in `utm` frame alongside `map`. Not the default.
- **RViz visualization** of map, aligned scan, pose, debug clouds and markers.

## Dependencies

- ROS 2 Jazzy
- PCL, Eigen3, OpenMP, nlohmann::json
- `PointType` and a vendored MIT-licensed `small_gicp` header snapshot ship
  inside this package; no separate `small_gicp` or
  `direct_lidar_inertial_odometry` dependency is required.
- The backend includes the Phase-1 work-plan hooks: DoF restriction and a
  soft IMU-attitude prior are applied inside the small_gicp linearized solve.
- For development: matplotlib (debug-script plots)

## Building

This package lives in the `DLIO_plusplus` workspace. From the repo root:

```bash
colcon build --packages-select gicp_plusplus --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage

### Launch

The single supported launch file is `localization_with_tf.launch.py`. It starts the localization node alongside `robot_state_publisher` (which publishes the URDF transforms the node depends on for sensor extrinsics).

```bash
ros2 launch gicp_plusplus localization_with_tf.launch.py \
    rviz:=true \
    pointcloud_topic:=/luminar_front/points \
    imu_topic:=/gps_p1/imu \
    gt_odom_topic:=/gps_p1/filtered_odom
```

### Launch arguments

| Arg | Default | Purpose |
|---|---|---|
| `rviz` | `false` | Launch RViz with the bundled config. |
| `pointcloud_topic` | `/luminar_front/points` | Primary LiDAR topic (gets remapped to `pointcloud`). |
| `imu_topic` | `/gps_p1/imu` | Point One Atlas `imu_calibrated` (sensor-calibrated, gravity present, 99 Hz, frame `gps_antenna_top`). **Watch for typos**: it's `imu_topic` (underscore), not `imu-topic`. |
| `odom_topic` | `/odom` | Pose-init odom topic when `localization/use_odom_init=true` and not bootstrapping from GT. |
| `gt_odom_topic` | `/gps_p1/filtered_odom` | Atlas FusionEngine INS odometry, at `gps_antenna_top`. Used when `localization/gt_odom/enable=true` and/or `gt_recovery/enable=true`. Same frame as `base_frame`, so no TF correction is needed. |
| `imu_only` | `false` | Disable GICP and propagate pose from IMU only (debug/sanity check). |
| `urdf_path` | (auto-found) | Path to the URDF (`av24.urdf`) used for offline extrinsic resolution. The launch resolves it by walking up from the launch dir; `av24.urdf` is also installed into `share/gicp_plusplus`. |
| `parent_frame` / `child_frame` | `base_link` / `luminar_front` | Used by the bundled static-TF helper. |
| `map_path` | (yaml) | Override the yaml `localization/map_path` from the command line. |

### Frame conventions (all-P1 single-source design)

Every comparison the node performs lives at the Atlas antenna phase centre (URDF link `gps_antenna_top`):

- `localization/base_frame: gps_antenna_top` — GICP's state is reported at this frame.
- `localization/imu_frame: gps_antenna_top` — the `/gps_p1/imu` stream is tagged with this frame. Physically the IMU (`IMUOutput`, FusionEngine Spec §3.4.1) is body-axis-rotated but **device-located** (`pointonenav`), not antenna-projected; setting `imu_frame = base_frame` treats it as co-located with the antenna — a deliberate approximation that drops the small device→antenna accel lever arm (see [`localization.yaml`](cfg/localization.yaml)).
- `gt_odom_topic` → `/gps_p1/filtered_odom` — Atlas INS pose, `child_frame_id="gps_antenna_top"`.

Because base_frame, imu_frame, and the gt_odom source all align, the in-code TF lookups in `callbackImu` (`baselink2imu_T`) and `callbackGtOdom` (`T_base_gtbody_`) degenerate to identity. No lever-arm work happens anywhere; the cross-check `gt_pos_err_m` is exact (no constant baseline bias); `applyInitialPose` correctly seeds the state; the snap helper composes a no-op identity TF.

This design deliberately bypasses race_common's downstream `cg`-frame intermediate (VKS / robot_localization). The trade-off is the localized pose lives at the antenna point rather than the controller-expected `cg` (downstream consumers need an extra `gps_antenna_top → cg` TF lookup, which `robot_state_publisher` already provides). See `docs/GICP_GNSS_IMU_bug_report.pdf` for the architectural alternatives and their trade-offs.

### RTK quality gate — consumer-side, with snap-recovery exemption

GICP uses Atlas's per-sample pose covariance as a quality signal, but the **gate is consumer-specific**. Every `/gps_p1/filtered_odom` message is pushed into the GT buffer unfiltered; the FIXED-quality check is applied where each consumer reads.

```yaml
localization/rtk_gate/enable:           true   # inspect msg->pose.covariance per consumer
localization/rtk_gate/max_pose_var_xy:  0.25   # m^2 (~0.5 m horizontal std)
localization/rtk_gate/max_pose_var_z:   1.0    # m^2 (~1.0 m vertical std)
```

| Consumer | Requires FIXED? | Why |
|---|---|---|
| **`tryRtkCalibrationStep`** — RTK-driven IMU bias calibration at startup | ✓ Yes | Needs cm-level truth to estimate gyro/accel bias residuals. If only degraded samples are available the init machine times out and falls back to stationary calibration. |
| **Scan cross-check** — diagnostic `gt_pos_err_m` published on every accepted scan | ✓ Yes | A diagnostic comparing GICP against a sub-cm reference is only meaningful when the reference IS sub-cm. |
| **`maybeSnapPoseToGT`** — recovery after GICP loses LiDAR features | ✗ **No — accepts any sample** | When GICP can't match the LiDAR scan, the next-best truth is Atlas's pose at whatever quality it currently has — not our own software IMU dead-reckoning. See the next subsection. |
| **`applyInitialPose` (use_odom_init)** | ✗ No | Falls back to whatever Atlas reports at startup; if RTK FIXED is required for init, set `localization/rtk_init/enable: true` (default) which gates through `tryRtkCalibrationStep`. |

Reference Atlas covariance on AV-24 RTK-FIXED: median `cov_xx`≈2.8e-5, `cov_yy`≈4.2e-5, `cov_zz`≈1.0e-4 m². RTK-FLOAT typically 1e-2…1e-1 m². GPS-only ≥ 1 m². The defaults above admit anything down to RTK-FLOAT for the consumers that need FIXED — tighten if you want strict FIXED-only for those paths.

Setting `rtk_gate/enable: false` makes every consumer (including calibration and cross-check) treat all samples as FIXED — useful only for bag-replay diagnostics.

### Snap recovery accepts any Atlas pose (the "GNSS is always next-best" policy)

When GICP fails to converge — typically a feature-poor scene (long straight under glass facades, tunnel, fog, sensor occlusion) — the recovery path is:

```
GICP scan-match fails
        │
        ▼
maybeSnapPoseToGT(): take latest /gps_p1/filtered_odom sample
   from buffer, regardless of RTK quality, and snap state.{p,q,v}
        │
        ▼
(only if no GT sample available at all)
software IMU dead-reckoning until GICP recovers
```

**The snap is intentionally gate-bypassed.** Atlas LG69T is a *tightly integrated RTK+IMU INS*: when RTK degrades, Atlas's internal solution continues to fuse its own calibrated IMU with whatever GNSS quality it has, producing a continuously-valid pose. That solution is strictly better than our software IMU dead-reckoning, which:

- Uses a less-rigorous integrator (DLIO's geometric observer, not a tightly-coupled EKF).
- Has no GNSS-aided bias correction during the dead-reckoning window.
- Compounds frame-extrinsic errors that Atlas's firmware already eliminates internally.

So the policy is: **when GICP cannot match the scan, snap to Atlas's pose at whatever quality Atlas currently has — FIXED, FLOAT, or even pure INS dead-reckoning — before falling further back to our own integrator.**

⚠️ **This policy depends on the GNSS source being a tightly integrated RTK+IMU INS.** It is safe for Atlas (which fuses GNSS + onboard IMU and explicitly outputs a continuously-valid INS pose). It would **NOT** be safe for a raw GNSS receiver that publishes nothing during RTK loss, or a loosely-coupled fusion that jumps when satellites lock back. If you ever re-point `gt_odom_topic` at a non-INS source, re-enable the strict consumer-side FIXED check in `maybeSnapPoseToGT` by adding a `gtSampleIsRtkFixed(gt)` guard to its lookup.

Operator-facing log lines:

- `snap fired: GT covariance was [cov_xx=… cov_yy=… cov_zz=…] (FIXED|degraded) — snapped state to gt at t=…` — appears every time the snap fires; the quality label tells you whether Atlas was RTK-FIXED at that moment.
- `IMU dead-reckoning fallback: no GT sample within max_dt of scan stamp` — only when even Atlas isn't publishing (true GNSS-denied + INS publication gap).

In normal operation on a well-mapped track you should rarely see either: GICP scan-match converges on every scan and `consecutive_failures_` resets to 0. The snap path exists for the edge case where LiDAR briefly cannot disambiguate the local map.

### Setting an initial pose

Three options, in priority order:

1. **`localization/gt_odom/enable: true` + `localization/use_odom_init: true`** (default in the shipped yaml): the first GT odom message bootstraps the pose. Works for any bag start-offset without hand-tuning numbers.
2. **`localization/initial_pose/use: true`**: use the numeric `x/y/z/roll/pitch/yaw` from the yaml. The `frame: "lidar"` mode is convenient for pasting from GLIM's `traj_lidar.txt` — the node post-multiplies `inv(T_base_lidar)` automatically.
3. **RViz "2D Pose Estimate"**: publish to `/initialpose`. Always available as a manual override.

## Configuration

All parameters live in `cfg/localization.yaml`. The yaml has inline comments explaining each knob; the cheat sheet below covers the parts most worth tuning.

### Frames

```yaml
localization/map_frame:    "map"               # with the adapter, this IS a local-ENU frame (fixed datum)
localization/base_frame:   "gps_antenna_top"  # body the node tracks; URDF link
localization/imu_frame:    "gps_antenna_top"  # IMU URDF link (matches /gps_p1/imu header)
localization/lidar_frame:  "luminar_front"    # primary LiDAR URDF link
```

GICP is frame-agnostic: it reports the pose in whatever frame the map is in.
With the adapter, `map` is a **local-ENU** frame (fixed datum, Putnam origin from
the `race_metadata` TTL), and the seed `/gps_p1/filtered_odom` is in that same
ENU frame. The `map`, the seed, and GICP must all share the one datum the adapter
defines — a single-datum consistency requirement. UTM publishing is an optional
legacy layer (see below), not the operational contract.

### GICP gating (P1 confidence-weighted rework)

```yaml
gicp/fitnessRejectThreshold: 1.0          # hard reject: fitness > threshold (catastrophic backstop)

# Per-map fitness normalization — gates operate on fitness / rolling-median
# of ACCEPTED-frame fitness, so they survive cross-run maps whose absolute
# floor differs 5-10x from the calibration map:
gicp/fitnessBaseline/enable: true
gicp/fitnessBaseline/window: 201          # rolling-median window (~20 s @ 10 Hz)
gicp/fitnessBaseline/minSamples: 50       # rolling median takes over after this
gicp/fitnessBaseline/seedBaseline: 0.28   # warm-up baseline so gates are live from frame 1 (re-measure per map!)
gicp/fitnessRatioRejectThreshold: 2.0     # wrong-basin gate: reject when ratio exceeds this

# Degeneracy partial update (replaces the old binary hessian reject):
gicp/hessianCondMax: 5.0e9                # TRIGGER: when tripped, project instead of reject
gicp/degeneracy/partialUpdate: true       # false = legacy binary combined gate below
gicp/degeneracy/full6d: true              # coupled 6x6 remapping (vs blockwise 3x3 A/B)
gicp/degeneracy/couplingLengthM: 20.0     # lever arm making rad/m commensurable
gicp/degeneracy/relFloor6d: 0.02          # eigen-axis degenerate if lambda < floor*lambda_max

# Turn-aware yaw-consistency veto (independent of partialUpdate):
gicp/yawGate/enable: true
gicp/yawGate/maxCorrDeg: 1.5              # veto yaw corr above this vs IMU prior...
gicp/yawGate/fitnessRatio: 1.2            # ...on low-confidence matches

gicp/rejectLargeJumps: true               # reject if applied pose jumps > thresholds (speed/dt-aware)

# LEGACY combined-gate warn floors — consulted only when partialUpdate=false.
# ABSOLUTE values calibrated on a same-run map; they go stale on cross-run maps.
gicp/hessianFitnessWarnThreshold: 0.15
gicp/hessianTransWarnM: 1.0
gicp/hessianRotWarnDeg: 1.5
```

When `hessianCondMax` trips, the GICP correction is eigendecomposed on the
vehicle-re-centered, unit-scaled 6×6 hessian and applied only along
well-constrained directions (the IMU prior holds the degenerate ones) —
`status=ok_partial`. The old behavior (reject the whole scan) produced
253-frame dead-reckoning streaks on cross-run replays; wholesale
`rejected_hessian` now fires only when all six axes are degenerate. The
fitness-ratio gate catches the opposite failure (wrong-basin matches accepted
with good-looking fitness). Rationale, measurements, and thresholds:
`docs/action_plan_turn_error_20260704.md`; score any replay with
`scripts/analyze_scan_debug_log.py` (it also suggests re-baselined ratio
thresholds per map).

### Multi-LiDAR concatenation

3x Luminar: `luminar_front` primary + `luminar_right`/`luminar_left` merged.

```yaml
localization/lidar_concat/enabled:        true
localization/lidar_concat/aux_topics:     ["/luminar_right/points", "/luminar_left/points"]
localization/lidar_concat/aux_frames:     ["luminar_right", "luminar_left"]
localization/lidar_concat/time_threshold: 0.1     # drop aux scans further than this from primary
localization/lidar_concat/buffer_size:    200     # per-aux ring depth (P4: raised from 20 — 2 s of history silently degraded frames)

# Strict merge guard — IDENTICAL semantics + defaults to GLIM:
localization/lidar_concat/require_all_aux:                    false  # false = localize on whatever LiDARs merged; true = incomplete merge SKIPS the scan (degraded cloud never registered; IMU propagation continues)
localization/lidar_concat/abort_on_merge_failure:            true   # only relevant when require_all_aux=true: abort node past budget vs keep skipping non-fatally
localization/lidar_concat/max_consecutive_aux_merge_failures: 10
```

**Offline extrinsic resolution (no live `/tf_static` needed).** Aux extrinsics
are resolved offline, in priority order: URDF (`av24.urdf` via
`lidar_concat/urdf_path`, passed by the launch which resolves it by walking up
from the launch dir; also installed into `share/gicp_plusplus`) > a static
per-aux 4x4 (`aux_static_transforms`, baked from `av24.urdf`) > live TF as a last
resort. The `base_frame ← lidar_frame` lever arm is resolved the same way
(URDF > `localization/base_lidar_transform` static matrix > live TF), so full
localization needs no `/tf_static`.

**Map voxel downsample + crop box.** The GICP target map is voxel-downsampled at
load (`localization/map_voxel_size: 0.3`) before building the kd-tree, to bound
memory. The crop box runs in the **sensor frame** before deskew. Live scans are
voxel-filtered at `dlio/preprocessing/voxelFilter/res: 0.3` (P4: lowered from
0.5 — the sparser scans starved GICP of yaw-constraining geometry at corners;
watch `gicp_ms` p99 against the 100 ms scan period if you densify further).

**Merge diagnostics (P4).** Every processed frame records its LiDAR source set:
debug topics `merged_aux_count`, `aux<i>_merge_dt_s` (signed, NaN = not
merged), `aux<i>_points`, `scan_time_span_s`, plus the same fields in the
`SCAN DEBUG` line (`concat=[n/2,dt0=…,pts0=…,…,span=…]`). Per-aux signed
header-offset stats are summarized every 512 merges with a warning when the
mean exceeds 20 ms — the constant-clock-offset signature worth absorbing
upstream.

### Ground-truth diagnostics + recovery

```yaml
localization/gt_odom/enable:        true
localization/gt_odom/buffer_size:   200      # ~2 s of history at 100 Hz
localization/gt_odom/max_dt:        0.1      # max scan-to-GT lookup gap

localization/gt_recovery/enable:                   true    # snap to GT after sustained GICP failure
localization/gt_recovery/min_consecutive_failures: 5       # snap after N consecutive non-accepts (P2: raised from 1 — per-frame snapping masked dead-reckoning quality)
```

When `gt_recovery/enable=true`, the node caches the `base_frame ← child_frame_id` TF on the first GT message and uses it to compose snap poses into `base_frame` (so the snap lands at the same reference point GICP normally tracks).

### IMU + observer

```yaml
dlio/deskew: true                  # Luminar per-point epoch-ns timestamps drive motion deskew
dlio/imu/bufferSize: 2000
dlio/imu/calibTime: 0.5            # initial stationary calibration window

odom/geo/Kp: 4.5                   # Position correction gain
odom/geo/Kv: 11.25                 # Velocity
odom/geo/Kq: 4.0                   # Orientation
odom/geo/Kab: 0.0                  # Online accel-bias adaptation disabled
odom/geo/Kgb: 0.0                  # Online gyro-bias adaptation disabled
odom/geo/delta_correction: true    # P3: apply GICP as a time-free delta (see below)
```

`Kab`/`Kgb` are intentionally zero for the fused Point One (Atlas) INS path. Initial
RTK/stationary calibration may still seed `state.b`, but GICP residuals do not
continue rewriting IMU bias online unless these gains are explicitly raised.

**Delta-form correction (P3).** The GICP measurement is stamped at the scan's
median point time — 0.1–0.3 s before the observer applies it. The legacy
absolute target dragged the current state backwards toward that stale pose,
which is zero-mean on straights but a systematic yaw/position lag in turns
(accepted-frame gt_err doubled from 1.0 m at <2°/s to 2.1 m at >25°/s on the
run-12 baseline). With `delta_correction: true` the observer instead applies
the time-free correction `T_meas · T_prior⁻¹` to the current state: perfect
IMU/GICP agreement produces zero correction at any latency. Gains unchanged.

**Bias path (P3).** IMU biases are subtracted **once, at buffering** in
`callbackImu`, so `propagateState`, the scan prior (`integrateImu`), and
per-point deskew all integrate the same corrected signal. (Previously only
`propagateState` subtracted; the prior/deskew path integrated raw gyro and
diverged once RTK calibration set a nonzero bias.) Calibration still consumes
the raw values.

**Per-point timestamps and deskew.** Luminar per-point timestamps are `UINT8[8]`
= a `uint64` PTP epoch in nanoseconds (per the *Luminar Iris Data Output
Specification v1.3.0*); only 8-byte absolute carriers (`UINT8[8]` / `FLOAT64`)
are accepted, and `UINT32` is intentionally rejected. With `dlio/deskew: true`,
GICP deskew is header-anchored on the **primary** scan's earliest timestamp with
a **signed** offset — so a merged aux scan that began before the primary gets a
correct negative offset.

## Topics

### Subscribed

| Topic (remap) | Type | Purpose |
|---|---|---|
| `pointcloud` | `sensor_msgs/PointCloud2` | Primary LiDAR scans. |
| `imu` | `sensor_msgs/Imu` | IMU data. Required for the motion prior and observer. |
| `<aux_topics>` | `sensor_msgs/PointCloud2` | Aux LiDAR scans (when `lidar_concat/enabled=true`). |
| `gt_odom` | `nav_msgs/Odometry` | Ground-truth pose for cross-check / recovery / bootstrap. |
| `odom` | `nav_msgs/Odometry` | External odom for init (when `use_odom_init=true` and GT not used). |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz initial pose override. |

### Published

| Topic | Type | Purpose |
|---|---|---|
| `localized_pose` (`gicp/localization/pose`) | `geometry_msgs/PoseStamped` | Localized pose (scan rate). |
| `localized_odom` (`gicp/localization/odom`) | `nav_msgs/Odometry` | Localized odom propagated at IMU rate (~100 Hz). |
| `localized_path` (`gicp/localization/path`) | `nav_msgs/Path` | Trajectory history. |
| `gicp/localization/pose_utm` / `odom_utm` / `path_utm` | (same types) | Optional legacy UTM-frame mirrors, only when `utm_transform_path` is set (the primary `map`-frame outputs above are already local ENU with the adapter). |
| `aligned_cloud` (`gicp/localization/aligned_cloud`) | `sensor_msgs/PointCloud2` | Aligned scan in `map`. |
| `map` (`gicp/localization/map`) | `sensor_msgs/PointCloud2` | Downsampled visualization map. |
| TF: `map → base_frame` | | Published when `publish_tf=true`. |

### Debug topics (require `localization/debug/enable_pub: true`)

Per-scan scalar metrics on `gicp/localization/debug/*`:

- `fitness`, `gicp_elapsed_ms`, `final_error`, `corr_norm`, `scan_dt`
- `imu_age`, `imu_buffer_span_s`, `scan_to_latest_imu_lag_s`
- `num_correspondences`, `correspondence_ratio`
- `guess_to_solution_m`, `guess_to_solution_deg`
- `guess_from_last_m`, `guess_from_last_deg`
- `jump_trans`, `jump_rot_deg` (raw GICP-vs-prior disagreement, pre-projection)
- `hessian_condition_proxy`
- **P1 gating**: `fitness_ratio` (−1 during warm-up without seed), `degen_rot_axes`, `degen_trans_axes`, `yaw_veto`
- **P4 concat**: `merged_aux_count` (−1 = concat disabled), `aux<i>_merge_dt_s` (signed; NaN = not merged), `aux<i>_points`, `scan_time_span_s`
- `gt_pos_err_m`, `gt_rot_err_deg` (when GT is enabled; measured against the pose actually applied)
- `converged` (Bool)

Plus pose / cloud topics: `initial_guess_pose`, `final_pose` (post-projection), `input_cloud_base`, `initial_guess_cloud`, `pose_markers`.

`enable_pub` and `verbose_scan_log` are **on by default** so every replay
produces this evidence; score a run's `localization.log` with
`scripts/analyze_scan_debug_log.py` (status/streaks, gicp_ms percentiles,
fitness floor + suggested ratio thresholds, gt_err yaw-rate buckets, concat
coverage).

## Algorithm

```
   IMU ──→ buffer ─────────────────────────────┐
                                                ↓
   LiDAR ──→ lidar_concat ──→ preprocess ──→ T_prior = integrate(IMU, last lidarPose)
                                                ↓
                                              GICP align (initial guess = T_prior)
                                                ↓
                            gate: fitness / fitness-ratio / degeneracy-projection / yaw-veto / jump
                                ┌─── accepted (ok | ok_partial) ─┴── rejected ──┐
                                ↓                                                ↓
                          updateState (geo observer,                dead-reckon: lidarPose ← T_prior,
                          delta-form target)                        prev_vel ← state.v (current)
                                ↓                                                ↓
                          state ← merge(GICP, IMU)                  consecutive_failures++
                                ↓                                                ↓
                                                       ≥ N consecutive AND gt_recovery on?
                                                                     ↓
                                                       maybeSnapPoseToGT(reason)

                          propagateState (every IMU sample) → publish odom/TF at ~100 Hz
```

### Status taxonomy

- **Accepted**: `ok` (full GICP correction) and `ok_partial` (P1: degeneracy
  projection and/or yaw veto shrank the correction; the projected pose is what
  gets applied, published, and GT-scored).
- **Rejected**: `failed_to_converge`, `rejected_fitness` (absolute),
  `rejected_fitness_ratio` (P1 wrong-basin gate), `rejected_hessian` (now only
  the all-axes-degenerate case), `rejected_jump`, `invalid_solution` — all fall
  through to the dead-reckoning branch (set `current_pose ← T_prior`, seed
  `prev_vel` from the current IMU-propagated velocity, increment streak
  counter, optionally trigger snap).
- `last_gicp_pose_` is **not** updated on rejection, so the IMU prior on the next scan is still anchored to the last successfully-matched GICP pose.

## Map preparation

GLIM dumps submaps as PLY-format files with a `.pcd` extension. Convert them with:

```bash
python3 gicp_plusplus/scripts/convert_ply_to_pcd.py \
    /path/to/glim_map.pcd \
    /path/to/output_map.pcd
```

The operational contract is **local ENU**: with the adapter, the GLIM-built map,
the seed `/gps_p1/filtered_odom`, and GICP all share the one ENU datum the adapter
defines, and the primary `map`-frame outputs are already ENU — no extra transform
is needed. UTM output is an **optional legacy layer**: only if you set
`localization/utm_transform_path` (e.g. at GLIM's `T_world_utm.txt` from the same
dump) does the node also publish the `utm`-frame mirrors. It is not the default.

## Troubleshooting

### "IMU never received" / pose stuck in dead-reckoning

Symptoms: `imu_buffer_span=-1.000s` in SCAN DEBUG, `guess_from_last=0`, no "First IMU message received" log.

Almost always a topic-mismatch problem. The launch arg is `imu_topic` (underscore). Passing `imu-topic:=/X` silently does nothing — the launch falls back to default `/gps_p1/imu` and your IMU subscription stays empty. Verify with:

```bash
ros2 topic list | grep -i imu
ros2 topic hz <your_imu_topic>
```

### GT recovery enabled but no snap fires

Look in the log for one of:

- `GT recovery: deferring snap — no GT odom received yet` → topic is wrong / not publishing.
- `GT recovery: deferring snap — base→<frame> TF not cached yet` → URDF doesn't include the GT publisher's `child_frame_id`. Add the link or repoint the publisher to a frame already in the URDF.
- `GT recovery: deferring snap — no GT sample within max_dt=...` → GT publishes slower than `gt_odom/max_dt`; increase the parameter.

### Scan dropouts during sharp turns

If you see SCAN DEBUG gaps > 200 ms during turns, `lidar_concat/time_threshold` is dropping aux scans that fell out of sync. Try raising it from the default `0.1` toward `0.15`. The merged cloud will have slightly worse intra-frame alignment but that's almost always cheaper than a 600 ms scan-stream gap during cornering. (If `require_all_aux: true`, an out-of-sync aux instead SKIPS the whole scan rather than degrading it.)

### GICP slides at corners

With the P1 partial-update gating, degenerate corners no longer binary-reject —
watch `degen_rot_axes`/`degen_trans_axes` and `yaw_veto` in the debug topics
(or `degen=[…]` in SCAN DEBUG) to see the projection engaging. Tunable knobs
(in order of impact):

1. Lower `gicp/hessianCondMax` to engage the eigen-projection earlier (default 5e9), or raise `gicp/degeneracy/relFloor6d` to zero out weaker axes more aggressively (default 0.02).
2. Lower `gicp/yawGate/maxCorrDeg` / `yawGate/fitnessRatio` to veto suspicious yaw corrections earlier (defaults 1.5° / 1.2).
3. Lower `gicp/fitnessRatioRejectThreshold` to reject wrong-basin matches earlier (default 2.0) — at the cost of more dead-reckoned frames; check the streak histogram in the scorecard after changing it.
4. If rejection still cascades, `gt_recovery` (on by default, N=5) recovers at corners.

(Legacy knobs `hessianTransWarnM`/`hessianRotWarnDeg`/`hessianFitnessWarnThreshold`
only apply with `degeneracy/partialUpdate: false`.)

### Frame check

`ros2 run tf2_tools view_frames` should show `map → gps_antenna_top` (`base_frame`) and the URDF chain `base_link → luminar_front`, `base_link → gps_antenna_top`, etc. The localization node tracks `base_frame`; everything else is just the URDF.

## Debug scripts

### Pose inspector

`scripts/debug_pose_inspector.py` prints / CSVs / plots the per-scan initial-guess vs final-pose deltas. Requires `localization/debug/enable_pub: true`.

```bash
python3 scripts/debug_pose_inspector.py --csv-path /tmp/gicp_pose_debug.csv
python3 scripts/debug_pose_inspector.py --csv-path /tmp/x.csv --no-plot
python3 scripts/debug_pose_inspector.py --csv-path /tmp/x.csv --max-samples 300
```

CSV columns: `current_*` (final pose), `guess_*` (initial guess), `delta_*` (`guess − current`), `delta_trans_norm`, `delta_rot_deg`.

### LiDAR topic visualizer

`scripts/visualize_lidar_topic.py` checks that the localization node is consuming the LiDAR topic you expect, by reconstructing the same cloud (TF + `flip_y`) and comparing against `gicp/localization/debug/input_cloud_base`.

```bash
python3 scripts/visualize_lidar_topic.py
python3 scripts/visualize_lidar_topic.py --topic /your/lidar/topic --expected-frame your_lidar_frame
```

If `mean_err`, `p95_err`, `max_err` stay near zero, the topic path is consistent.

### Profiling

`scripts/profile_localization_resources.py` and `scripts/plot_localization_profile.py` capture and chart per-scan resource usage and the full debug-topic time series. Useful for tuning real-time performance.

`scripts/plot_source_switches.py` plots when the node switches between GICP, dead-reckoning, and GT snap — handy when investigating snap behavior.
