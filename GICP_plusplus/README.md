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
   → scripts/prep_bag.py (normalized bag + GLIM dump by default)
      → scripts/export_glim_dump_to_pcd.py (ENU PCD + manifest)
         → gicp_plusplus (online localize)
```

The `adapter` package converts raw Atlas WGS84 fixes into a fixed local-ENU
datum (Putnam origin from the `race_metadata` TTL) and republishes `/gps_p1/*`
(including the ENU-framed seed `/gps_p1/filtered_odom`). By default,
`scripts/prep_bag.py` produces the normalized bag **and** a GLIM dump; export
that dump with `scripts/export_glim_dump_to_pcd.py` before launching GICP++.
GICP is **frame-agnostic**: it localizes the scan against the PCD map and reports
the pose in whatever frame the map is in — with this pipeline, that frame is
local ENU.

## Features

- **small_gicp GICP scan-to-map matching** against a single pre-built PCD map (no submap stitching at runtime).
- **IMU + LiDAR pipeline**: IMU integrates a motion prior between scans; GICP refines; a geometric observer fuses the two and propagates pose at IMU rate (~100 Hz).
- **Optional multi-LiDAR concatenation** (`lidar_concat`): the production
  perception-ws contract builds the map offline from all three LiDARs but runs
  online GICP on `luminar_front` only, so concat is disabled by default to meet
  the live 10 Hz deadline. Set `lidar_concat_enabled:=true` explicitly for a
  synchronization/diagnostic A/B. In that mode, raw epoch-ns carriers are
  matched by **absolute per-point time** (endpoint-range error ≤ 10 ms; header
  time only as tie-break). Laguna's ordinary FLOAT64 seconds-since-sweep-start
  carrier instead uses GLIM's safe future-header fallback, then rebases each
  auxiliary point time onto the primary header before deskew. An
  **asynchronous Luminar front worker/aux synchronizer** keeps DDS reception
  independent of GICP latency and prevents an early concat release from
  selecting the previous side sweep. A strict merge guard (`require_all_aux` /
  `abort_on_merge_failure`, identical semantics + defaults to GLIM) controls
  whether an incomplete merge degrades or skips the scan.
- **Map-independent deployment gating** (Laguna/perception-ws parity):
  - Absolute and rolling-ratio fitness rejection are disabled in the deployed
    default because their scale changes with map density, scene and speed. A
    finite high ceiling remains so NaN/Inf fail closed.
  - A 30% correspondence-support gate, finite-pose validation, physical
    jump/yaw limits and RTK candidate sanity/recovery remain active. The optional
    `fitnessBaseline/*` machinery is retained for controlled A/B tests.
  - **Degeneracy partial update** (`gicp/degeneracy/*`): when the hessian condition proxy trips `hessianCondMax`, the correction is projected onto well-constrained eigen-directions of the vehicle-re-centered, unit-scaled 6×6 hessian (full-6D by default — coupled rot/trans null directions included) and the IMU prior is kept along degenerate axes. Accepted-with-projection logs `status=ok_partial`; wholesale `rejected_hessian` remains only for the all-axes-degenerate case. Legacy binary gate available via `degeneracy/partialUpdate: false`.
  - **Yaw-consistency veto** (`gicp/yawGate/*`, independent of partialUpdate): a GICP yaw correction > `maxCorrDeg` vs. the IMU-integrated prior on a low-confidence match (ratio > `fitnessRatio`) keeps the IMU yaw — the wrong-basin *entry* signature the jump gate can't see.
  - Large-jump reject (compares the applied candidate to the IMU-predicted prior; speed/scan-dt-aware thresholds).
- **IMU dead-reckoning fallback**: any non-accepted scan falls back to the IMU-integrated prior instead of freezing at the last accepted pose, seeded with the *current* IMU-propagated velocity (P2 fixed a stale-velocity bug that made multi-scan rejection streaks cut corners).
- **Atlas divergence cross-check + production safety envelope** (optional):
  subscribes to `gt_odom`, computes per-scan
  `gt_err=[trans,rot,dt]` against the post-projection candidate and publishes
  the deltas. With `max_candidate_position_error_m: 0` it is diagnostic only.
  The Laguna deployment default is 5 m: a time-matched, RTK-quality Atlas
  sample can reject a GICP candidate outside that broad envelope, but is not
  blended into healthy poses inside it. This catches long-run repeated-track
  wrong basins before they poison the observer.
- **GT-driven pose recovery**: when GICP fails for N consecutive scans (default 5), snap pose+twist to a time-matched GT sample (composed through TF into `base_frame`) so GICP can re-acquire from a known-good state. Twist sources resolve independently (P2): angular rate backfills from the live bias-corrected gyro and linear velocity from GT pose finite-differencing when the odom twist is unpopulated — never zeroing a moving vehicle. Falls back to dead-reckoning when GT is unavailable.
- **GT-bootstrapped initial pose** (optional): take the first GT message as the initial pose so the node starts at the right location regardless of bag offset.
- **Local-ENU output** (operational contract): the primary `map_frame` pose / odom / path are already in the map's frame, which — with the adapter — is a fixed local-ENU datum (Putnam origin from the `race_metadata` TTL). GICP itself is frame-agnostic and simply reports the pose in the map's frame.
- **UTM-frame output** (legacy layer, manifest-less world-frame maps ONLY): when `localization/utm_transform_path` is set, publish pose / odom / path in `utm` frame alongside `map`. With an ENU-manifest map (the exporter's default output) the node **fatally refuses to start** if this path is set — leave it empty.
- **RViz visualization** of map, pose, and debug pose markers.

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

`map_path` is **required** — the yaml `localization/map_path` default points at an
external, untracked map, so pass the ENU PCD you exported from the GLIM dump:

```bash
ros2 launch gicp_plusplus localization_with_tf.launch.py \
    rviz:=true \
    map_path:=/path/to/track_map.pcd \
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
| `odom_topic` | `/odom` | Declared and remapped by the launch but currently **unused** — the node creates no `odom` subscription; `use_odom_init` seeds from the first `gt_odom` message instead. |
| `gt_odom_topic` | `/gps_p1/filtered_odom` | Atlas FusionEngine INS odometry, at `gps_antenna_top`. Used when `localization/gt_odom/enable=true` and/or `gt_recovery/enable=true`. Same frame as `base_frame`, so no TF correction is needed. |
| `imu_only` | `false` | Disable GICP and propagate pose from IMU only (debug/sanity check). |
| `lidar_concat_enabled` | `false` | Opt in to front+left+right online GICP for synchronization/diagnostic A/B tests. Production uses a three-LiDAR offline map with front-only online GICP to meet 10 Hz. |
| `primary_queue_size` | `8` | Bounded front compute queue. Keep 8 for live operation. A lossless offline replay may use a larger bounded queue to absorb rosbag delivery bursts, but must separately prove sub-100 ms scan compute and zero overload drops. |
| `config_path` | empty | Optional run-local YAML loaded after the package default. Parameter files are logged in precedence order. Use `cfg/front_quality_replay.yaml` for the GNSS-aided Laguna profile or `cfg/front_no_atlas_translation_replay.yaml` for the per-scan zero-Atlas-translation A/B. |
| `urdf_path` | (auto-found) | Path to the URDF (`av24.urdf`) used for offline extrinsic resolution. The launch resolves it by walking up from the launch dir; `av24.urdf` is also installed into `share/gicp_plusplus`. |
| `parent_frame` / `child_frame` | `base_link` / `luminar_front` | `child_frame` overrides `localization/lidar_frame` (the LiDAR link the node resolves extrinsics for); `parent_frame` is declared but currently unused (no static-TF helper is launched — `robot_state_publisher` provides the URDF tree). |
| `map_path` | (yaml) | Override the yaml `localization/map_path` from the command line. |

### Frame conventions (all-P1 single-source design)

Every comparison the node performs lives at the Atlas antenna phase centre (URDF link `gps_antenna_top`):

- `localization/base_frame: gps_antenna_top` — GICP's state is reported at this frame.
- `localization/imu_frame: gps_antenna_top` — the `/gps_p1/imu` stream is tagged with this frame. Physically the IMU (`IMUOutput`, FusionEngine Spec §3.4.1) is body-axis-rotated but **device-located** (`pointonenav`), not antenna-projected; setting `imu_frame = base_frame` treats it as co-located with the antenna — a deliberate approximation that drops the small device→antenna accel lever arm (see [`localization.yaml`](cfg/localization.yaml)).
- `gt_odom_topic` → `/gps_p1/filtered_odom` — Atlas INS pose, `child_frame_id="gps_antenna_top"`.

Because base_frame, imu_frame, and the gt_odom source all align, the in-code TF lookups in `callbackImu` (`baselink2imu_T`) and `callbackGtOdom` (`T_base_gtbody_`) degenerate to identity. No lever-arm work happens anywhere; the cross-check `gt_pos_err_m` is exact (no constant baseline bias); `applyInitialPose` correctly seeds the state; the snap helper composes a no-op identity TF.

This design deliberately bypasses race_common's downstream `cg`-frame intermediate (VKS / robot_localization). The trade-off is that the localized pose lives at the antenna point rather than the controller-expected `cg`; downstream consumers must apply the `gps_antenna_top → cg` URDF transform that `robot_state_publisher` provides.

### RTK quality gate — consumer-side, with snap-recovery exemption

GICP uses Atlas's per-sample pose covariance as a quality signal, but the **gate is consumer-specific**. Every `/gps_p1/filtered_odom` message is pushed into the GT buffer unfiltered; the FIXED-quality check is applied where each consumer reads.

A sample qualifies only when every position variance is **finite, nonnegative,
and within its threshold** (`rtk_gate.hpp`, unit-tested). A plain `<=` check
formerly accepted the finite `-1` "covariance not populated" sentinel as
RTK-quality; NaN and ±inf also fail closed now — parity with the adapter's
`/gps_p1/filtered_odom_rtk_fixed` gate. For **interpolated** GT poses, the
position variance combine is conservative in both directions: if *either*
bracketing endpoint is non-finite or negative the component becomes +inf
(fails the gate); otherwise the max. (Yaw variance keeps plain max by design —
the yaw gate treats negative as "unpopulated, passes".)

```yaml
localization/rtk_gate/enable:           true   # inspect msg->pose.covariance per consumer
localization/rtk_gate/max_pose_var_xy:  0.25   # m^2 (~0.5 m horizontal std)
localization/rtk_gate/max_pose_var_z:   1.0    # m^2 (~1.0 m vertical std)
```

| Consumer | Requires FIXED? | Why |
|---|---|---|
| **`tryRtkCalibrationStep`** — RTK-driven IMU bias calibration at startup | ✓ Yes | Needs cm-level truth to estimate gyro/accel bias residuals. If only degraded samples are available the init machine times out and falls back to stationary calibration. |
| **INS heading prior** (`applyInsHeadingPriorToBasePose`, when `ins_prior/require_rtk_fixed` is set) | ✓ Yes | The prior rotates the GICP seed toward the INS heading; a degraded-heading sample would inject the very yaw error the prior exists to remove. |
| **Scan cross-check / candidate sanity envelope** — `gt_pos_err_m` plus optional `max_candidate_position_error_m` reject | ✓ Yes | A cm-level diagnostic and a hard wrong-basin decision both require a trusted reference. Inside the configured radius Atlas position is not fused into GICP. |
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

- `GT recovery: delta-form snap — correction |t|=… m |rot|=… deg applied to the LIVE observer state…` followed by `Localization: ⟳ snapped pose to GT (… after N consecutive non-accepts) — pose=… v=… ω=… | gt_body=…` — appears every time the snap fires (the snap logs the applied pose/twist; it does not report GT covariance — snap intentionally accepts any-quality Atlas samples).
- `GT recovery: deferring snap — no GT sample within max_dt=…s of scan stamp … (streak=N)` — only when even Atlas isn't publishing (true GNSS-denied + INS publication gap); each non-accepted scan also logs `Localization: ⚠ GICP … — holding IMU dead-reckoning pose …`.

In normal operation on a well-mapped track you should rarely see either: GICP scan-match converges on every scan and `consecutive_failures_` resets to 0. The snap path exists for the edge case where LiDAR briefly cannot disambiguate the local map.

### Setting an initial pose

Three options, in priority order:

1. **`localization/gt_odom/enable: true` + `localization/use_odom_init: true`** (default in the shipped yaml): the first GT odom message bootstraps the pose. Works for any bag start-offset without hand-tuning numbers.
2. **`localization/initial_pose/use: true`**: use the numeric `x/y/z/roll/pitch/yaw` from the yaml. The `frame: "lidar"` mode is convenient for pasting from GLIM's `traj_lidar.txt` — the node post-multiplies `inv(T_base_lidar)` automatically.
3. **RViz "2D Pose Estimate"**: publish to `/initialpose`. Always available as a manual override.

## Sensor division of labor — IMU vs INS (2026-07-06)

The two Atlas streams have distinct, deliberate roles:

| Stream | Role |
|---|---|
| `/gps_p1/imu` (imu_calibrated) | **IMU-rate propagation + per-point deskew.** High rate, low latency, body-frame — everything short-horizon. |
| `/gps_p1/filtered_odom` (INS solution) | **Stable heading (+ optional position) prior.** Dual-antenna-aided heading is drift-free — the reference the gyro-integrated chain lacks. |

Before each scan's IMU integration, the integration seed's yaw is blended
toward the time-matched, quality-gated INS attitude
(`localization/ins_prior/*`: blend 0.25, per-scan cap 2°, sanity guard 30°,
position-RTK gate + heading-quality gate `max_yaw_sigma_deg: 3.0` on
`pose.covariance[35]`, mirroring GLIM's `orientation_prior_max_yaw_sigma_deg` —
Atlas can be position-FIXED while dual-antenna heading is degraded).
"Time-matched" means matched to the stamp `basePose` is actually valid at —
the previous scan's **median point time**, not its header stamp; querying at
the header stamp injected a yaw-rate-proportional bias (~half-sweep × yaw
rate, e.g. 50 ms × 30°/s = 1.5°) into every turn (review fix 2026-07-08).
Because the correction lands **before** deskew/placement, the deskewed cloud,
`T_prior`, the initial guess, the 4-DoF fixed axes, the soft rotation-prior
target, the yaw veto/innovation gates, and the delta-form observer all
inherit the stable heading consistently — and the yaw-safety layer is now
anchored to a drift-free reference instead of its own integration history.

The same bounded step is applied to the geometric-observer state
(`state.q`, world-frame velocities, `geo.prev_q/prev_vel`) under the observer
lock (review fix 2026-07-08): the observer runs in delta form, so when GICP
merely *confirms* the corrected prior the delta is identity and the IMU-rate
odometry would otherwise never inherit the heading fix — only the scan-time
chain would.

Recovery property: if a bad yaw accept ever slips every gate, subsequent
priors are pulled back toward INS truth at up to 20°/s (2°/scan @ 10 Hz) — a
5° heading error decays below 1° within ~6 scans, and (since the observer
inherits each step) the IMU-rate output decays with it.

Diagnostics: `debug/ins_yaw_diff_deg` topic and `ins_dyaw=` in SCAN DEBUG; the
scorecard reports its distribution. **A persistent nonzero value measures
map-vs-ENU yaw misalignment (or an INS heading fault) — fix that, don't raise
the blend.** `pos_blend` defaults OFF; enabling it makes `gt_pos_err` against
the same INS non-independent (score with held-out segments).

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

### GICP gating (Laguna/perception-ws deployment default)

```yaml
gicp/maxCorrespondenceDistance: 1.0
gicp/minCorrespondences: 0
gicp/minCorrespondenceRatio: 0.3
gicp/fitnessRejectThreshold: 1000000000.0 # finite ceiling; NaN/Inf still fail closed

# Optional rolling fitness normalization is retained for A/B experiments, but
# disabled in the deployed Laguna contract:
gicp/fitnessBaseline/enable: false
gicp/fitnessBaseline/window: 201          # rolling-median window (~20 s @ 10 Hz)
gicp/fitnessBaseline/minSamples: 50       # rolling median takes over after this
gicp/fitnessBaseline/seedBaseline: 0.28   # warm-up baseline so gates are live from frame 1 (re-measure per map!)
gicp/fitnessRatioRejectThreshold: 0.0     # disabled for perception-ws parity

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
default support and physical gates catch loss of overlap or impossible motion
without assuming a particular map's fitness scale. Score a replay with
`scripts/analyze_scan_debug_log.py`; it reports accepted fitness and support so
optional ratio thresholds can still be evaluated in an explicit A/B.

### Compressed-map quality profile

`cfg/front_quality_replay.yaml` is the checked-in Laguna compressed-map
profile used through the launch file's `config_path` argument. It leaves the
production motion chain enabled, uses 0.25 m target and 0.30 m source voxels
with a 100 m sensor-frame crop, 32 iterations, and an 80 ms cooperative
scan-registration budget. The budget includes source KD-tree/covariance
preparation and passes only its remaining time to the iterative optimizer.
Atlas translation seeds only the GICP optimizer; it never modifies
`basePose`, observer state, or published output. Every candidate must still
pass correspondence, physical-jump, and the unchanged 5 m Atlas wrong-basin
gate.

`cfg/front_no_atlas_translation_replay.yaml` is the registration-side A/B: it
sets both the per-scan Atlas translation seed blend and Atlas
candidate-position gate to zero. Package defaults may still use Atlas for
initialization, heading, and recovery, so this profile is not independent
truth. The audit runner requires an explicit `gnss_aided` or `independent`
evidence label; independent evidence must use a reference topic distinct from
the runtime GT topic.

Use the profile with the topic-reduced replay bag and the repository audit
runner documented in the root workflow. A rate pass requires 1.0x playback,
zero front overload drops, and measured scan-compute latency below the 10 Hz
deadline. The live-car queue remains 8; a lossless offline audit may use a
larger bounded queue only to absorb rosbag delivery bursts, and must report
that queue separately.

### Multi-LiDAR concatenation

3x Luminar: `luminar_front` primary + `luminar_right`/`luminar_left` merged.

**Matching is gated by absolute point time, never by header proximity.** Each
Iris cloud carries `UINT8[8]` epoch-nanosecond point times; an aux sweep is
coherent only when its endpoint-range error vs the primary
(`max(|min−min|,|max−max|)`) is ≤ `luminar_point_time_threshold_s`. Header
distance is only a tie-break. Header-nearest selection is exactly the
wrong-sweep failure mode (a one-period-early sweep produced ~149 ms merged
spans and corrupted deskew); it is retained solely for non-Luminar sensors.
In Luminar mode an unsupported time layout merges **front-only** (all aux
omitted, `unsupported_point_time`). The supported Laguna FLOAT64
seconds-since-sweep-start layout uses a future-header watermark and nearest
header selection; it is not treated as an unsupported absolute-time stream.
Big-endian clouds are rejected before matching.

```yaml
localization/lidar_concat/enabled:        true
localization/lidar_concat/reliable_qos:   false   # live BEST_EFFORT default; opt into RELIABLE for lossless bag audits
localization/lidar_concat/aux_topics:     ["/luminar_right/points", "/luminar_left/points"]
localization/lidar_concat/aux_frames:     ["luminar_right", "luminar_left"]
localization/lidar_concat/luminar_point_time_threshold_s: 0.010  # ABSOLUTE point-time acceptance gate (Luminar)
localization/lidar_concat/time_threshold: 0.05    # non-Luminar/relative-time header matching gate
localization/lidar_concat/buffer_size:    200     # per-aux ring depth (P4: raised from 20 — 2 s of history silently degraded frames)
localization/lidar_concat/aux_time_offsets: [0.0, 0.0]  # measured residual point-clock corrections; keep zero —
                                                  # header phase is NOT clock evidence. Validated at startup
                                                  # (finite, |v| <= 0.5 s; refuses to start otherwise).
localization/lidar_concat/float64_time_is_epoch_ns: false # false = FLOAT64 relative seconds (Laguna);
                                                          # true only for verified raw uint64 epoch-ns bytes

# Async Luminar front worker (front-only production and concat diagnostic paths):
localization/lidar_concat/future_aux_wait_timeout_s: 0.150   # arrival-time release deadline for a pending front
localization/lidar_concat/primary_queue_size:        8      # HARD bound; overflow = counted overload drop of the OLDEST front

# Strict merge guard — IDENTICAL semantics + defaults to GLIM:
localization/lidar_concat/require_all_aux:                    false  # false = localize on whatever LiDARs merged; true = incomplete merge SKIPS the scan (degraded cloud never registered; IMU propagation continues)
localization/lidar_concat/abort_on_merge_failure:            true   # only relevant when require_all_aux=true: abort node past budget vs keep skipping non-fatally
localization/lidar_concat/max_consecutive_aux_merge_failures: 10
```

### Async Luminar front worker and aux synchronizer

Every Luminar front scan, including the production front-only path, enters a
bounded worker queue. A long GICP iteration therefore cannot block its DDS
subscription callback and silently exhaust the RELIABLE keep-last history.
When concat is enabled, the point-coherent right sweep arrives ~92 ms **after**
the front cloud (acquisition phase), so aux waiting also stays off the
subscription callback. Instead:

- The front callback only **validates and enqueues** (microseconds, never
  blocks). Aux callbacks decode the point-time range once, buffer, and wake
  the worker.
- A dedicated **worker thread owns release order** and runs the unchanged
  merge→deskew→GICP pipeline. Front-only scans release immediately in FIFO
  order. With concat enabled, an absolute-time front releases when every aux
  is *matched* (in-gate) or *final* (point-time watermark); relative FLOAT64
  waits until every aux stream reaches the front header, then selects the
  nearest header. The timeout remains the live fail-safe.
- **Aux state can never drop a front.** The only front drops are: invalid
  primary data (`front_invalid`), explicit shutdown accounting, the
  coordinated epoch-reset queue purge (`front_epoch_dropped` — queued fronts
  from before a detected timestamp/session reset are discarded with
  accounting), and the
  **compute-overload policy** — `primary_queue_size` is a hard bound and
  overflow drops the OLDEST queued front with an ERROR log and the
  `front_overload_dropped` counter (bounded latency/memory instead of a
  backlog outliving the 2000-sample IMU history). Sustained overload means
  the solver, not the queue, needs fixing (VGICP/decimation).
- **Conservation invariant**, checked in the end-of-run summary:
  `front_received == front_released + front_invalid + front_shutdown_unprocessed
  + front_overload_dropped + front_epoch_dropped`. (`front_epoch_dropped` counts
  fronts discarded when a coordinated epoch reset clears the queue on a
  timestamp/session reset.) Violation logs an ERROR.
- **Teardown drains, not abandons**: a pre-shutdown callback runs the drain
  while the ROS context is still valid (Ctrl-C and the bag-EOS SIGTERM path),
  so the run tail is processed in order. A pipeline exception on the worker
  (e.g. strict-merge abort) becomes a controlled shutdown and a **nonzero
  exit code** — never `std::terminate`.
- Per-frame telemetry: `debug/front_release_reason`
  (0=all_matched 1=watermark 2=timeout 4=shutdown_drain 5=primary_no_abstime, −1=legacy path),
  `debug/front_wait_ms`, `debug/primary_queue_depth`, alongside the existing
  `merged_aux_count` / `aux<i>_merge_dt_s` / `scan_time_span_s` records.
  Healthy front-only replay: ~all `all_matched`, near-zero `front_wait_ms`,
  and `front_overload_dropped=0`. Healthy concat replay waits about 92 ms and
  reports a merged span around 49 ms (never ≥ 100 ms).

The operational acceptance checks are the conservation invariant above,
`front_overload_dropped=0`, mostly `all_matched` releases, merged span below
100 ms, and no systematic per-aux point-time mismatch. These checks are
available directly from the end-of-run summary and `SCAN DEBUG` telemetry.

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
header-offset stats are summarized every 512 merges as **acquisition-phase
observability only** — a stable nonzero mean (66–92 ms right on AV-24) is
expected on PTP-synchronized Iris units whose absolute point clocks agree to
<1 ms. It is NOT point-clock evidence; never copy it into
`aux_time_offsets` (doing so shifts an aligned range out of the 10 ms gate).

### Ground-truth diagnostics + recovery

```yaml
localization/gt_odom/enable:        true
localization/gt_odom/buffer_size:   200      # ~2 s of history at 100 Hz
localization/gt_odom/max_dt:        0.1      # max scan-to-GT lookup gap
localization/gt_odom/max_candidate_position_error_m: 5.0 # RTK-quality wrong-basin envelope; 0 = diagnostic only

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
odom/geo/max_pos_correction: 2.0   # Bound one accepted scan's observer position injection
odom/geo/max_vel_correction: 5.0   # Bound one accepted scan's observer velocity injection
odom/geo/max_state_speed: 100.0    # Physical fail-safe above Laguna race speed
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
The three observer bounds prevent a wrong-basin residual from turning directly
into an unphysical prediction and an expensive full-map miss; they are
fail-safes, not normal-operation tuning targets.
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
| `localized_path` (`gicp/localization/path`) | `nav_msgs/Path` | Trajectory history (requires `localization/debug/enable_pub: true`, the default). |
| `gicp/localization/pose_utm` / `odom_utm` / `path_utm` | (same types) | Legacy UTM-frame mirrors, only when `utm_transform_path` is set — manifest-less world-frame maps only (fatal with an ENU-manifest map). |
| `gicp/localization/gt_snap` | `geometry_msgs/PoseStamped` | Pose applied by each GT recovery snap (requires `debug/enable_pub`). |
| `map` (`gicp/localization/map`) | `sensor_msgs/PointCloud2` | Downsampled visualization map. |
| TF: `map → base_frame` | | Published when `publish_tf=true`. |

### Debug topics (require `localization/debug/enable_pub: true`)

Per-scan scalar metrics on `gicp/localization/debug/*`:

- `fitness`, `gicp_elapsed_ms`, `final_error`, `corr_norm`, `scan_dt`
- `imu_age`, `imu_buffer_span_s`, `scan_to_latest_imu_lag_s`
- `num_correspondences`, `correspondence_ratio`
- `guess_to_solution_trans_m`, `guess_to_solution_rot_deg`
- `guess_from_last_m`, `guess_from_last_deg`
- `jump_trans`, `jump_rot_deg` (raw GICP-vs-prior disagreement, pre-projection)
- `hessian_condition_proxy`
- **P1 gating**: `fitness_ratio` (−1 during warm-up without seed), `degen_rot_axes`, `degen_trans_axes`, `yaw_veto`
- **P4 concat**: `merged_aux_count` (−1 = concat disabled), `aux<i>_merge_dt_s` (signed; NaN = not merged), `aux<i>_points`, `scan_time_span_s`
- `gt_pos_err_m`, `gt_rot_err_deg` (when GT is enabled; measured against the pose actually applied)
- `yaw_innovation_deg`, `yaw_marginal_stiffness`, `ins_yaw_diff_deg` (yaw-gate / INS-prior calibration inputs)
- `raw_points`, `preprocessed_points` (per-frame point counts)
- `converged` (Bool)

Plus pose topics: `initial_guess_pose`, `final_pose` (post-projection), `pose_markers`.

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
                            gate: support / fitness / fitness-ratio / degeneracy-projection / yaw-veto / jump
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
- **Rejected**: `failed_to_converge`, `rejected_support` (min-correspondence
  gate: `gicp/minCorrespondences: 500` / `minCorrespondenceRatio: 0.2`),
  `rejected_fitness` (absolute),
  `rejected_fitness_ratio` (P1 wrong-basin gate), `rejected_hessian` (now only
  the all-axes-degenerate case), `rejected_jump`, `invalid_solution` — all fall
  through to the dead-reckoning branch (set `current_pose ← T_prior`, seed
  `prev_vel` from the current IMU-propagated velocity, increment streak
  counter, optionally trigger snap).
- `last_gicp_pose_` is **not** updated on rejection, so the IMU prior on the next scan is still anchored to the last successfully-matched GICP pose.

## Map preparation

A current GLIM dump is a directory of **compact submap folders**, not a single
cloud. From the repository root, the supported handoff is
`scripts/export_glim_dump_to_pcd.py`, which
composes the submaps via their `T_world_origin`, converts WORLD→ENU with
`inverse(T_world_utm)`, and writes the `*.manifest.yaml` provenance record that
this node checks at load:

```bash
python3 scripts/export_glim_dump_to_pcd.py /path/to/glim_dump /path/to/track_map.pcd \
    --voxel-size 0.1
```

Do **not** hand-convert individual submap clouds (e.g. a raw PLY→PCD copy): that
yields a WORLD-frame map with no manifest, which is frame-mismatched against the
Atlas ENU seeds/GT and which `loadMap` warns about or rejects. The exporter reads
the datum from `<dump>/enu_origin.txt` when present (the all-in-one `prep_bag.py`
route writes it); for a hand-run dump pass `--enu-origin "<lat,lon,alt>"`.

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

If you see SCAN DEBUG gaps > 200 ms during turns, first inspect the async front
worker counters (`front_overload_dropped`, queue depth, and the conservation
summary). In concat mode, diagnose the aux synchronizer rather than reaching
for `time_threshold` — in Luminar mode that
header window is only a fallback/tie-break, and the authoritative match is the
decoded per-point endpoint error (`luminar_point_time_threshold_s`), so raising
`time_threshold` will not close a real point-time gap. Inspect the release
telemetry: `front_release_reason` (was the front released by watermark or by
timeout?), `front_wait_ms` (how long it waited for aux), `merged_aux_count` (did
front+left+right actually merge?), and the per-aux point-time range error. A
persistent timeout-release with low `merged_aux_count` means an aux stream is
genuinely late or mis-clocked (check the per-aux mean header-offset warning), not
that the gate is too tight. (If `require_all_aux: true`, an out-of-sync aux SKIPS
the whole scan rather than degrading it.)

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

`scripts/visualize_lidar_topic.py` was written against a `gicp/localization/debug/input_cloud_base` topic that this node **no longer publishes**, so its comparison mode cannot produce output until the script is ported. To verify the node is consuming the LiDAR topic you expect, use the per-frame debug counters instead: `raw_points` / `preprocessed_points` (nonzero at scan rate) and `merged_aux_count` (source set), plus `ros2 topic hz` on the input topic.

### Profiling

`scripts/profile_localization_resources.py` and `scripts/plot_localization_profile.py` capture and chart per-scan resource usage and the full debug-topic time series. Useful for tuning real-time performance.

`scripts/plot_source_switches.py` plots when the node switches between GICP, dead-reckoning, and GT snap — handy when investigating snap behavior.
