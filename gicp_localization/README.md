# GICP Localization

GICP scan-to-map localization with IMU dead-reckoning and optional ground-truth-driven recovery. Locks onto a pre-built PCD map produced by GLIM (or any compatible source) and publishes pose at IMU rate. Designed for AV-24 Cybertruck-class platforms with multi-LiDAR setups.

## Pipeline context

GICP is the online-localization stage of a four-stage pipeline:

```
adapter (Atlas WGS84 → local-ENU, publishes /gps_p1/*)
   → scripts/prep_bag.py (normalized bag)
      → GLIM (offline map, PCD)
         → gicp_localization (online localize)
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

- **GICP scan-to-map matching** against a single pre-built PCD map (no submap stitching at runtime).
- **IMU + LiDAR pipeline**: IMU integrates a motion prior between scans; GICP refines; a geometric observer fuses the two and propagates pose at IMU rate (~100 Hz).
- **Multi-LiDAR concatenation** (`lidar_concat`): 3x Luminar (`luminar_front` primary + `luminar_right`/`luminar_left` merged); time-aligns aux LiDARs to the primary, transforms them via offline-resolved extrinsics, and concatenates per-point timestamps onto the primary clock. A strict merge guard (`require_all_aux` / `abort_on_merge_failure`, identical semantics + defaults to GLIM) controls whether an incomplete merge degrades or skips the scan.
- **Layered rejection gates**:
  - Hard fitness reject (`gicp/fitnessRejectThreshold`)
  - Combined geometric-degeneracy gate (`hessianCondMax` AND any of `fitness`/`trans`/`rot` warn floors) — catches optimizer slides on feature-poor corners
  - Large-jump reject (compares GICP candidate to IMU-predicted prior)
- **IMU dead-reckoning fallback**: any non-accepted scan falls back to the IMU-integrated prior instead of freezing at the last accepted pose, so transient corner failures don't cascade.
- **Ground-truth divergence cross-check** (optional): subscribes to a `gt_odom` topic, computes per-scan `gt_err=[trans,rot,dt]`, publishes deltas. Diagnostic only — never feeds back into accept/reject.
- **GT-driven pose recovery** (optional): when GICP fails for N consecutive scans, snap pose+velocity to a time-matched GT sample (composed through TF into `base_frame`) so GICP can re-acquire from a known-good state. Disabled by default; falls back to dead-reckoning when GT is unavailable.
- **GT-bootstrapped initial pose** (optional): take the first GT message as the initial pose so the node starts at the right location regardless of bag offset.
- **Local-ENU output** (operational contract): the primary `map_frame` pose / odom / path are already in the map's frame, which — with the adapter — is a fixed local-ENU datum (Putnam origin from the `race_metadata` TTL). GICP itself is frame-agnostic and simply reports the pose in the map's frame.
- **UTM-frame output** (optional legacy layer): only active if `localization/utm_transform_path` is set; when provided, publish pose / odom / path in `utm` frame alongside `map`. Not the default.
- **RViz visualization** of map, aligned scan, pose, debug clouds and markers.

## Dependencies

- ROS 2 Jazzy
- PCL, Eigen3, OpenMP, nlohmann::json
- `PointType` and a vendored copy of `nano_gicp` ship inside this package; no
  separate `direct_lidar_inertial_odometry` dependency is required.
- For development: matplotlib (debug-script plots)

## Building

This package lives in the `DLIO_plusplus` workspace. From the repo root:

```bash
colcon build --packages-select gicp_localization --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage

### Launch

The single supported launch file is `localization_with_tf.launch.py`. It starts the localization node alongside `robot_state_publisher` (which publishes the URDF transforms the node depends on for sensor extrinsics).

```bash
ros2 launch gicp_localization localization_with_tf.launch.py \
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
| `urdf_path` | (auto-found) | Path to the URDF (`av24.urdf`) used for offline extrinsic resolution. The launch resolves it by walking up from the launch dir; `av24.urdf` is also installed into `share/gicp_localization`. |
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

### GICP rejection gates

```yaml
gicp/fitnessRejectThreshold: 1.0          # hard reject: fitness > threshold
gicp/rejectLargeJumps: true               # reject if pose jumps > debug_jump_*
gicp/hessianCondMax: 5.0e9                # condition-number floor for combined gate
gicp/hessianFitnessWarnThreshold: 0.15    # OR trigger: fitness elevated
gicp/hessianTransWarnM: 1.0               # OR trigger: GICP correction > X m
gicp/hessianRotWarnDeg: 1.5               # OR trigger: GICP correction > X deg
```

The combined hessian gate fires when condition number is high AND any of the three warn floors is crossed. High hessian alone is harmless when GICP barely moved (good IMU prior, degenerate but well-anchored geometry); large corrections in degenerate geometry are the slide signature. Set any threshold ≤ 0 to disable that OR branch.

### Multi-LiDAR concatenation

3x Luminar: `luminar_front` primary + `luminar_right`/`luminar_left` merged.

```yaml
localization/lidar_concat/enabled:        true
localization/lidar_concat/aux_topics:     ["/luminar_right/points", "/luminar_left/points"]
localization/lidar_concat/aux_frames:     ["luminar_right", "luminar_left"]
localization/lidar_concat/time_threshold: 0.1     # drop aux scans further than this from primary

# Strict merge guard — IDENTICAL semantics + defaults to GLIM:
localization/lidar_concat/require_all_aux:                    false  # false = localize on whatever LiDARs merged; true = incomplete merge SKIPS the scan (degraded cloud never registered; IMU propagation continues)
localization/lidar_concat/abort_on_merge_failure:            true   # only relevant when require_all_aux=true: abort node past budget vs keep skipping non-fatally
localization/lidar_concat/max_consecutive_aux_merge_failures: 10
```

**Offline extrinsic resolution (no live `/tf_static` needed).** Aux extrinsics
are resolved offline, in priority order: URDF (`av24.urdf` via
`lidar_concat/urdf_path`, passed by the launch which resolves it by walking up
from the launch dir; also installed into `share/gicp_localization`) > a static
per-aux 4x4 (`aux_static_transforms`, baked from `av24.urdf`) > live TF as a last
resort. The `base_frame ← lidar_frame` lever arm is resolved the same way
(URDF > `localization/base_lidar_transform` static matrix > live TF), so full
localization needs no `/tf_static`.

**Map voxel downsample + crop box.** The GICP target map is voxel-downsampled at
load (`localization/map_voxel_size: 0.3`) before building the kd-tree, to bound
memory. The crop box runs in the **sensor frame** before deskew.

### Ground-truth diagnostics + recovery

```yaml
localization/gt_odom/enable:        true
localization/gt_odom/buffer_size:   200      # ~2 s of history at 100 Hz
localization/gt_odom/max_dt:        0.1      # max scan-to-GT lookup gap

localization/gt_recovery/enable:                   true    # snap to GT after sustained GICP failure
localization/gt_recovery/min_consecutive_failures: 1       # snap after N consecutive non-accepts
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
```

`Kab`/`Kgb` are intentionally zero for the fused Point One (Atlas) INS path. Initial
RTK/stationary calibration may still seed `state.b`, but GICP residuals do not
continue rewriting IMU bias online unless these gains are explicitly raised.

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
| `localized_path` (`gicp/localization/path`) | `nav_msgs/Path` | Final adopted localization trajectory history. |
| `gt_ins` (`/gt_ins`) | `nav_msgs/Path` | RViz/debug GT/INS reference path, sampled at the same LiDAR scan stamps as `localized_path`. |
| `gicp/localization/gicp_only_path` | `nav_msgs/Path` | RViz/debug current accepted-GICP-only segment; clears when the node falls back to IMU/GT recovery. |
| `gicp/localization/gicp_only_segments` | `visualization_msgs/MarkerArray` | RViz/debug accepted-GICP-only history as broken orange line segments, preserving gaps across fallback/recovery. |
| `gicp/localization/pose_utm` / `odom_utm` / `path_utm` | (same types) | Optional legacy UTM-frame mirrors, only when `utm_transform_path` is set (the primary `map`-frame outputs above are already local ENU with the adapter). |
| `aligned_cloud` (`gicp/localization/aligned_cloud`) | `sensor_msgs/PointCloud2` | Aligned scan in `map`. |
| `map` (`gicp/localization/map`) | `sensor_msgs/PointCloud2` | Downsampled visualization map. |
| TF: `map → base_frame` | | Published when `publish_tf=true`. |

### RViz path colors

The default `launch/localization.rviz` config uses fixed colors for the path
overlays:

| Color | Display | Topic | Meaning |
|---|---|---|---|
| Green | Trajectory Path | `/gicp/localization/path` | Final adopted localization trajectory, including accepted GICP, IMU dead-reckoning fallback, and GT recovery snaps. |
| Orange | GICP-only Segments | `/gicp/localization/gicp_only_segments` | Accepted-GICP-only trajectory segments. The line breaks while the node is on IMU fallback or GT recovery. |
| Blue | GT INS Reference | `/gt_ins` | GT/INS reference path sampled at the same LiDAR scan timestamps as the final path. |

### Debug topics (require `localization/debug/enable_pub: true`)

Per-scan scalar metrics on `gicp/localization/debug/*`:

- `fitness`, `gicp_elapsed_ms`, `final_error`, `corr_norm`, `scan_dt`
- `imu_age`, `imu_buffer_span_s`, `scan_to_latest_imu_lag_s`
- `num_correspondences`, `correspondence_ratio`
- `guess_to_solution_m`, `guess_to_solution_deg`
- `guess_from_last_m`, `guess_from_last_deg`
- `jump_trans`, `jump_rot_deg`
- `hessian_condition_proxy`
- `gt_pos_err_m`, `gt_rot_err_deg` (when GT is enabled)
- `converged` (Bool)

Plus pose / cloud topics: `initial_guess_pose`, `final_pose`, `input_cloud_base`, `initial_guess_cloud`, `pose_markers`.

## Algorithm

```
   IMU ──→ buffer ─────────────────────────────┐
                                                ↓
   LiDAR ──→ lidar_concat ──→ preprocess ──→ T_prior = integrate(IMU, last lidarPose)
                                                ↓
                                              GICP align (initial guess = T_prior)
                                                ↓
                                              gate: fitness / hessian-combined / jump
                                ┌─── accepted ─┴── rejected ──┐
                                ↓                              ↓
                          updateState (geo observer)     dead-reckon: lidarPose ← T_prior
                                ↓                              ↓
                          state ← merge(GICP, IMU)        consecutive_failures++
                                ↓                              ↓
                                                       ≥ N consecutive AND gt_recovery on?
                                                              ↓
                                                       maybeSnapPoseToGT(reason)

                          propagateState (every IMU sample) → publish odom/TF at ~100 Hz
```

### Rejection branches

- `failed_to_converge`, `rejected_fitness`, `rejected_hessian`, `rejected_jump`, `invalid_solution` — all fall through to the dead-reckoning branch (set `current_pose ← T_prior`, increment streak counter, optionally trigger snap).
- `last_gicp_pose_` is **not** updated on rejection, so the IMU prior on the next scan is still anchored to the last successfully-matched GICP pose.

## Map preparation

GLIM dumps submaps as PLY-format files with a `.pcd` extension. Convert them with:

```bash
python3 gicp_localization/scripts/convert_ply_to_pcd.py \
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

Watch for `GICP REJECTED (... — degenerate slide)` warns. Tunable knobs (in order of impact):

1. Lower `gicp/hessianTransWarnM` / `hessianRotWarnDeg` to catch slides earlier (default 1.0 m / 1.5°).
2. Lower `gicp/hessianCondMax` to be stricter about what counts as "degenerate" (default 5e9).
3. If the rejection cascades, enable `gt_recovery` to recover at corners.

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
