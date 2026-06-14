# GICP Localization

GICP scan-to-map localization with IMU dead-reckoning and optional ground-truth-driven recovery. Locks onto a pre-built PCD map produced by GLIM (or any compatible source) and publishes pose at IMU rate. Designed for AV-24 Cybertruck-class platforms with multi-LiDAR setups.

## Features

- **GICP scan-to-map matching** against a single pre-built PCD map (no submap stitching at runtime).
- **IMU + LiDAR pipeline**: IMU integrates a motion prior between scans; GICP refines; a geometric observer fuses the two and propagates pose at IMU rate (~100 Hz).
- **Multi-LiDAR concatenation** (`lidar_concat`): subscribes to N aux LiDARs, time-aligns to the primary, transforms via URDF, and concatenates per-point timestamps onto the primary clock.
- **Layered rejection gates**:
  - Hard fitness reject (`gicp/fitnessRejectThreshold`)
  - Combined geometric-degeneracy gate (`hessianCondMax` AND any of `fitness`/`trans`/`rot` warn floors) — catches optimizer slides on feature-poor corners
  - Large-jump reject (compares GICP candidate to IMU-predicted prior)
- **IMU dead-reckoning fallback**: any non-accepted scan falls back to the IMU-integrated prior instead of freezing at the last accepted pose, so transient corner failures don't cascade.
- **Ground-truth divergence cross-check** (optional): subscribes to a `gt_odom` topic, computes per-scan `gt_err=[trans,rot,dt]`, publishes deltas. Diagnostic only — never feeds back into accept/reject.
- **GT-driven pose recovery** (optional): when GICP fails for N consecutive scans, snap pose+velocity to a time-matched GT sample (composed through TF into `base_frame`) so GICP can re-acquire from a known-good state. Disabled by default; falls back to dead-reckoning when GT is unavailable.
- **GT-bootstrapped initial pose** (optional): take the first GT message as the initial pose so the node starts at the right location regardless of bag offset.
- **UTM-frame output** (optional): if `T_world_utm.txt` is provided, publish pose / odom / path in `utm` frame alongside `map`.
- **RViz visualization** of map, aligned scan, pose, debug clouds and markers.

## Dependencies

- ROS 2 Humble
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
| `urdf_path` | (auto-found) | Path to the URDF that publishes sensor TFs. |
| `parent_frame` / `child_frame` | `base_link` / `luminar_front` | Used by the bundled static-TF helper. |
| `map_path` | (yaml) | Override the yaml `localization/map_path` from the command line. |

### Frame conventions (all-P1 single-source design)

Every comparison the node performs lives at the Atlas antenna phase centre (URDF link `gps_antenna_top`):

- `localization/base_frame: gps_antenna_top` — GICP's state is reported at this frame.
- `localization/imu_frame: gps_antenna_top` — IMU subscription from `/gps_p1/imu` is at this frame (Atlas firmware projects the chassis-mounted IMU to the antenna point internally via lever-arm).
- `gt_odom_topic` → `/gps_p1/filtered_odom` — Atlas INS pose, `child_frame_id="gps_antenna_top"`.

Because base_frame, imu_frame, and the gt_odom source all align, the in-code TF lookups in `callbackImu` (`baselink2imu_T`) and `callbackGtOdom` (`T_base_gtbody_`) degenerate to identity. No lever-arm work happens anywhere; the cross-check `gt_pos_err_m` is exact (no constant baseline bias); `applyInitialPose` correctly seeds the state; the snap helper composes a no-op identity TF.

This design deliberately bypasses race_common's downstream `cg`-frame intermediate (VKS / robot_localization). The trade-off is the localized pose lives at the antenna point rather than the controller-expected `cg` (downstream consumers need an extra `gps_antenna_top → cg` TF lookup, which `robot_state_publisher` already provides). See `docs/GICP_GNSS_IMU_bug_report.pdf` for the architectural alternatives and their trade-offs.

### RTK quality gate (P1 covariance-based)

GICP enforces in-code that every gt_odom sample carries Atlas-reported pose covariance below configured thresholds before it enters the buffer. Replaces the legacy NovAtel BESTGNSSPOS enum gate; the gate is now self-contained in `callbackGtOdom` (no separate status topic).

```yaml
localization/rtk_gate/enable:           true   # inspect msg->pose.covariance
localization/rtk_gate/max_pose_var_xy:  0.25   # m^2 (~0.5 m horizontal std)
localization/rtk_gate/max_pose_var_z:   1.0    # m^2 (~1.0 m vertical std)
```

Mechanism: on every `/gps_p1/filtered_odom` message, the node reads `pose.covariance[0]`, `[7]`, `[14]` (xx, yy, zz position variances). The sample is rejected if any horizontal variance exceeds `max_pose_var_xy` OR the vertical variance exceeds `max_pose_var_z`. Reference covariances from a known-RTK-fixed AV-24 bag: median `cov_xx`≈2.8e-5, `cov_yy`≈4.2e-5, `cov_zz`≈1.0e-4 m². RTK-float typically lives in the 1e-2…1e-1 m² band; GPS-only at 1 m²+.

When the gate rejects, GICP runs on IMU dead-reckoning until Atlas's solution recovers. Operator-facing log line (throttled to 5 s):

- `RTK gate: dropping gt_odom -- pose covariance exceeds threshold (cov_xx=… cov_yy=… cov_zz=… ; max_xy=… max_z=…). Rejected total=N`

Set `rtk_gate/enable: false` only for bag-replay diagnostics where the covariance isn't trustworthy — disabling the gate lets snap and init seed state from degraded GNSS.

### Setting an initial pose

Three options, in priority order:

1. **`localization/gt_odom/enable: true` + `localization/use_odom_init: true`** (default in the shipped yaml): the first GT odom message bootstraps the pose. Works for any bag start-offset without hand-tuning numbers.
2. **`localization/initial_pose/use: true`**: use the numeric `x/y/z/roll/pitch/yaw` from the yaml. The `frame: "lidar"` mode is convenient for pasting from GLIM's `traj_lidar.txt` — the node post-multiplies `inv(T_base_lidar)` automatically.
3. **RViz "2D Pose Estimate"**: publish to `/initialpose`. Always available as a manual override.

## Configuration

All parameters live in `cfg/localization.yaml`. The yaml has inline comments explaining each knob; the cheat sheet below covers the parts most worth tuning.

### Frames

```yaml
localization/map_frame:    "map"
localization/base_frame:   "gps_antenna_top"  # body the node tracks; URDF link
localization/imu_frame:    "gps_antenna_top"  # IMU URDF link (matches /gps_p1/imu header)
localization/lidar_frame:  "luminar_front"    # primary LiDAR URDF link
```

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

```yaml
localization/lidar_concat/enabled:        true
localization/lidar_concat/aux_topics:     ["/luminar_right/points", "/luminar_left/points"]
localization/lidar_concat/aux_frames:     ["luminar_right", "luminar_left"]
localization/lidar_concat/time_threshold: 0.05    # drop aux scans further than this from primary
```

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
dlio/deskew: false                 # Luminar timestamps are collapsed → deskew has no effect
dlio/imu/bufferSize: 2000
dlio/imu/calibTime: 0.5            # initial stationary calibration window

odom/geo/Kp: 4.5                   # Position correction gain
odom/geo/Kv: 11.25                 # Velocity
odom/geo/Kq: 4.0                   # Orientation
odom/geo/Kab: 0.0                  # Online accel-bias adaptation disabled
odom/geo/Kgb: 0.0                  # Online gyro-bias adaptation disabled
```

`Kab`/`Kgb` are intentionally zero for the fused Point One Atlas INS path. Initial
RTK/stationary calibration may still seed `state.b`, but GICP residuals do not
continue rewriting IMU bias online unless these gains are explicitly raised.

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
| `gicp/localization/pose_utm` / `odom_utm` / `path_utm` | (same types) | UTM-frame mirrors when `utm_transform_path` is set. |
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

For UTM output, point `localization/utm_transform_path` at GLIM's `T_world_utm.txt` from the same dump.

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

If you see SCAN DEBUG gaps > 200 ms during turns, `lidar_concat/time_threshold` is dropping aux scans that fell out of sync. Try raising it from `0.05` to `0.1`–`0.15`. The merged cloud will have slightly worse intra-frame alignment but that's almost always cheaper than a 600 ms scan-stream gap during cornering.

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
