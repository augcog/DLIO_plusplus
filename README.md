# DLIO++

https://drive.google.com/file/d/15UakSaWzjoIB35DNQZRRd4rf3OvrjUqs/view?usp=sharing

ROS 2 perception stack for the AV-24 Cybertruck autonomous race car. Pairs a GPU-accelerated LiDAR-inertial SLAM front end with a map-based localizer so the vehicle can build a map offline, then localize against it online at IMU rate.

## Packages

| Package | Upstream | Purpose in this fork |
|---|---|---|
| [`GLIM/`](GLIM/) | [`koide3/GLIM`](https://github.com/koide3/glim) (+ `glim_ext`, `glim_ros2`) | LiDAR-inertial SLAM. Builds a 3D map from IMU + multi-LiDAR + GNSS. |
| [`gicp_localization/`](gicp_localization/) | Vendored from the `vectr-ucla` DLIO line (uses `nano_gicp`) | GICP scan-to-map localization against a PCD map produced by GLIM. |
| [`dlio/`](dlio/) | new in this repo | Convenience metapackage that pulls both packages into a single colcon build. |

Each subpackage has its own README (`GLIM/README.md`, `gicp_localization/README.md`) covering installation, configuration, and per-knob tuning. **This top-level README focuses on what we changed versus upstream and why.**

## Sensor / Vehicle Target

The configs target an AV-24 Cybertruck instrumented with:

- **3× Luminar Iris LiDAR** — `luminar_front` is the primary sensor; `luminar_left` and `luminar_right` are concatenated into the primary cloud via URDF transforms.
- **Point One Atlas (LG69T) INS** publishing IMU on `/gps_p1/imu` (`imu_calibrated`: sensor-level bias/scale/misalignment removed by FusionEngine firmware, gravity present, no fused orientation) and odometry on `/gps_p1/filtered_odom`. Atlas firmware projects both the IMU and the INS pose to the primary antenna phase centre, so the URDF link `gps_antenna_top` is used as both `base_frame` and `imu_frame` in the localization config. RTK quality is gated on the Atlas-reported pose covariance.
- **RTK GPS** — the FusionEngine INS itself; no separate raw RTK topic is needed for localization.
- Optional camera (used only by extension modules).

All sensor extrinsics are resolved at runtime from [`av24.urdf`](av24.urdf); the `*_frame` strings in the configs are URDF link names, not free-form labels.

Current localization scope is intentionally single-source Point One Atlas. Earlier
project notes mention NovAtel and VectorNav GNSS integration, but
`gicp_localization` no longer subscribes to either; adding them back is future
work and needs a fresh source-selection and fix-status design rather than a
topic remap.

### GNSS lever-arm policy

Atlas firmware compensates the IMU-to-antenna lever arm internally, so the software side stays **off** to avoid double-compensation. The disable is explicit in three independent places — any one is sufficient:

1. **Config flag** — `GLIM/glim_ext/config/config_gnss_global.json` sets `"enable_lever_arm": false`. This is the grep-able single source of truth.
2. **Empty antenna frame** — same file sets `"urdf_gnss_frame": ""`. With this empty, the URDF lookup is skipped and `t_imu_gnss` stays zero even if the flag check were bypassed.
3. **Even though the module is loaded** — `libgnss_global.so` is enabled in `GLIM/glim/config/config_ros.json` (it provides the RTK position priors and the `T_world_utm.txt` export) — items 1 and 2 keep its lever-arm math disabled.

To verify the disable in one command:

```bash
grep enable_lever_arm GLIM/glim_ext/config/config_gnss_global.json
# expected: "enable_lever_arm": false,
```

If the module ever loads with this config, it logs `lever-arm compensation explicitly disabled via gnss.enable_lever_arm=false; t_imu_gnss=0` at startup. If the GNSS extension is ever turned back on, verify the receiver's `LEVERARMCONFIG` state first and flip the flag accordingly — only one side should be doing the correction.

### Recovery during GICP failures

In low-feature stretches the localizer first falls back to IMU dead-reckoning. If GICP keeps rejecting, the node snaps pose and velocity to the latest Atlas INS sample that passed the pose-covariance quality gate. If the gate rejects (Atlas covariance above the configured thresholds), GT samples are dropped and the node stays on IMU dead-reckoning until either LiDAR geometry or RTK quality recovers.

### Initialization: RTK-driven IMU calibration

By default the localizer uses the post-gate Atlas GT odom stream to calibrate gyro/accel biases while the vehicle is moving, and seeds pose+velocity from the first high-quality sample rather than assuming the vehicle is stationary. Falls back to the legacy stationary calibration if no gated GT odom is received within a configurable timeout. With `localization/rtk_gate/enable=true`, the localizer inspects `pose.covariance` on every `/gps_p1/filtered_odom` sample and drops anything that exceeds `max_pose_var_xy` / `max_pose_var_z`. Knobs live under `localization/rtk_init/*` and `localization/rtk_gate/*` in the localization yaml.

### Remaining tuning work

- **LiDAR-specific hyperparameter tuning.** Motion-model tuning is in place for race-car dynamics, but the lidar density/range parameters (preprocessing downsample target, voxel-resolution fade horizons) are still close to GLIM's defaults, which were chosen for a lower-density rotary lidar at indoor-to-short-outdoor ranges. Retuning these for Luminar is on the TODO list; current values are workable but not optimal.

### Diagnostic: silent IMU subscription failures

When the `imu_topic:=` launch arg points at a non-existent topic, the subscription is created but no callback fires and historically there was nothing in the log to explain it. The localizer now runs a periodic health check that warns (every 3 s, until the first IMU arrives) with the resolved topic name and whether 0 publishers exist — surfacing the typo case immediately. The timer self-cancels on first IMU receipt.

## Workflow

> **See [PIPELINE.md](PIPELINE.md) for the complete, tested, end-to-end
> recipe** (bag prep → mapping → PCD export → localization replay →
> evaluation) including the `/atlas/*` → `/gps_p1/*` conversion step that the
> raw AV-24 recordings require.

Raw rosbags, generated maps, and `dlio_data/` outputs are intentionally not
versioned. For public clones, copy `dlio.env.example` to
`dlio.env`, fill in the local dataset paths, then run the pipeline
wrapper. The local `dlio.env` file is ignored by git.

```bash
cp dlio.env.example dlio.env
${EDITOR:-nano} dlio.env
scripts/run_dlio_pipeline.sh
```

`dlio.env.example` is the committed template. `dlio.env` is your private
workstation file; keep local mount points, selected run names, and optional
prepared-bag overrides there. `scripts/run_dlio_pipeline.sh` reads it
automatically, and any CLI flag you pass still overrides the file for that
one run.

1. **Record** a bag containing IMU + LiDAR + GNSS topics during a driving session.
1b. **Prep** the bag with `scripts/prep_bag.py` (topic conversion, IMU/GNSS
   re-stamping, UTM odometry — see PIPELINE.md §1).
2. **Map** offline with GLIM:
   ```bash
   DATA="${DATA:-./dlio_data}"
   PREPPED_BAG="$DATA/run_5_prepped"
   DUMP_DIR="$DATA/run_5_dump"
   ros2 run glim_ros glim_rosbag "$PREPPED_BAG" --ros-args -p dump_path:="$DUMP_DIR"
   ```
   Outputs `graph.bin`, `traj_lidar.txt`, `odom_lidar.txt`, numbered submap point clouds, and `T_world_utm.txt` (GNSS-to-map SE(3)) into `dump_path`.
3. **Convert** submaps into a single PCD map. Scripted route (used by the automated pipeline): `ros2 run glim_ros glim_dump_to_pcd "$DUMP_DIR" "$DATA/run_5_map.pcd"`. QA route (recommended before freezing a production map): open the dump in `glim_ros offline_viewer`, inspect/re-optimize/close loops, export PLY, then `gicp_localization/scripts/convert_ply_to_pcd.py` — see "Why the offline_viewer step is manual" below for what the GUI pass buys you.
4. **Localize** online against that PCD map with `gicp_localization`. Point the launch file at the PCD and (optionally) the matching `T_world_utm.txt`.

### Why the offline_viewer step is manual

A reviewer reasonably asks: why not auto-merge the per-submap directories into a single PCD with a script? Because the viewer pass is the QA stage for the mapping output, and skipping it would silently push bad maps into the localizer:

- **Visual inspection** of the assembled map before it's frozen as the localization reference catches drift, ghosting, and bad submaps that would otherwise propagate into GICP at runtime.
- **Post-hoc global optimization** — the viewer prompts "Do optimization?" on load (see `offline_viewer.cpp:191`) and re-runs the iSAM2 backend over the full graph, which can improve the dump beyond what the online pass produced.
- **Manual loop closure** — `manual_loop_close_modal` lets the operator add constraints when the automatic detector misses a loop (common on long highway runs with weak geometry).

A blind `merge_glim_submaps.py` would skip all three and bake any unresolved drift into the PCD. Adding such a script as a dev-only "quick-look" mode is reasonable, but it must not become the default mapping→localization handoff.

## Build

ROS 2 Jazzy + colcon on a local Ubuntu 24.04 environment. Run commands from
the repo root so config files that reference `av24.urdf` resolve correctly.

```bash
source /opt/ros/jazzy/setup.bash
make install-deps
make install-gtsam-points-cuda
make build
source install/setup.bash
```

Headline dependencies (per-package READMEs go deeper):

- GTSAM 4.2, gtsam_points (GPU factors), Iridescence, Eigen3, PCL, OpenMP, nlohmann::json, spdlog
- Optional: CUDA 11.8+ (GPU acceleration), OpenCV
- Python bag/pipeline tools: `mcap`, `mcap-ros2-support`, `pyproj`, `numpy`, and `matplotlib`.
  `make install-deps` installs them with:
  `python3 -m pip install --user --break-system-packages mcap mcap-ros2-support pyproj numpy matplotlib`

If `ros2 pkg prefix glim` does not point inside this workspace's `install/`, an apt-installed `ros-jazzy-glim-*` package is being picked up instead of this fork — re-source `install/setup.bash` **after** `/opt/ros/jazzy/setup.bash`. The same caveat applies to `gicp_localization` if a sibling workspace is also sourced.

## Quick Reference

```bash
export DATA="${DATA:-./dlio_data}"

# Live SLAM with real sensors (config_path defaults to the glim package's config/)
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=config

# Offline bag → map (ROS 2 mcap input)
PREPPED_BAG="$DATA/run_5_prepped"
DUMP_DIR="$DATA/run_5_dump"
ros2 run glim_ros glim_rosbag "$PREPPED_BAG" --ros-args -p dump_path:="$DUMP_DIR"

# Offline pcap → map (raw Luminar pcap + IMU/GNSS from a sibling mcap)
PCAP_PATH="/path/to/luminar_capture.pcap"
MCAP_BAG="/path/to/sibling_bag.mcap"
PCAP_DUMP_DIR="$DATA/pcap_dump"
ros2 run glim_ros glim_pcap_rosbag "$PCAP_PATH" "$MCAP_BAG" --ros-args -p dump_path:="$PCAP_DUMP_DIR"

# Inspect a saved map
ros2 run glim_ros offline_viewer

# GICP localization against a pre-built PCD map
# (single-source P1 design: IMU + GT odom both from Atlas, at gps_antenna_top;
#  gt_odom must be in the MAP frame — for bag replay use
#  gicp_localization/scripts/utm_to_map_odom.py, see PIPELINE.md §4)
MAP_PATH="$DATA/run_5_map.pcd"
UTM_TF="$DATA/run_5_dump/T_world_utm.txt"
ros2 launch gicp_localization localization_with_tf.launch.py rviz:=true \
    pointcloud_topic:=/luminar_front/points \
    imu_topic:=/gps_p1/imu \
    gt_odom_topic:=/gps_p1/filtered_odom_map \
    map_path:="$MAP_PATH" \
    utm_transform_path:="$UTM_TF"

# Re-open a previous localization result in RViz without rerunning GICP/rosbag.
scripts/show_cached_error_viz.sh "$DATA/run_3_loc_gnss_live" "$MAP_PATH"
```

---

## Key Changes vs. Upstream

The two packages started from different upstream codebases and diverged for different reasons. This section summarizes the substantive deltas — small config tweaks, log-level changes, and routine refactors aren't enumerated here; consult `git log` for the exhaustive list.

### GLIM (vs. `koide3/glim`, `glim_ext`, `glim_ros2`)

Upstream GLIM publishes `glim`, `glim_ext`, and `glim_ros2` as three sibling repos. This fork keeps them together under `GLIM/` and adds:

**Sensor / preprocessing**

- **Multi-LiDAR concatenation (`lidar_concat`).** New module in `glim_ros2` (`include/glim_ros/lidar_concat.hpp`) that subscribes to N aux LiDAR topics, time-aligns each scan to the primary clock, transforms aux points into the primary frame via URDF, **rebases per-point timestamps** so the concatenated cloud has a single monotonic time base, and emits a single merged cloud to the rest of the pipeline. Includes a validation step that **rolls back the aux-merge append** if the merged cloud fails sanity checks, instead of letting a malformed cloud poison odometry (commit `52f88cb`).
- **URDF-based extrinsic resolution.** Sensor extrinsics (`T_lidar_imu`, inter-LiDAR transforms, IMU↔GNSS) are read from a runtime URDF instead of hand-edited JSON. The relevant configs (`config_sensors.json`) reference *URDF link names*; the loader walks the URDF at startup. Removes the previous hard-coded URDF path.
- **`flip_points_y` preprocessing** flag (`config_sensors.json` → `glim_ros.cpp`) for mirrored-installed LiDARs.
- **Per-point timestamp rebasing fix** when merging multi-LiDAR clouds (commit `7f5a6d9`). Without this the merged cloud had a non-monotonic stamp field that broke deskewing.

**Mapping / odometry**

- **INS-driven odometry mode** for sparse-feature stretches (commit `c26c8b0`). Lets the optimizer lean on INS odometry when LiDAR geometry is degenerate (e.g. open sky and runway-like surfaces).
- **Race-car drift tuning** in `glim/config/` (commit `20ba88d`). Defaults relaxed to admit higher angular rates and lateral slip than the road-car defaults assume.

**GNSS extension (`glim_ext/modules/mapping/gnss_global`)**

- **`T_world_utm.txt` export** of the recovered GNSS-to-map SE(3) once GNSS alignment initializes. Downstream localizers (including `gicp_localization` here) consume this file to publish poses in a `utm` frame in addition to `map`.
- **URDF lever-arm support** (commit `50ae6ae`/`50ae...50a...50aa50a` — see `git log`): the IMU→GNSS lever-arm is taken from the URDF rather than from a manual offset in the config.
- **Orientation prior** mode: optionally constrain map yaw directly from GNSS heading.
- **Strip stale GNSS rotation priors on graph reload** (commit `6a50632`) so a re-opened graph doesn't double-apply an orientation constraint that no longer matches the live frame.
- **Warn when URDF IMU↔GNSS rotation breaks the lever-arm assumption** (commit `622271f`). The lever-arm math assumes IMU and GNSS share orientation; if the URDF says otherwise the user is told instead of silently getting biased corrections.
- Switched the noise model expression from `Isotropic::Information(diagonal)` (which silently dispatched to `Gaussian::Information(Matrix)` through inheritance) to `Diagonal::Precisions(vector)` (commit `d8b2809`). Same numerical result, more honest signature — see `AGENTS.md §1` for the reasoning trail.

**Offline tooling**

- **`glim_pcap_rosbag`** (`glim_ros2/src/glim_pcap_rosbag.cpp` + `iris_pcap_reader.cpp`). Reads raw Luminar `.pcap` files alongside a sibling mcap (for IMU and GNSS) and runs offline mapping directly, skipping the intermediate "decode pcap into a bag" step. Useful when the live-recorded mcap is missing LiDAR or had a decode hiccup.
- **`scripts/merge_luminar_pcap.py`** remains as a slower Python reference/debug fallback for validating Luminar packet decoding, PointCloud2 layout, and timestamp alignment against the C++ reader.

### gicp_localization (vs. the vectr-ucla DLIO line)

`gicp_localization` is built around a vendored `nano_gicp`. Compared to a stock DLIO odometry node turned into a localizer, this fork adds:

**Scan-to-map vs. scan-to-submap**

- **Single pre-built PCD map.** No submap stitching at runtime — the map is loaded once and never grows. Trades adaptability for a small, predictable working set.
- **Multi-LiDAR concatenation** (mirrors the GLIM-side feature). Subscribes to N aux LiDARs, transforms via URDF, concatenates onto the primary cloud's clock.

**Robustness against degenerate geometry**

- **Layered rejection gates** on every GICP solve:
  1. Hard fitness reject (`gicp/fitnessRejectThreshold`).
  2. **Combined hessian-degeneracy gate.** Fires only when the hessian condition number is high *and* one of `fitness`/`trans`/`rot` warn floors is crossed — high hessian alone is fine if GICP barely moved, but high hessian combined with a large correction is the slide-along-unconstrained-axis signature (commit `4a594a9`).
  3. Large-jump reject vs. the IMU-predicted prior.
- **IMU dead-reckoning fallback.** Rejected scans propagate from the IMU-integrated prior, not by freezing at the last accepted pose — transient corner failures don't cascade into a stuck pose.
- **GT-driven pose recovery** (optional, off by default). When GICP rejects N scans in a row, optionally snap pose + velocity to a time-matched GT odom sample (composed through TF into `base_frame`) so GICP can re-acquire from a known-good state (commit `83b48a4`).
- **`getFitnessScore` correctness fixes.** Cleared `sq_distances_` per align so stale distances couldn't leak into the score (commit `d949e64`); cached `sq_distances_` reused inside `NanoGICP::getFitnessScore` to avoid recomputing nearest neighbors (commit `1e27b84`).

**Geometric observer**

- **Observer + IMU pipeline aligned to upstream DLIO design** (commit `db5cda8`). The original lift-and-shift had subtle differences in how the geometric observer was driven; this commit brings the data path back in line with DLIO's reference implementation so IMU dead-reckoning is mathematically consistent with scan corrections.

**Initialization**

- **GT-bootstrapped initial pose** (commit `add7a54`). The first message on the GT odom topic seeds the localizer; works for any bag start-offset without hand-tuning numerics or clicking in RViz.
- Three init paths in priority order: GT bootstrap → numeric pose from YAML (with `frame: "lidar"` mode that auto-applies `inv(T_base_lidar)` for direct pasting from GLIM's `traj_lidar.txt`) → RViz "2D Pose Estimate".

**Output frames**

- **UTM-frame publishing.** If `T_world_utm.txt` (from the GLIM run that built the map) is configured, the node publishes pose/odom/path in a `utm` frame alongside `map`. Lets downstream consumers consume world-referenced poses without re-deriving the transform.
- **TF policy:** the `map → base_link` TF broadcast is disabled by default (commit `9cc18d7`) to avoid fighting other publishers; downstream nodes consume the published `nav_msgs/msg/Odometry` instead.

**Operational defaults**

- Verbose logging, debug topic publication, per-scan jump/scan logs, and outgoing point-cloud topics are all **off by default** (commits `d492a69`, `b5116d3`, `100011c`). The pipeline is quiet and lean unless you explicitly enable diagnostics.

---

## Repo Layout

```
repo-root/
├── GLIM/                # SLAM workspace (glim, glim_ext, glim_ros2)
├── gicp_localization/   # Map-based localization package
├── dlio/                # Convenience metapackage
├── av24.urdf            # Vehicle URDF (drives all sensor extrinsics)
├── scripts/             # Bag-prep, map-merge, and analysis helpers
├── profiling_logs/      # Resource-profile CSVs + comparison plots
├── CLAUDE.md            # Developer-facing project summary
├── AGENTS.md            # Notes for AI reviewers (false positives, watch-conditions)
└── README.md            # This file
```

## License

GLIM and `gtsam_points` are MIT-licensed; GTSAM is BSD. See the upstream repositories and `GLIM/README.md` for full attributions.
