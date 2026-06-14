# GLIM ROS2 Workspace

ROS2 workspace for **GLIM** (Graph-based LiDAR-Inertial Mapping) maintained as an `airacingtech` fork of the upstream `koide3/GLIM` project family.

## Overview

This directory is the GLIM workspace inside the [`augcog/DLIO_plusplus`](https://github.com/augcog/DLIO_plusplus) monorepo and contains:
- `glim` for the core SLAM framework
- `glim_ext` for extension modules
- `glim_ros2` for ROS2 integration

**Target sensor stack:** AV-24 Cybertruck with three Luminar Iris LiDAR (front + left + right concatenated) and the **Point One Nav Atlas (LG69T) dual-antenna RTK-INS**. All GNSS, RTK, and IMU input comes from Atlas — Atlas projects its IMU output and INS pose solution to the primary GNSS antenna phase centre (URDF link `gps_antenna_top`) via firmware lever-arm, so GLIM consumes both at the same body frame with no second lever-arm step. IMU rate is 99 Hz, RTK is delivered at cm-level horizontal / sub-cm vertical when FIXED.

### Differences From Upstream GLIM

- This fork keeps `glim`, `glim_ext`, and `glim_ros2` together inside the parent `DLIO_plusplus` monorepo instead of as separate sibling repositories.
- `glim` includes optional `flip_points_y` preprocessing support for mirrored LiDAR clouds.
- `glim` includes packed LiDAR per-point timestamp parsing support for `UINT8[8]` timestamp fields (the Luminar Iris layout).
- `glim_ext` includes the GNSS-related modules and configs from the synced `glim_ws` copy.
- `glim_ext` preserves export of the recovered GNSS-to-map SE(3) transform as `T_world_utm.txt` when GNSS alignment is initialized.
- ROS2 and configuration defaults in this fork are sized for the Atlas dual-antenna INS and the AV-24 vehicle — covariance gates, GNSS prior precisions, IMU noise, and the LiDAR-IMU extrinsic all assume that specific stack.

### Key Features

- **INS-driven offline map-building default** — `config.json` currently selects `config_odometry_ins.json`, which interpolates `/gps_p1/filtered_odom_rtk_fixed` for the mapping trajectory and deskews scans with Atlas INS motion. This is the stable Putnam-track path for RTK-good bags; it pauses where the RTK-fixed stream has gaps.
- **RTK-FIXED-only GNSS anchoring** — `libgnss_global.so` consumes Atlas samples whose pose covariance indicates FIXED-integer quality and turns them into position-prior factors on the global graph. The same stream also provides the current INS-driven trajectory.
- **LiDAR+IMU dropout-tolerant mode remains available** — switching `config.json` to `config_odometry_gpu.json` (or CPU/CT variants) makes VGICP+IMU the primary odometry and uses GNSS factors as sparse global anchors. That mode is useful for GNSS-denied terrain but has more drift risk on this feature-poor track.
- **Geo-referenced output** — `T_world_utm.txt` saves the SE(3) transform from the local UTM frame to GLIM's map/world frame for GICP localization and post-processing.

The exact behavior of this fork should be taken from the checked-in config and source files in this repository, not assumed to match upstream defaults.

## Repository Structure

```
.
├── glim/          # Core SLAM framework
│   ├── config/    # Configuration files (optimized for cybertruck)
│   ├── include/   # Header files
│   └── src/       # Source code
├── glim_ext/      # Extension modules
│   ├── modules/
│   │   └── mapping/
│   │       └── gnss_global/  # RTK-GPS constraint module
│   └── config/    # Extension configs
└── glim_ros2/     # ROS2 interface
    ├── launch/    # Launch files
    └── src/       # ROS2 nodes
```

## Dependencies

### System Requirements
- Ubuntu 22.04 (recommended)
- ROS2 Humble
- CUDA 11.8+ (optional, for GPU acceleration)

### Core Dependencies
```bash
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libboost-all-dev \
  libfmt-dev \
  libomp-dev \
  libmetis-dev \
  ros-humble-tf2-eigen \
  ros-humble-pcl-ros
```

### GTSAM (Required)
```bash
# Install GTSAM
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc)
sudo make install
```

### gtsam_points (Required)
```bash
# Install gtsam_points
git clone https://github.com/koide3/gtsam_points.git
cd gtsam_points
mkdir build && cd build
cmake .. -DBUILD_WITH_CUDA=ON  # Set OFF if no GPU
make -j$(nproc)
sudo make install
```

### iridescence (Optional, for visualization)
```bash
git clone https://github.com/koide3/iridescence.git
cd iridescence
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Building

### Clone and Build
```bash
# Clone the parent monorepo
cd ~/ros2_ws/src
git clone https://github.com/augcog/DLIO_plusplus.git

# Build the GLIM packages (use --packages-up-to to limit scope, or omit to build everything)
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to glim_ros

# Source the workspace
source install/setup.bash
```

### Build Options
- **CPU-only build**: Remove `-DBUILD_WITH_CUDA=ON` from gtsam_points build
- **Debug build**: Use `-DCMAKE_BUILD_TYPE=Debug` instead of Release

## Usage

### Mapping pipeline overview

```
  /gps_p1/filtered_odom ──▶ RTK covariance gate ──▶ /gps_p1/filtered_odom_rtk_fixed
                                                               │
                                                               ├──▶ libodometry_estimation_ins
                                                               │    (primary trajectory + deskew)
                                                               │
                                                               └──▶ libgnss_global.so
                                                                    (position priors + T_world_utm)

  /luminar_front/points ───────────────────────────▶ GLIM mapping graph / map output
```

The checked-in default is deliberately INS-driven for RTK-clean offline map
builds. The mapping trajectory follows `/gps_p1/filtered_odom_rtk_fixed`, so
the map is stable on feature-poor Putnam sections but pauses if the RTK-fixed
stream stops. `libgnss_global.so` still writes the UTM-to-map alignment and
adds position priors to the global graph.

The older dropout-tolerant LiDAR+IMU mode is still available by selecting
`config_odometry_gpu.json` (or CPU/CT variants) in `glim/config/config.json`.
In that mode odometry runs every scan and GNSS factors become sparse anchors,
but that is not the current default because it drifted on the high-speed,
low-feature Putnam bags.

### Startup procedure (RTK FIXED required at session start)

The current INS-driven mapping path should start with Atlas in RTK FIXED. It
uses `/gps_p1/filtered_odom_rtk_fixed` as the trajectory source, so no LiDAR
frame is inserted until that stream covers the scan. In LiDAR+IMU mode, odometry
can run before GNSS is fixed, but the first GNSS prior should still land while
RTK is FIXED so the global frame is anchored cleanly.

**Step 1 — Park with sky view, wait for Atlas FIXED:**
- Stop the vehicle at the intended map origin with a clear sky view.
- Watch Atlas's status display or `ros2 topic echo /gps_p1/filtered_odom` and look for `pose.covariance[0]` dropping under ~1×10⁻³ m² (≈ 3 cm σ). Typical FIXED acquisition under open sky is 30 s — 2 min.

**Step 2 — Launch the RTK-FIXED pre-filter:**
```bash
python3 gicp_localization/scripts/rtk_fixed_odom_filter.py
```
Expect a log line:
```
RTK-FIXED odometry pre-filter ready: '/gps_p1/filtered_odom' -> '/gps_p1/filtered_odom_rtk_fixed'
First INS sample received at stamp=… cov=[…] -> FIXED
```
If `-> NOT FIXED` instead, wait. The filter will log the transition the moment Atlas reaches FIXED.

**Step 3 — Launch GLIM:**
```bash
# Live
ros2 launch glim_ros glim_ros.launch.py config_path:=config

# Or replay an existing bag
ros2 launch glim_ros glim_ros.launch.py config_path:=config use_sim_time:=true
ros2 bag play <your_bag_file.db3> --clock
```
With the current INS-driven config, map points appear once RTK-fixed odometry
covers the LiDAR scan window. If you switch to LiDAR+IMU odometry, you should
instead see `estimate initial IMU state` from loose-init followed by GNSS-prior
factor insertion when a FIXED sample lands.

**Step 4 — Drive the track:**
Watch the viewer; with the current INS-driven config the map grows while
RTK-fixed odometry is available and pauses through gaps. The filter will print
FIXED/NOT_FIXED transitions whenever Atlas's RTK quality crosses the covariance
gate. In LiDAR+IMU mode, the map can keep extending through those gaps.

### RTK-denied terrain strategy for LiDAR+IMU mode

This section applies when `config.json` is switched from the current
INS-driven default to `config_odometry_gpu.json` / CPU / CT. The current
`config_odometry_ins.json` path instead pauses when `/gps_p1/filtered_odom_rtk_fixed`
has gaps.

GLIM is designed for tracks that include GNSS-denied passages (tunnels, dense foliage, urban canyons, mountain switchbacks where multipath kills FIXED quality temporarily). The behaviour is:

**During the dropout:**
- `rtk_fixed_odom_filter.py` stops forwarding samples. It logs `RTK transition: FIXED -> NOT_FIXED at stamp=… cov=[…]`.
- `libgnss_global.so` receives no new messages → no new GNSS factors added to the iSAM2 graph.
- **LiDAR+IMU odometry keeps running every scan.** VGICP between-factors + ImuFactor preintegration drive the trajectory forward. The map perimeter keeps extending — every new scan's points get inserted at the LiDAR+IMU-estimated pose, with no holes.
- The trajectory in the GNSS-denied section gradually drifts (sub-metre over hundreds of metres on a calibrated MEMS-grade IMU + multi-LiDAR Luminar; multi-metre on kilometre-scale dropouts).

**On RTK FIXED reacquisition (exit the tunnel):**
- The filter resumes forwarding. Logs `RTK transition: NOT_FIXED -> FIXED at stamp=… cov=[…]`.
- The next forwarded message becomes a fresh GNSS prior factor on the current pose.
- iSAM2 detects the disagreement between the drifted current pose and the GNSS anchor. The incremental smoother **redistributes the error retroactively across all the poses inside the dropout**, satisfying both the LiDAR/IMU consistency constraints and the GNSS anchor at exit.
- Map points in the dropout segment shift with their poses. The result is a continuous, drift-corrected map.

This is why mapping **never** falls back to pure IMU dead-reckoning (would drift visibly within seconds) or to LiDAR-only odometry (would create a discontinuity at rejoin). The native LiDAR+IMU fusion already handles GNSS loss as a first-class scenario.

If the dropout is long enough or feature-poor enough that residual error matters, two follow-ups help:

- **Re-traverse the dropout area** on a later lap. Loop closure factors get added, further refining the dropout trajectory.
- **Bump `smoother_lag`** in `config_odometry_gpu.json` from 2 s to a duration longer than your expected dropout (e.g. 10–30 s for a ~200 m tunnel at 30 m/s). iSAM2 needs the dropout to fall inside the active smoothing window for retroactive correction to work.

### Running modes

**Live mode (with real sensors):**
```bash
# Terminal 1: launch the RTK-FIXED pre-filter
python3 gicp_localization/scripts/rtk_fixed_odom_filter.py

# Terminal 2: launch GLIM
ros2 launch glim_ros glim_ros.launch.py config_path:=config
```

**Offline mode (rosbag processing):**
```bash
ros2 run glim_ros glim_rosbag <rosbag_path> --ros-args -p dump_path:=<output_directory>
```
Note: `glim_rosbag` plays the bag and processes it in one step. Run the pre-filter in a separate terminal first (it'll pick up the played `/gps_p1/filtered_odom`).

**Offline mode (rosbag replay with launch):**
```bash
# Terminal 1: launch GLIM
ros2 launch glim_ros glim_ros.launch.py config_path:=config use_sim_time:=true

# Terminal 2: launch the pre-filter
python3 gicp_localization/scripts/rtk_fixed_odom_filter.py --ros-args -p use_sim_time:=true

# Terminal 3: play the rosbag
ros2 bag play <your_bag_file.db3> --clock
```

**With logging:**
```bash
ros2 launch glim_ros glim_ros.launch.py config_path:=config use_sim_time:=true | tee /tmp/glim_live.log
```

### Monitoring RTK and GNSS Alignment

Pre-filter messages (live tracking of RTK quality):
```
[rtk_fixed_odom_filter] RTK-FIXED odometry pre-filter ready: ...
[rtk_fixed_odom_filter] First INS sample received at stamp=… cov=[…] -> FIXED
[rtk_fixed_odom_filter] RTK transition: FIXED -> NOT_FIXED at stamp=…   # entering dropout
[rtk_fixed_odom_filter] RTK transition: NOT_FIXED -> FIXED at stamp=…   # exiting dropout
```

GLIM `gnss_global` messages (global anchoring):
```
[gnss_global] initializing GNSS global constraints
[gnss_global] gnss_global_config_path=<path>
[gnss_global] T_world_utm=<transformation>           # first anchor
[gnss_global] insert <N> GNSS prior factors          # debug level
[gnss_global] saved T_world_utm (4x4 SE(3)) to: <dump_path>/T_world_utm.txt
```

GLIM odometry messages (only when using LiDAR+IMU odometry):
```
[odometry_estimation] estimate initial IMU state          # ~5 s after start
[validate_imu] residual=…                                 # per-keyframe IMU sanity
```

### Map Output

When using `glim_rosbag`, maps are saved to the specified `dump_path`:
```bash
ros2 run glim_ros glim_rosbag <rosbag> --ros-args -p dump_path:=<output_directory>
```

Each directory contains:
- `graph.txt` / `graph.bin` - Pose graph structure
- `000000/`, `000001/`, ... - Submap directories with point clouds
- `odom_lidar.txt` / `odom_imu.txt` - Odometry trajectories
- `traj_lidar.txt` / `traj_imu.txt` - Optimized trajectories
- `T_world_utm.txt` - **SE(3) transformation from GNSS/UTM to odom frame** (if GNSS enabled)
- `config/` - Configuration files used for this map

## Configuration

### Main Configuration Files

**GLIM Core (`glim/config/`):**
- `config.json` — Main config (selects which odometry estimator to load)
- `config_ros.json` — ROS topics and extension modules
- `config_sensors.json` — Sensor noise + IMU/LiDAR extrinsics (`T_lidar_imu`)
- `config_odometry_ins.json` — **Currently selected** offline map-building estimator (Atlas INS trajectory; pauses on RTK-fixed gaps)
- `config_odometry_gpu.json` — Optional VGICP + IMU estimator for dropout-tolerant mapping
- `config_odometry_{cpu,ct}.json` — Other alternatives (CPU-only VGICP, continuous-time)
- `config_preprocess.json` — Point cloud preprocessing
- `config_sub_mapping_cpu.json` / `config_global_mapping_cpu.json` — Current mapping backends

**GNSS Extension (`glim_ext/config/`):**
- `config_gnss_global.json` — RTK prior factor topic and precision

### Key Parameters (Atlas-tuned values, AV-24 deployment)

**Atlas-derived noise envelope** — measured on a known-RTK-fixed AV-24 bag (`run_2`, 17 min):

| Field | Median | p95 | Equivalent σ |
|---|---|---|---|
| `pose.covariance[0]` (x) | 2.8×10⁻⁵ m² | 4.1×10⁻⁵ m² | ~5–6 mm |
| `pose.covariance[7]` (y) | 4.2×10⁻⁵ m² | 5.7×10⁻⁵ m² | ~6–8 mm |
| `pose.covariance[14]` (z) | 1.0×10⁻⁴ m² | 1.3×10⁻⁴ m² | ~1.0–1.1 cm |
| IMU stationary accel σ | 3 mm/s² @ 99 Hz | — | density ~3×10⁻⁴ m/s²/√Hz |
| IMU stationary gyro σ | 7 mrad/s @ 99 Hz | — | density ~7×10⁻⁴ rad/s/√Hz |

The IMU and GNSS noise parameters below sit ~3× looser than these measured values, to leave headroom for transients (vibration spikes, multipath bursts) that the per-message covariance doesn't capture.

**RTK-FIXED pre-filter** (`gicp_localization/scripts/rtk_fixed_odom_filter.py` params):
```yaml
input_topic:    /gps_p1/filtered_odom         # raw Atlas INS pose
output_topic:   /gps_p1/filtered_odom_rtk_fixed
max_pose_var_xy: 0.001    # m^2 — admit only Atlas FIXED quality (~3 cm σ allowed)
max_pose_var_z:  0.005    # m^2 — Z naturally looser (~7 cm σ allowed)
```
Loosen these to admit RTK-FLOAT if your sky view is poor; tighten to reject Atlas's occasional bias-walk during long FIXED stretches.

**GNSS prior factor precision** (`config_gnss_global.json`):
```json
{
  "gnss": {
    "gnss_topic": "/gps_p1/filtered_odom_rtk_fixed",
    "gnss_msg_type": "nav_msgs/msg/Odometry",
    "min_baseline": 5.0,
    "enable_orientation_prior": false,
    "prior_inf_scale": [1e6, 1e6, 1e5],
    "enable_lever_arm": false
  }
}
```
- `prior_inf_scale` is **precision** (1/variance), not sigma. Equivalent sigmas: σ_x = σ_y ≈ 1 mm, σ_z ≈ 3 mm. This intentionally pins RTK-clean INS-driven maps tightly to Atlas; loosen if GNSS factors visibly fight LiDAR consistency in LiDAR+IMU mode.
- `enable_orientation_prior: false` because Atlas does not populate the `sensor_msgs/Imu.orientation` field; INS attitude comes through IMU preintegration on the LiDAR+IMU side instead.
- `enable_lever_arm: false` because Atlas firmware already projects to `gps_antenna_top`. Software-side lever-arm would double-compensate.

**IMU noise** (`config_sensors.json`, tuned for Atlas `imu_calibrated`):
```json
{
  "sensors": {
    "imu_acc_noise":  0.05,   // m/s^2/sqrt(Hz) — ~4x tighter than uncalibrated MEMS
    "imu_gyro_noise": 0.01,   // rad/s/sqrt(Hz) — ~5x tighter
    "imu_bias_noise": 1e-5,   // Atlas firmware bias is firmware-stable
    "imu_int_noise":  0.001,
    "urdf_imu_frame": "gps_antenna_top"
  }
}
```
Revert to 0.2 / 0.05 if you ever re-point GLIM at a raw MEMS IMU stream.

**Current odometry estimator** (`config_odometry_ins.json`):
```json
{
  "odometry_estimation": {
    "so_name": "libodometry_estimation_ins.so",
    "urdf_ins_frame": "gps_antenna_top",
    "enable_lidar_refinement": false,
    "max_ins_wait_seconds": 0.05,
    "num_threads": 2
  }
}
```

**Threading (adjust based on your CPU):**
```json
"odometry_estimation": { "num_threads": 2 },
"preprocess":          { "num_threads": 2 }
```
Sub/global mapping use library defaults; tune up if you have spare cores.

### When to retune

| Symptom | Likely fix |
|---|---|
| INS-driven map pauses unexpectedly | Check the RTK-fixed sample count from `prep_bag.py`; the current odometry source only inserts scans covered by `/gps_p1/filtered_odom_rtk_fixed`. |
| Loop closures show >10 cm Z error through dropouts in LiDAR+IMU mode | Tighten `prior_inf_scale[2]` above `1e5` or increase the active smoothing window. |
| GNSS factors visibly tug the trajectory each scan in LiDAR+IMU mode | Loosen `prior_inf_scale` from `[1e6, 1e6, 1e5]` so LiDAR consistency has more room. |
| Dropout segment shows visible kink after iSAM2 finishes in LiDAR+IMU mode | Bump `smoother_lag` to >= longest expected dropout duration (5x-10x by default). |
| Map-viewer points jitter on still vehicle in LiDAR+IMU mode | Tighten `imu_acc_noise` further (e.g. 0.02) or check vibration coupling. |
| IMU bias jitters in LiDAR+IMU mode | Keep `imu_bias_noise` small; if needed, set `fix_imu_bias: true` in the selected LiDAR+IMU odometry config. |
| Pre-filter never reaches FIXED | Loosen `max_pose_var_xy` / `max_pose_var_z` to admit RTK-FLOAT for that session |

## Coordinate Transformation

The GNSS module automatically computes the transformation between:
- **SLAM world frame** (local mapping frame)
- **GPS/UTM frame** (global coordinates)

**Transformation variable:** `T_world_utm`

This transformation is:
- Computed once per session after achieving `min_baseline` travel distance (currently `5.0 m` in `config_gnss_global.json`)
- Remains static throughout the mapping run
- **Automatically saved to `T_world_utm.txt` in the map directory**

**Convert map point to GPS:**
```cpp
Eigen::Vector3d gps_position = T_world_utm.inverse() * map_position;
```

**Convert GPS to map:**
```cpp
Eigen::Vector3d map_position = T_world_utm * gps_position;
```

The transformation is logged when alignment initializes:
```
[gnss_global] T_world_utm=<transformation>
```

And saved to the map directory when mapping completes:
```
[gnss_global] saved T_world_utm (4x4 SE(3)) to: <dump_path>/T_world_utm.txt
```

## Troubleshooting

### Pre-filter never reports FIXED
- Atlas itself hasn't reached FIXED. Check `ros2 topic echo /gps_p1/filtered_odom --once` and inspect `pose.covariance[0]`; it should drop to ~1×10⁻⁴ m² or below.
- For poor sky-view sessions, raise the pre-filter thresholds to admit FLOAT: launch with `-p max_pose_var_xy:=0.05 -p max_pose_var_z:=0.1`.

### Map not anchoring to global frame (no `T_world_utm` log line)
- The pre-filter is running but `libgnss_global.so` isn't subscribing. Check `extension_modules` in `config_ros.json` includes `libgnss_global.so`.
- Verify `gnss_topic` in `config_gnss_global.json` matches the filter's output (`/gps_p1/filtered_odom_rtk_fixed` by default).
- Vehicle hasn't traveled `min_baseline` (5.0 m by default) since the first GNSS factor — `libgnss_global.so` needs two well-separated samples to initialize the SE(3) anchor.

### Map shows discontinuity / kink after a GNSS-denied passage in LiDAR+IMU mode
- Increase `smoother_lag` in `config_odometry_gpu.json` to a duration longer than the dropout. iSAM2 can only redistribute error inside the active smoothing window.
- For very long dropouts, plan re-traversal on a later lap so loop closure factors can refine the trajectory.

### Trajectory drifts visibly during long FIXED stretch in LiDAR+IMU mode
- IMU bias may not be locked. Verify `fix_imu_bias: true` in the selected LiDAR+IMU odometry config.
- Check Atlas's pose covariance is actually still FIXED (`pose.covariance[0]` < 1e-3 m²); transient bias-walk during good FIXED can briefly degrade.

### LiDAR points appear shifted by a fixed offset everywhere
- The IMU/LiDAR extrinsic (`T_lidar_imu` in `config_sensors.json`) is wrong. With Atlas, the IMU lives at `gps_antenna_top` (firmware-projected), so the translation is `luminar_front` (URDF) → `gps_antenna_top` (URDF) = `(−0.95065, −0.005, 0.47194)`. Verify against `av24.urdf`.

### Low performance
- Reduce thread counts if CPU usage is 100%
- Increase downsampling: lower `random_downsample_target` from the default `10000`
- Disable viewers if running headless

### CUDA errors
- Build gtsam_points with `-DBUILD_WITH_CUDA=OFF`
- System falls back to CPU automatically (use `config_odometry_cpu.json` instead of `_gpu.json`)

### Map not saving
- Use `tee` for logging instead of piping through `grep` so the SIGINT shutdown
  sequence reaches GLIM directly: `ros2 launch ... | tee output.log`
- Default dump path is `/tmp/dump`; override with `-p dump_path:=<dir>` and
  check write permissions on the chosen directory

## Credits

This workspace is based on:

- **GLIM** by Kenji Koide
  - Repository: https://github.com/koide3/glim
  - Paper: [Graph-based LiDAR-Inertial Mapping](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf)

- **gtsam_points** by Kenji Koide
  - Repository: https://github.com/koide3/gtsam_points

- **GTSAM** by Georgia Tech
  - Repository: https://github.com/borglab/gtsam

## License

This workspace inherits licenses from its constituent packages:
- GLIM: MIT License
- gtsam_points: MIT License
- GTSAM: BSD License

See individual package directories for full license texts.

## Modifications

This fork includes:
- **Point One Atlas dual-antenna RTK-INS integration** — all GNSS/RTK/IMU input from `/gps_p1/*`, both IMU and pose projected to `gps_antenna_top` by Atlas firmware (no software lever-arm needed).
- **RTK-FIXED-only covariance gate** — `gicp_localization/scripts/rtk_fixed_odom_filter.py` pre-filters Atlas's `/gps_p1/filtered_odom` to admit only FIXED-integer quality before feeding GLIM's GNSS factor source.
- **INS-driven RTK-clean map-building default** — `libodometry_estimation_ins.so` follows Atlas RTK-fixed odometry for stable offline maps on the feature-poor Putnam course.
- **Optional GNSS-denied terrain continuity** — LiDAR+IMU odometry (`libodometry_estimation_gpu.so` or CPU/CT variants) remains available when continuous mapping through RTK gaps is more important than the extra drift risk.
- **Atlas-derived precision tuning** — `prior_inf_scale` and IMU noise parameters are sized against the measured noise envelope of Atlas RTK-FIXED on AV-24 (see *Key Parameters* above).
- Automatic SE(3) transformation saving (`T_world_utm.txt`).
- GNSS module fixes for ROS2 compatibility.
- Enhanced logging for debugging RTK transitions and dropout/recovery behaviour.

## Citation

If you use this work, please cite the original GLIM paper:

```bibtex
@article{koide2024glim,
  title={GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author={Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal={IEEE Robotics and Automation Letters},
  year={2024}
}
```
