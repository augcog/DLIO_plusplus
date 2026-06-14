# Map-building + localization pipeline (AV-24, Putnam-style bags)

End-to-end recipe: raw recorded rosbag → GLIM map → GICP localization at IMU
rate, with trajectory-error checks at each stage. Written for a local ROS 2
Jazzy workstation. Every command below runs **from the repo root** (the GLIM
configs reference `av24.urdf` relative to the working directory).

```bash
# From anywhere inside this checkout:
cd "$(git rev-parse --show-toplevel)"
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
```

Paths below use these placeholders — substitute your own:

| Placeholder | Example |
|---|---|
| `$RAW_BAG` | `../rosbags/putnam/may_26/run_5/filtered/all` |
| `$DATA` | `./dlio_data` |
| `$RUN` | `run_5` |

If the raw bags live on another disk, create one local relative alias once,
for example `ln -s "/path/to/AI racing Tech/rosbags" ../rosbags`, then keep
the pipeline commands relative to the checkout.

The raw bag must contain `/atlas/imu_calibrated`, `/atlas/pose_filtered`, and
`/luminar_front|left|right/points`.

Common local runs:

```bash
# Build run_5's map and localize run_5 against it.
scripts/run_dlio_pipeline.sh \
  --raw "../rosbags/putnam/may_26/run_5/filtered/all" \
  --data-root "./dlio_data" \
  --run "run_5"

# Prep run_3 with run_5's UTM origin, then localize it against run_5's map.
scripts/run_dlio_pipeline.sh \
  --raw "../rosbags/putnam/may_26/run_3/filtered/all" \
  --data-root "./dlio_data" \
  --run "run_3" \
  --origin-run "run_5" \
  --map-run "run_5" \
  --rviz true
```

## 0. One-time setup

```bash
# system deps: libpcap-dev is a hard BUILD dependency of glim_ros (pcap reader);
# python3-pip is for the bag-prep / eval scripts.
make install-deps

# Build a local CUDA-enabled gtsam_points into .deps/ and rebuild the workspace
# with the right CMAKE_PREFIX_PATH/RPATH.
make install-gtsam-points-cuda
make build
source install/setup.bash
```

## 1. Prep the bag (`/atlas/*` → `/gps_p1/*`)

None of the pipeline configs subscribe to the raw `/atlas/*` topics, and both
Atlas streams are arrival-stamped in bursts (which wrecks IMU preintegration
and time-misassociates GNSS positions). `scripts/prep_bag.py` fixes all of
it: the IMU is re-stamped onto its true uniform sampling grid (backward-min
de-jitter), the pose stream is re-stamped exactly from the FusionEngine
`p1_time` time-of-validity, the LLA pose becomes `nav_msgs/Odometry` in a
fixed local-UTM frame, an RTK-FIXED-only copy is gated out for GLIM's GNSS
factors, and the three Luminar topics pass through untouched.

```bash
python3 -u scripts/prep_bag.py \
    --input  "$RAW_BAG" \
    --output "$DATA/${RUN}_prepped"
```

Output topics: `/luminar_*/points`, `/gps_p1/imu` (frame `gps_antenna_top`),
`/gps_p1/filtered_odom` (frame `utm`), `/gps_p1/filtered_odom_rtk_fixed`.

Sanity checks printed by the script: the de-jitter period should be ~10 ms,
the P1→ROS clock-offset envelope drift should be a few ms, and the RTK-fixed
count should be a large fraction of the odom count. If it warns that 0
samples passed the RTK gate, the map will have no global anchoring — don't
proceed without understanding why.

The "utm" frame is real UTM minus a fixed local origin (first fix rounded
down to a 10 km grid; written to `<output>/utm_origin.txt`) so coordinates
stay float32-safe. **When prepping another session at the same track, pass
that session's origin explicitly** so all bags share one world frame:

```bash
python3 -u scripts/prep_bag.py --input ... --output ... \
    --utm-origin "$(tail -1 $DATA/${RUN}_prepped/utm_origin.txt)"
```

`--utm-zone N` similarly pins the UTM zone (auto = zone of the first fix).

## 2. Build the map with GLIM

```bash
ros2 run glim_ros glim_rosbag "$DATA/${RUN}_prepped" \
    --ros-args -p dump_path:="$DATA/${RUN}_dump" -p auto_quit:=true
```

Configuration notes (already set in tree — listed so you know what you're
running and what to change for other conditions):

- **Odometry = INS-driven** (`config.json` → `config_odometry_ins.json`):
  the trajectory comes from interpolating `/gps_p1/filtered_odom_rtk_fixed`
  and LiDAR just paints the map. Chosen because pure LiDAR+IMU registration
  measurably drifts (~12 %/distance) and eventually bias-runs-away on this
  feature-poor track at racing speed. **It pauses where the RTK stream
  pauses** — for bags with significant RTK loss, switch back to
  `config_odometry_cpu.json` and expect to hand-QA the result.
- **Deskewing is ON** (`config_sensors.json` → `global_shutter_lidar:
  false`). The Luminar per-point timestamps are valid; never set this back
  to true for moving platforms.
- **`lidar_concat` is OFF for mapping** (`config_sensors.json`): the three
  Luminars' clock-to-ROS mapping constants differ by up to ~100 ms, which
  the concat time-rebasing doesn't model — with deskewing active that would
  smear the aux points at speed. The front lidar alone sweeps the whole
  track over a lap. (Localization has its own concat setting and doesn't use
  point times, so it stays on there.)
- GNSS anchoring: every submap gets an RTK position prior
  (`glim_ext/config/config_gnss_global.json`, `prior_inf_scale` 1e6/1e6/1e5).
- To watch the map grow, re-add `"libstandard_viewer.so"` to
  `extension_modules` in `GLIM/glim/config/config_ros.json` (removed for
  headless runs).
- Useful extra params: `-p start_offset:=SECONDS`, `-p playback_duration:=SECONDS`
  for mapping only a slice of the bag.

Runs much faster than real time (~12x on this machine with INS odometry).
The dump directory contains `traj_imu.txt` / `traj_lidar.txt` (TUM-format
trajectories), the submaps, and `T_world_utm.txt` (SE(3) from the local-UTM
frame to the map frame — written only if GNSS factors initialized).

### 2a. Check the mapping trajectory against RTK GNSS

```bash
python3 scripts/eval_traj_vs_gnss.py \
    --dump "$DATA/${RUN}_dump" \
    --bag  "$DATA/${RUN}_prepped"
```

This interpolates the RTK-gated track (`/gps_p1/filtered_odom_rtk_fixed` by
default; samples inside RTK dropout gaps are excluded) to every trajectory
stamp and prints RMS / median / p95 / max position error. Reference result
on run_5 (782 s, full session): **horizontal RMS 0.46 m, median 0.13 m,
z RMS 0.17 m**. If you see tens of meters, the map is bad — do not feed it
to localization.

## 3. Export the PCD map

Scripted (no GUI):

```bash
ros2 run glim_ros glim_dump_to_pcd "$DATA/${RUN}_dump" "$DATA/${RUN}_map.pcd"
```

Or the QA route (recommended before freezing a map for the car): open the
dump in `ros2 run glim_ros offline_viewer`, inspect for ghosting/drift,
optionally re-optimize and manually close loops, then export PLY and convert
with `gicp_localization/scripts/convert_ply_to_pcd.py`.

### 3b. Render presentation-quality views of the map

`scripts/render_map.py` produces headless 3D renders of the PCD (no GUI/GPU
needed): z-buffer point splatting, height×intensity coloring, eye-dome
lighting for depth, and an optional trajectory ribbon overlay.

```bash
python3 scripts/render_map.py \
    --map  "$DATA/${RUN}_map.pcd" \
    --out  "$DATA/renders" \
    --traj "$DATA/${RUN}_dump/traj_imu.txt" \
    --views overview,low,top,start
```

Views auto-frame from the map's bounding box (`overview` oblique aerial,
`low` near-horizon, `top` plan view, `start` close-up at the trajectory
start). `--width/--height` set resolution (default 1920×1080); `--cmap` any
matplotlib colormap.

For an **interactive** view (mouse orbit / zoom / pan), either:

- **Browser viewer** — `scripts/export_map_html.py` packages the map into a
  shareable folder (Three.js; downsampled to stay responsive):

  ```bash
  python3 scripts/export_map_html.py --map "$DATA/${RUN}_map.pcd" \
      --out "$DATA/map_web" --traj "$DATA/${RUN}_dump/traj_imu.txt"
  python3 -m http.server 8000 -d "$DATA/map_web"   # then open http://localhost:8000
  ```

  Left-drag rotates, wheel zooms, right-drag pans; HUD has a point-size
  slider and a top-view button. The folder works on any machine (needs
  internet for the Three.js CDN and a local `http.server` — browsers block
  `fetch()` from `file://`).

- **offline_viewer** (full resolution + re-optimization/loop-closure tools):
  `ros2 run glim_ros offline_viewer "$DATA/${RUN}_dump"`.

## 4. Localize against the map (bag replay)

One-command version (starts everything, waits for the map to load, replays
in real time, tears down, prints the error stats):

```bash
scripts/run_localization_replay.sh \
    "$DATA/${RUN}_prepped" \
    "$DATA/${RUN}_map.pcd" \
    "$DATA/${RUN}_dump/T_world_utm.txt" \
    "$DATA/${RUN}_loc"
```

(Append `true` as a 5th argument to open RViz.) The replay script also starts
`scripts/live_gnss_error_monitor.py`: RViz shows the GICP trajectory in green,
the GNSS/RTK reference trajectory in red, and a live marker from the GICP pose
to the nearest GNSS pose. It also publishes a live 3D "error rollercoaster":
the GNSS path stays on the physical map, while colored vertical cylinders rise by
the current GICP-vs-GNSS error in meters. It writes live error samples to
`$DATA/${RUN}_loc/live_error.csv` and refreshes
`$DATA/${RUN}_loc/live_error.png` during replay.

Do **not** rerun localization just to inspect the same result again. Once
`live_error.csv` exists, use the cached RViz path instead:

```bash
scripts/show_cached_error_viz.sh \
    "$DATA/${RUN}_loc" \
    "$DATA/${MAP_RUN:-$RUN}_map.pcd"

# Same idea through Make:
make viz-cache RUN=run_3 MAP_RUN=run_5 LOC="$DATA/run_3_loc_gnss_live"
```

This republishes the saved red GNSS path, green GICP path, current error
marker, sampled map backdrop, and live 3D error rollercoaster from the CSV.
It does not start `gicp_localization`, does not play a rosbag, and does not
rebuild or reload the GICP map.

Manual version — four processes, four terminals (all local shells, from the
repo root, with `/opt/ros/jazzy/setup.bash` and `install/setup.bash` sourced):

```bash
# T1 — localization node + robot_state_publisher (+ rviz:=true if you want)
ros2 launch gicp_localization localization_with_tf.launch.py \
    pointcloud_topic:=/luminar_front/points \
    imu_topic:=/gps_p1/imu \
    gt_odom_topic:=/gps_p1/filtered_odom_map \
    map_path:="$DATA/${RUN}_map.pcd" \
    utm_transform_path:="$DATA/${RUN}_dump/T_world_utm.txt"

# T2 — GT bridge: republishes the UTM GNSS odometry in the map frame
python3 gicp_localization/scripts/utm_to_map_odom.py --ros-args \
    -p utm_transform_path:="$DATA/${RUN}_dump/T_world_utm.txt"

# T3 — live GNSS-vs-GICP overlay/error plot for RViz and PNG output
python3 scripts/live_gnss_error_monitor.py --ros-args \
    -p est_topic:=/gicp/localization/odom \
    -p gnss_topic:=/gps_p1/filtered_odom_map \
    -p csv_path:="$DATA/${RUN}_loc/live_error.csv" \
    -p plot_path:="$DATA/${RUN}_loc/live_error.png"

# T4 — record the outputs for evaluation (optional but recommended)
ros2 bag record -o "$DATA/${RUN}_loc_eval" \
    /gicp/localization/odom /gps_p1/filtered_odom_map \
    /gps_p1/filtered_odom_map/path \
    /gicp/localization/debug/gnss_error_m \
    /gicp/localization/debug/gnss_error_markers \
    /gicp/localization/debug/gnss_error_rollercoaster

# T5 — replay (last; --clock because the node runs with use_sim_time)
ros2 bag play "$DATA/${RUN}_prepped" --clock 100
```

The localizer bootstraps its initial pose from the first
`/gps_p1/filtered_odom_map` message, calibrates IMU bias against RTK while
moving, publishes `gicp/localization/pose` per scan and
`gicp/localization/odom` at IMU rate (~99 Hz), plus `*_utm` mirrors of both.

To localize a *different* session against this map, prep that session's bag
(step 1) and reuse the same `map_path` / `utm_transform_path` — the shared
UTM frame makes the GT bridge and bootstrap line up automatically.

Concrete validation example: use `run_3` as the test run against the map
built from `run_5`:

```bash
python3 -u scripts/prep_bag.py \
  --input "../rosbags/putnam/may_26/run_3/filtered/all" \
  --output "$DATA/run_3_prepped" \
  --utm-origin "$(tail -n 1 "$DATA/run_5_prepped/utm_origin.txt")"

scripts/run_localization_replay.sh \
  "$DATA/run_3_prepped" \
  "$DATA/run_5_map.pcd" \
  "$DATA/run_5_dump/T_world_utm.txt" \
  "$DATA/run_3_loc" \
  true
```

### 4a. Check localization error

```bash
python3 scripts/eval_odom_vs_gt.py \
    --bag "$DATA/${RUN}_loc_eval" \
    --est-topic /gicp/localization/odom \
    --gt-topic  /gps_p1/filtered_odom_map
```

Prints the publish rate of the localized odom (should be ~99 Hz) and position
error stats vs the RTK INS solution.

During replay, `live_error.csv`/`live_error.png` in the localization output
directory provide the same GICP-vs-GNSS comparison incrementally, so RViz can
display the live error while the final evaluation bag is still being recorded.

Reference result (run_5 localized against its own map, full 791 s replay):
~99 Hz output, horizontal error median 0.57 m / rms 2.4 m; 66 % of samples
under 1 m. The tail comes from brief GICP-rejection bursts at feature-poor
corners (dead-reckoning until the GT snap recovers) — the tuning knobs for
those are the `gicp/hessian*` gates and `localization/gt_recovery/*` in
[gicp_localization/cfg/localization.yaml](gicp_localization/cfg/localization.yaml).

Two latency rules worth knowing before re-tuning:
- Per-scan GICP time must stay under the 50 ms scan period, or the published
  pose lags ground truth (every 100 ms of processing latency ≈ 1.5 m at race
  speed). Record `gicp/localization/debug/gicp_elapsed_ms` to check; the
  shipped `gicp/maxIterations: 32` + scan voxel 0.75 keep it at ~5–60 ms on
  this machine against the full-track map.
- The first map load builds GICP covariances for the whole map and takes
  minutes (~5–10 for the 26M-point Putnam map at `voxel_leaf_size: 0.4`);
  the replay script waits for it via ROS-graph readiness.

## Troubleshooting

- **`Failed to parse URDF: av24.urdf`** — you didn't run from the repo root.
- **GLIM prints `unsupported time type` / huge per-point warnings** — the bag
  wasn't prepped, or a non-Luminar lidar is present.
- **Localizer logs `RTK gate: dropping gt_odom`** — Atlas covariance degraded;
  the node free-runs on IMU+GICP until quality returns (expected behavior).
- **Localizer keeps warning `No IMU received on /gps_p1/imu (0 publishers)`
  during replay** — you played a raw bag (it only has `/atlas/*` topics);
  play the prepped bag. (Remapping `imu_topic` at a raw Atlas topic instead
  trips the startup allowlist hard-guard and the node exits — the prepped
  bag is the supported path either way.)
- **No `T_world_utm.txt` in the dump** — no RTK-fixed GNSS factor ever
  initialized (see the prep-script RTK warning). The map is usable but only
  in its local frame: localization init/GT features and the UTM mirrors won't
  work.
- **`ros2 pkg prefix glim_ros` points outside this repo** — re-source
  `install/setup.bash` after `/opt/ros/jazzy/setup.bash`.
