# DLIO++

ROS 2 perception stack for the AV-24 Cybertruck autonomous race car. Pairs a GPU-accelerated LiDAR-inertial SLAM front end with a map-based localizer so the vehicle can build a map offline, then localize against it online at IMU rate.

## Packages

| Package | Upstream | Purpose in this fork |
|---|---|---|
| [`adapter/`](adapter/) | new in this repo | Point One Atlas normalization boundary. Converts raw Atlas WGS84 pose/IMU into the `/gps_p1/*` (and optional `/gnss*`) streams in a **local ENU** `map` frame consumed by GLIM/GICP. |
| [`GLIM/`](GLIM/) | [`koide3/GLIM`](https://github.com/koide3/glim) (+ `glim_ext`, `glim_ros2`) | LiDAR-inertial SLAM. Builds a 3D map from IMU + multi-LiDAR + GNSS. |
| [`gicp_localization/`](gicp_localization/) | Vendored from the `vectr-ucla` DLIO line (uses `nano_gicp`) | Current production/default GICP scan-to-map localizer against a PCD map produced by GLIM. |
| [`GICP_plusplus/`](GICP_plusplus/) | new in this repo (vendored `small_gicp`) | A/B localizer with the same ENU input contract and an asynchronous Luminar front worker/aux synchronizer. Run it instead of—not alongside—`gicp_localization`. |
| [`dlio/`](dlio/) | new in this repo | Convenience metapackage that pulls the packages into a single colcon build. |

`scripts/prep_bag.py` ties the adapter to offline mapping: by default it normalizes a raw bag/PCAP, copies raw Luminar topics through untouched, **and runs GLIM** into a dump directory. Pass `--skip-glim` only when you intend to run GLIM manually afterward.

Each subpackage has its own README (`adapter/README.md`, `GLIM/README.md`, `gicp_localization/README.md`, `GICP_plusplus/README.md`) covering installation, configuration, and per-knob tuning. **This top-level README focuses on the end-to-end contract and what changed versus upstream.**

### Pipeline at a glance

```
adapter        Atlas WGS84 pose/IMU  ──►  /gps_p1/*  (+ /gnss*)      [local ENU, map frame]
prep_bag.py    raw bag/PCAP          ──►  normalized bag + GLIM dump [default, offline]
GLIM           normalized bag        ──►  offline 3D map (PCD)      [local ENU after export]
localizer      ENU PCD + /gps_p1/*   ──►  online pose @ IMU rate     [gicp_localization or GICP++]
```

## Sensor / Vehicle Target

The configs target an AV-24 Cybertruck instrumented with:

- **3× Luminar Iris LiDAR** — `luminar_front` is the primary sensor; `luminar_left` and `luminar_right` are merged into the primary cloud by `lidar_concat`. Aux extrinsics are resolved **offline** (no live `/tf_static` needed): priority **URDF** (`av24.urdf`) → **static 4×4 matrix** in config → live TF as last resort. GICP resolves the `base_frame ← luminar_front` lever arm the same way, so full localization also needs no `/tf_static`. A **strict merge guard** with identical semantics + defaults in GLIM and GICP governs incomplete merges — see [Multi-LiDAR merge policy](#multi-lidar-merge-policy) below.
- **Point One Atlas (LG69T) INS** publishing IMU on `/gps_p1/imu` (`imu_calibrated`: sensor-level bias/scale/misalignment removed by FusionEngine firmware, gravity present, no fused orientation) and odometry on `/gps_p1/filtered_odom`. Per the FusionEngine Message Spec v0.21 §3.4.1, `IMUOutput` is bias/scale-corrected and **rotated into vehicle body axes but not lever-arm-projected** — the accelerometer stays at the physical device link `pointonenav`; only the INS **pose/position** output is referenced to the primary antenna phase centre (`gps_antenna_top`). The localization config sets both `base_frame` and `imu_frame` to `gps_antenna_top`: exact for the pose, and a deliberate approximation for the IMU that drops the small (~0.8 m) device→antenna accelerometer lever arm (`ω×(ω×r)`, negligible at mapping speeds; gyro unaffected). See [`localization.yaml`](gicp_localization/cfg/localization.yaml) and the [adapter README](adapter/README.md#imu-frame). RTK quality is gated on the Atlas-reported pose covariance.
- **RTK GPS** — the FusionEngine INS itself; no separate raw RTK topic is needed for localization.
- Optional camera (used only by extension modules).

All sensor extrinsics are resolved at startup from [`av24.urdf`](av24.urdf) (offline-safe, with a static-matrix fallback so no live TF is required); the `*_frame` strings in the configs are URDF link names, not free-form labels.

### Coordinate frames — local ENU

The `map` frame is a **local ENU** tangent frame anchored at the current
dataset's fixed geodetic **datum**, matching race_common's convention. The
datum must come from that dataset's map metadata (for example, Putnam reads its
own origin from `race_metadata`'s TTL); an origin from another venue must never
be reused. The [`adapter`](adapter/) package is the **single authority** that
converts raw Atlas WGS84 fixes into that ENU frame and republishes `/gps_p1/*`
(and optional `/gnss*`) already in ENU, so GLIM and GICP consume ENU directly.
GICP is **frame-agnostic** — it localizes the scan against the PCD map and
reports the pose in whatever frame the map is in; because the map is built in
ENU and the seed is ENU, its output is ENU with no extra transform.

The one hard constraint is a **single shared datum**: the map, the seed (`/gps_p1/filtered_odom`), and GICP must all use the origin the adapter defines, or the frames silently disagree.

This **replaces the earlier UTM contract**. GLIM's `gnss_global` still aligns the map to the GNSS input frame via a 2D Umeyama fit and can export that SE(3) (the file/variable are still named `T_world_utm` for historical reasons), but when fed ENU input that transform is effectively world↔ENU. GICP's `utm`-frame publishing is now an **optional legacy layer**, active only if `localization/utm_transform_path` is set.

### Multi-LiDAR merge policy

`lidar_concat` (in both GLIM and GICP) merges `luminar_left`/`luminar_right` into `luminar_front`. Two config flags — **identical names, semantics, and defaults in both pipelines** — govern what happens when an aux scan is missing or late:

| Flag | Default | Meaning |
|---|---|---|
| `require_all_aux` | `false` | `false` = localize/map on whatever LiDARs merged this scan (front + available aux). `true` = an incomplete merge **skips** the scan (a degraded cloud is never registered; IMU propagation continues). |
| `abort_on_merge_failure` | `true` | Only relevant when `require_all_aux=true`. Past `max_consecutive_(aux_)merge_failures` (default `10`): `true` = abort the node (fail-fast, for sync validation / bring-up); `false` = keep skipping non-fatally (robust long replays). |
| `time_threshold` | `0.1 s` | Header-time matching window. In **GLIM** it is the non-Luminar fallback/tie-break only — the authoritative Luminar gate there is the decoded per-point endpoint error (`luminar_time_threshold`, 0.01 s), and a Luminar scan with no decodable absolute point time does not fall back to header matching. In **GICP_plusplus** the equivalent gate is `luminar_point_time_threshold_s`. Legacy `gicp_localization` still matches every aux by header time with this window. Do **not** raise this setting to cure front gaps. |

The operational default (`require_all_aux=false`) localizes on available LiDARs; strict mode (`require_all_aux=true`) is for a sync-validation pass. Merged-sweep deskew timing anchors on the **primary** scan's earliest timestamp, not the global merged minimum. Config lives in `gicp_localization/cfg/localization.yaml` and `GLIM/glim/config/config_sensors.json`.

Both stacks now record **per-frame merge evidence** (P4): GICP publishes `merged_aux_count`, per-aux signed `aux<i>_merge_dt_s`, `aux<i>_points`, and `scan_time_span_s` debug topics (plus the same fields in its `SCAN DEBUG` log line); GLIM's offline mapping tools emit one parseable `CONCAT DEBUG | ...` INFO line per primary scan (`lidar_concat.frame_diag_log`, default on). Both accumulate per-aux signed header-offset stats (periodic INFO summary, acquisition-phase observability only — a stable nonzero mean is expected on PTP-synced Iris units and must not be copied into the point-clock offsets; only legacy `gicp_localization` still flags a >20 ms mean). The GICP concat ring buffer is 200 (GLIM parity; 20 was only ~2 s of aux history and silently degraded frames to fewer LiDARs).

Current localization scope is intentionally single-source Point One Atlas. Earlier
project notes mention NovAtel and VectorNav GNSS integration, but
`gicp_localization` no longer subscribes to either; adding them back is future
work and needs a fresh source-selection and fix-status design rather than a
topic remap.

### GNSS lever-arm policy

The Atlas INS **pose/position** solution is already output at the antenna phase centre (`gps_antenna_top`), and the mapping graph body frame is that same point, so the software GNSS **position** lever-arm stays **off** to avoid double-compensation. (This is a pose-frame argument, independent of the IMU stream — which is device-located, per the Sensor/Vehicle Target note above.) The disable is explicit in two independent places — either one is sufficient:

1. **Config flag** — `GLIM/glim_ext/config/config_gnss_global.json` sets `"enable_lever_arm": false`. This is the grep-able single source of truth.
2. **Empty antenna frame** — same file sets `"urdf_gnss_frame": ""`. With this empty, the URDF lookup is skipped and `t_imu_gnss` stays zero even if the flag check were bypassed.

Note the module itself **is loaded** (`libgnss_global.so` is in `config_ros.json`'s `extension_modules`) — it provides the RTK position anchoring and the dual-antenna heading priors for mapping; only its software lever-arm is disabled.

To verify the disable in one command:

```bash
grep enable_lever_arm GLIM/glim_ext/config/config_gnss_global.json
# expected: "enable_lever_arm": false,
```

If the module ever loads with this config, it logs `lever-arm compensation explicitly disabled via gnss.enable_lever_arm=false; t_imu_gnss=0` at startup. If the GNSS extension is ever turned back on, verify the receiver's `LEVERARMCONFIG` state first and flip the flag accordingly — only one side should be doing the correction.

### Recovery during GICP failures

In low-feature stretches the localizer first falls back to IMU dead-reckoning. If GICP keeps rejecting (after `gt_recovery/min_consecutive_failures` non-accepts), the node snaps pose and velocity to the time-matched Atlas INS sample.

**The RTK quality gate is applied per-consumer, not globally, and snap recovery is intentionally exempt.** `callbackGtOdom()` buffers *every* Atlas sample regardless of FIXED/FLOAT/dead-reckoning state; the covariance gate (`gtSampleIsRtkFixed`) is then applied at each consumer:

- **Bias calibration / seed** (`tryRtkCalibrationStep`) → **requires RTK-FIXED**.
- **GT divergence cross-check / candidate sanity envelope** (`gt_pos_err`,
  deployed `max_candidate_position_error_m: 5.0`) → **requires RTK-FIXED**.
  Atlas position is not blended into healthy GICP poses inside the envelope;
  the gate only rejects a repeated-geometry wrong basin before observer update.
- **Snap recovery** (`maybeSnapPoseToGT`) → **accepts any-quality Atlas sample**.

The rationale is that Atlas FusionEngine already runs a coupled GNSS+IMU INS with calibrated sensors, so during RTK loss its degraded pose is still the better truth source than the node's own software IMU dead-reckoning. This means recovery can snap toward an RTK-float/GPS-only fix when GICP has failed — a deliberate trade. It is enabled by default (`gt_recovery/enable: true`, `min_consecutive_failures: 5` — raised from 1 in the P2 turn-error fixes: per-frame snapping masked dead-reckoning quality in replay metrics); raise `min_consecutive_failures` further, or disable `gt_recovery` if you require the snap to be strictly RTK-gated. The joint low-feature-LiDAR and degraded-RTK case remains an operational watch condition.

### Initialization: RTK-driven IMU calibration

By default the localizer uses RTK-FIXED Atlas GT odom to calibrate gyro/accel biases while the vehicle is moving, and seeds pose+velocity from the first high-quality sample rather than assuming the vehicle is stationary. Falls back to the legacy stationary calibration if no RTK-FIXED GT odom is received within a configurable timeout. With `localization/rtk_gate/enable=true`, the calibration/seed and the divergence cross-check inspect `pose.covariance` on each `/gps_p1/filtered_odom` sample and use only those within `max_pose_var_xy` / `max_pose_var_z` (the buffer itself keeps every sample; the gate is per-consumer, and snap recovery is exempt — see above). Knobs live under `localization/rtk_init/*` and `localization/rtk_gate/*` in the localization yaml.

### GLIM mapping init — stationary IMU initialization, then a short RTK-FIXED drive

> The conditions are **sequenced, not conflicting**. Point One Atlas's
> dual-antenna LG69T can acquire RTK FIXED + INS attitude at standstill, so the
> parked phase is sufficient for IMU initialization and for confirming RTK
> quality. GLIM's world↔ENU fit then completes during the first gentle ≥5 m of
> travel; it cannot complete while parked.

**🅐 PHASE 1 — Stationary calibration of the GLIM odometry estimator.**

> **PARK THE VEHICLE LEVEL. KEEP IT STATIONARY FOR AT LEAST 5 SECONDS AFTER LAUNCHING GLIM.**

GLIM's `LOOSE` init (`config_odometry_gpu.json`: `initialization_mode=LOOSE`, `initialization_window_size=5.0`) collects 5 s of IMU + LiDAR, then runs a batch optimization that estimates the **gravity direction** by averaging the normalized IMU specific-force vector across the window. The math assumes mean acceleration ≈ gravity, which is exact at standstill. The result is locked: `fix_imu_bias: true` freezes the IMU bias at whatever the init optimizer landed on. Aggressive accel during this phase tilts the gravity estimate and rotates the map for the rest of the session — a restart is the only fix.

**🅑 PHASE 2 — RTK-FIXED world↔ENU alignment during the first ≥5 m.**

> **DO NOT BEGIN DRIVING UNTIL `rtk_fixed_odom_filter.py` HAS LOGGED `RTK transition: … -> FIXED` AT LEAST ONCE.**

The pre-filter (`gicp_localization/scripts/rtk_fixed_odom_filter.py`) gates `/gps_p1/filtered_odom` on Atlas's pose covariance and forwards only RTK-FIXED-quality samples to `libgnss_global.so`. GLIM banks these qualified samples while it builds the required trajectory and GNSS baselines; once the ≥5 m fit succeeds, it backfills position/yaw priors for the eligible early submaps. **The early map segment must therefore be recorded with FIXED-quality Atlas poses—not degraded RTK-FLOAT or GPS-only samples.** Begin driving before FIXED and the early trajectory grows without cm-level anchors until RTK later reacquires.

**Why the conditions sequence cleanly (and why they don't conflict):**

A single-antenna INS receiver that aligns its heading from motion would create a real conflict with Phase 1's stationary requirement. The dual-antenna Atlas does not: heading is observable from the antenna baseline at standstill, and RTK position fixing depends only on satellite geometry + base-station correction, also fine at standstill with clear sky. In practice the operator parks once until RTK FIXED and the 5 s LOOSE window complete, then performs the short gentle drive that completes Phase 2.

**Operator sequence:**

1. Park the vehicle level at the intended map origin, with clear sky view.
2. Power Atlas; wait for its status display to read RTK FIXED + INS aligned.
3. Launch `rtk_fixed_odom_filter.py`; verify `First INS sample received … -> FIXED` in its log.
4. Launch GLIM; wait for the 5 s LOOSE init to complete — look for `estimate initial IMU state` and the first sub-map appearing in the viewer.
5. **Now begin driving — gently — for the first ≥ 5 m.** `gnss_global` **cannot** emit a prior factor while parked: it must first fit the one-shot world↔ENU transform, which needs the trajectory baseline to exceed `min_baseline: 5.0 m` (and, as of the residual-gated fit, the GNSS-side baseline too). Waiting for a prior factor *before* driving is therefore unsatisfiable — the parked RTK-FIXED samples are buffered and **backfilled** as factors once the baseline is reached, so no early submap is left unanchored.
6. Confirm `gnss_global` logs `T_world_utm=…` followed by prior-factor insertions during this first stretch, then continue the run normally.

Steps 3–4 happen inside the parked 30 s – 2 min RTK acquisition window; the transform fit and first factors land during the first few metres of step 5. If Atlas never reaches FIXED while parked, that's a hardware/sky-view problem to resolve before driving — it should not be papered over by starting GLIM and "hoping" RTK lands later.

Neither phase applies to `gicp_localization` — that pipeline does RTK-driven IMU calibration while the vehicle is moving and **seeds** from a single RTK-FIXED GT sample (the initial seed and calibration are FIXED-gated; the failure-recovery snap is not — see [Recovery during GICP failures](#recovery-during-gicp-failures)). Only GLIM mapping needs the two-phase sequenced startup.

### 2026-07 turn-error campaign (P1–P5)

The cross-run replay campaign (run 3 ↔ run 5, July 2026) diagnosed and fixed a family of turn-localization errors. Use `gicp_localization/scripts/analyze_scan_debug_log.py` to score a replay from its `SCAN DEBUG` evidence. Headlines:

- **P1** — GICP binary accept/reject gates replaced with support/physics-aware
  gating and full-6D degeneracy partial updates. The deployed Laguna default
  follows perception-ws and disables map-density-dependent absolute and rolling
  fitness rejection; the ratio machinery remains available for explicit A/B.
- **P2** — state-continuity fixes on the rejected-scan path (stale-velocity
  bug), GT-snap twist continuity, RTK-quality candidate safety envelope, and
  observer position/velocity/speed bounds that prevent a wrong basin from
  producing an unphysical prediction; recovery threshold 1 → 5.
- **P3** — delta-form observer correction (removes the 0.1–0.3 s stale-measurement yaw lag in turns) and a unified IMU bias path (bias applied once, at buffering).
- **P4** — geometry densification: scan voxel 0.5 → 0.3 m, dense GLIM map profile (**active default**, see below), per-frame merge diagnostics in both stacks, concat buffer parity (200).
- **P5** — dual-antenna heading priors in GLIM mapping hardened with a per-sample yaw-quality gate.

### Remaining tuning work

- **Dense-map rebuild + threshold re-baseline.** The dense GLIM localization-map profile (`config_preprocess_dense_map.json` / `config_sub_mapping_dense_map.json`) is now the active default; the run3/run5 maps must be rebuilt with it, after which the GICP fitness floor and the P1 ratio thresholds (`fitnessBaseline/seedBaseline`, `yawGate/fitnessRatio`, `fitnessRatioRejectThreshold`) should be re-measured from the scorecard script's suggestions.
- **Validation replays.** Run both cross-pair replays, including one pass with `gt_recovery/enable=false`, and compare accepted/rejected streaks, GICP latency, fitness ratios, yaw error, and merge coverage from the generated `SCAN DEBUG` evidence.

### Diagnostic: silent IMU subscription failures

When the `imu_topic:=` launch arg points at a non-existent topic, the subscription is created but no callback fires and historically there was nothing in the log to explain it. The localizer now runs a periodic health check that warns (every 3 s, until the first IMU arrives) with the resolved topic name and whether 0 publishers exist — surfacing the typo case immediately. The timer self-cancels on first IMU receipt.

### Per-point timestamp formats and motion deskewing

Both GLIM and `gicp_localization` deskew each LiDAR scan to compensate for vehicle motion across the scan duration. Deskewing reads the scan's per-point timestamp field, interpolates the IMU-propagated pose for each point's capture instant, and projects every point into a single common time. At race speeds (30 m/s) this can be the difference between a 30 cm scan-end smear and a clean point.

**Deskewing is ON in both stacks** (`dlio/deskew: true` in `gicp_localization/cfg/localization.yaml`; `autoconf_perpoint_times: true` + `autoconf_prefer_frame_time: false` in `GLIM/glim/config/config_sensors.json`).

#### Luminar Iris per-point timestamps — the definitive account

This is the authoritative description; if other comments disagree, this section and the validated code win.

**On-the-wire format (Luminar Iris Data Output Specification v1.3.0).** The Iris does *not* emit a single 64-bit timestamp on the wire. It splits the PTP time across two places: **48-bit integer epoch seconds in the packet header** (§2.1, `UQ48.0`) and a **32-bit sub-second nanosecond count per ray** (§2.2 / §2.6.3, `UQ32.0`) that wraps every 1 s; all fields little-endian. The ROS2 driver reconstructs these into one **little-endian `uint64` of full epoch nanoseconds** per point and publishes it as PointCloud2 field `timestamp` (`datatype=UINT8`, `count=8`, `offset=0`, `point_step=56`). *(Note: the previously cited "Iris PIG R2.0.7 §7.8.4" is PTP Troubleshooting, not the data layout — the Data Output Spec above is the real source.)*

**Validated (May-26 `run_5`/`run_3` bags).** All three Luminar topics expose that exact schema; the bytes decode as `uint64` epoch ns (e.g. `1779827344001041615` → 2026-05-26T20:29:04Z), intra-scan span ≈ **48.997 ms**, second rollovers safe, and no collapse.

**GICP deskew — robust by construction.** `copyPointTimeFromCloud` (LUMINAR case in `localization.cc`) stores the raw `uint64` ns; `deskewPointcloud` then computes each point's capture time as

```
t_point = scan_stamp.seconds()  +  (ts - anchor_ts) * 1e-9
          └── header anchor ──┘     └── intra-sweep relative offset (signed) ──┘
```

It uses **only the relative offset within the sweep, anchored at the header stamp** — it never trusts the absolute epoch of `ts`. `anchor_ts` is the **earliest timestamp of the PRIMARY scan**, captured in `mergeAuxClouds()` before any aux cloud is appended (for a single-sensor scan this is just that scan's own minimum). Anchoring on the primary — rather than the global merged minimum — keeps a multi-LiDAR sweep correctly timed when an aux scan started *before* the primary: such aux points get correctly **negative** offsets (hence the **signed** `int64` subtraction), instead of being collapsed onto the header stamp and shifting the whole sweep late. That makes GICP deskew **correct regardless of whether the per-point clock is on the Unix/INS epoch or a sensor-local/PTP axis**. The only way it could break is a driver emitting a bare 32-bit sub-second field that wraps mid-scan; the one-second-boundary check (Procedure C) confirms that does not happen. This is why GICP needs no timestamp repair and is the more trustworthy pipeline for deskew.

**GLIM deskew — correct, but requires epoch alignment.** `ros_cloud_converter.hpp` reads `UINT8[8]` as little-endian `uint64` and divides by `1e9` → epoch *seconds* (~1.78e9). `TimeKeeper::replace_points_stamp` then sees `max_time ≥ 1.0`, takes the *absolute → relative* branch, and (with `prefer_frame_time=false`) **overwrites the frame stamp with the first point time** while making per-point times relative; `point_time_scale` stays `1.0`. Because GLIM *trusts the absolute point-time epoch*, that epoch must match the IMU/header epoch. Raw bags were observed with point times on the sensor/PTP axis (~2e13 ns) while the header/IMU were on the ROS/INS epoch — GLIM then overwrote the frame stamp with a sensor-clock value and dropped every scan as unsynchronized. The **live, in-pipeline safeguard** closes this: `ros_cloud_converter.hpp` rebases the absolute per-point times onto the header epoch (span preserved), anchored on the primary scan's earliest timestamp (`epoch_anchor_count`), and only when the times are absolute *and* `|header − min| > 1 s` — a no-op on already-aligned data and on scan-relative sensors.

Note: `scripts/prep_bag.py` deliberately does **not** rebase LiDAR — it copies the raw Luminar messages **byte-for-byte** (keeping the raw measurements unchanged) and only normalizes the small Atlas-derived streams. Timestamp handling is owned entirely by the runtime converter/TimeKeeper, so the offline path relies on the same live safeguard. GICP requires no rebase at all because of the header-anchored relative-offset design above.

**Status.** Deskew is validated correct for all bagged/offline data in both stacks. The only open item is a **live-hardware PTP-lock repeat** (no live publishers were available during the final check). That item concerns absolute-epoch / GT time association, **not** GICP deskew geometry, which depends only on the (validated) intra-scan span. Previously both stacks ran with deskew effectively off because this encoding was ambiguous; it no longer is.

**`gicp_localization` supports five sensor-type-driven decoders** (`copyPointTimeFromCloud` in `localization.cc`), selected by `localization/sensor_type` in the yaml:

| `localization/sensor_type` | Field encodings handled | Notes |
|---|---|---|
| `luminar` | `UINT8[8]` (uint64 epoch ns), `FLOAT64` (scan-relative seconds by default; raw uint64 epoch-ns bits only with the explicit driver opt-in) | Laguna's decoder publishes ordinary FLOAT64 seconds-since-sweep-start. The localizer rebases auxiliary relative times onto the primary header before deskew. `UINT8[8]` remains an absolute PTP carrier. `UINT32` is intentionally rejected on the Luminar path because it cannot carry an absolute epoch and its unit/anchor would otherwise be ambiguous. |
| `ouster` | `UINT32`, `FLOAT32`, `FLOAT64` (all scan-relative ns or s) | Standard Ouster ROS driver layouts. |
| `velodyne` | `FLOAT32`, `UINT32` (scan-relative s or ns) | VLP-16/32 and similar. |
| `hesai` | `FLOAT64`, `FLOAT32` (absolute or relative seconds) | Pandar / XT line. |
| `livox` | `UINT8[8]`, `UINT32`, `FLOAT64`, `FLOAT32` | MID/HAP/Avia. Both packed uint64 ns and scaled-double conventions. |

That's **5 sensor types × multiple PointField datatypes** per family. Adding a new vendor means extending the `case dlio::SensorType::*` switch with the right `memcpy` and unit conversion — about 10 lines.

**GLIM supports three auto-detected timestamp buckets** (`TimeKeeper::replace_points_stamp` in `glim/src/glim/util/time_keeper.cpp`), one per encoding family:

| Bucket detected from `min/max` per-point time | Source encodings that fall here | What GLIM does |
|---|---|---|
| `max_time < 1.0` | Scan-relative FLOAT seconds (Ouster, Velodyne, Hesai, Livox in their FLOAT modes) | Use as-is. |
| `1.0 ≤ max_time < 1e16` | Absolute epoch **seconds** — Hesai FLOAT64 absolute, **and Luminar Iris** (its `UINT8[8]` ns are divided by `1e9` in `ros_cloud_converter.hpp` *before* TimeKeeper, landing here at ~1.78e9) | Overwrite frame stamp with first point time; treat per-point times as relative seconds (`point_time_scale = 1.0`). |
| `min_time ≥ 1e16` | Raw, *unconverted* 64-bit nanoseconds (e.g. Livox FLOAT64 ns forwarded without scaling) | Apply `1e-9` scale; rebase to first-point time. |

Note: Luminar lands in the **middle** bucket, not the `≥1e16` one, precisely because `ros_cloud_converter.hpp` already applies the `1e-9` divide. The `≥1e16` branch only fires for pipelines that forward raw nanoseconds — which this one never does for Iris.

The combination of `autoconf_perpoint_times: true` and `autoconf_prefer_frame_time: false` makes GLIM use the *per-point* times for deskew. The earlier "Luminar timestamps look collapsed" symptom was the `autoconf_prefer_frame_time: true` default collapsing each scan to its single header stamp — that has been turned off.

Concatenation note: both stacks leave Iris `UINT8[8]` absolute per-point times **unshifted**. Scan-relative fields are shifted by `dt = T_aux − T_primary` to rebase them onto the primary header. In GLIM, `FLOAT64` is scan-relative seconds by default; treat its raw bytes as epoch nanoseconds only with the explicit `sensors.float64_time_is_epoch_ns=true` driver contract. Luminar localizers treat their supported raw 8-byte epoch carriers as absolute rather than applying a header shift.

If you ever switch sensors and the deskew looks wrong, use the one-shot diagnostic in `gicp_localization`: it emits a `[LUMINAR_TS_DIAG] BEGIN ... END` block on the first PointCloud2 message of each session, including per-point field metadata and raw bytes interpreted four ways.

## Workflow

1. **Record** a raw bag containing LiDAR + `/atlas/*` (and a Point One PCAP for IMU) during a driving session.
2. **Normalize + map** with `prep_bag.py`. By **default it does BOTH**: it normalizes the raw bag (Atlas pose/IMU → `/gps_p1/*` in the local-ENU `map` frame, raw Luminar topics copied through unchanged) **and then runs GLIM** on the normalized bag into `--dump-dir`, writing `enu_origin.txt` (the datum single-source-of-truth the exporter reads) alongside the dump. Pick ONE route — do **not** run GLIM again by hand on the normalized bag (that double-maps, and the second dump has no `enu_origin.txt`):

   **All-in-one (recommended):**
   ```bash
   python3 scripts/prep_bag.py --input /path/to/raw_bag --output /path/to/normalized_bag \
     --p1-imu-pcap /path/to/ins.pcap --dump-dir /tmp/dump
   ```
   Produces the normalized bag **and** the GLIM dump in `/tmp/dump` (with `enu_origin.txt`). Skip to step 4.

   **Normalization-only** (when you want to run/re-run GLIM yourself):
   ```bash
   python3 scripts/prep_bag.py --input /path/to/raw_bag --output /path/to/normalized_bag \
     --p1-imu-pcap /path/to/ins.pcap --skip-glim
   ```
   Then map manually and pass the datum to the exporter explicitly (there is no `enu_origin.txt` on a hand-run dump):
   ```bash
   ros2 run glim_ros glim_rosbag <normalized_bag> --ros-args -p dump_path:=/tmp/dump
   ```
3. The dump (`/tmp/dump`) contains `graph.bin`, `traj_lidar.txt`, `odom_lidar.txt`, numbered submap point clouds, and `T_world_utm.txt`. **Frame contract (fixed 2026-07-10):** the dump lives in GLIM's internal WORLD frame, related to Atlas local ENU by the exported `T_world_utm` — the dump is NOT itself ENU unless that transform happens to be identity.
4. **QA** the dump in `glim_ros offline_viewer` (visual inspection, optional post-hoc optimization, manual loop closures — see "Why the offline_viewer step is manual" below). If you optimize, re-save the dump. Do **not** use the viewer's own point export as the localization map: it writes raw WORLD-frame points (`global_mapping.cpp` export path) without the ENU conversion.
5. **Export** the localization map — this step is the REQUIRED handoff:
   ```bash
   python3 scripts/export_glim_dump_to_pcd.py /tmp/dump /path/to/track_map.pcd --voxel-size 0.1
   ```
   The exporter defaults to `--frame enu`: it applies `inverse(T_world_utm)` so the PCD is genuinely in the Atlas local-ENU frame, fails closed when the transform is missing, and writes a `*.manifest.yaml` recording the frame and transform (check it before shipping a map). It reads the datum from `<dump>/enu_origin.txt` automatically (written by the all-in-one `prep_bag.py` route); for a **hand-run GLIM dump** that file does not exist, so pass the datum explicitly: `--enu-origin "<lat,lon,alt>"` (the same origin the adapter used).
   Do not deploy a dense union of every repeated lap. For a perception-ws-style
   deployment map, export two or more representative laps independently as XYZ
   at the final voxel size, then retain repeatable voxels inside the driven
   corridor:
   ```bash
   python3 scripts/export_glim_dump_to_pcd.py /tmp/dump /tmp/lap1.pcd \
     --submap-range START1:END1 --voxel-size 0.15 --pcd-fields xyz \
     --gnss-enu-origin "INPUT_LAT,LON,ALT" --enu-origin "OUTPUT_LAT,LON,ALT"
   python3 scripts/export_glim_dump_to_pcd.py /tmp/dump /tmp/lap2.pcd \
     --submap-range START2:END2 --voxel-size 0.15 --pcd-fields xyz \
     --gnss-enu-origin "INPUT_LAT,LON,ALT" --enu-origin "OUTPUT_LAT,LON,ALT"
   python3 scripts/export_glim_dump_to_pcd.py /tmp/dump /tmp/lap3.pcd \
     --submap-range START3:END3 --voxel-size 0.15 --pcd-fields xyz \
     --gnss-enu-origin "INPUT_LAT,LON,ALT" --enu-origin "OUTPUT_LAT,LON,ALT"
   python3 scripts/export_glim_dump_to_pcd.py /tmp/dump /tmp/staging.pcd \
     --submap-range STAGING_START:STAGING_END --submap-step 10 \
     --voxel-size 0.15 --pcd-fields xyz \
     --gnss-enu-origin "INPUT_LAT,LON,ALT" --enu-origin "OUTPUT_LAT,LON,ALT"
   python3 scripts/build_consistent_pcd.py /path/to/deploy_map.pcd \
     /tmp/lap1.pcd /tmp/lap2.pcd /tmp/lap3.pcd \
     --coverage-pcd /tmp/staging.pcd \
     --corridor-trajectory /tmp/dump/traj_lidar.txt \
     --corridor-index-range START1:END3 \
     --corridor-radius 75 --min-sessions 2 --voxel-size 0.15
   ```
   `build_consistent_pcd.py` globally deduplicates each lap, requires
   cross-lap support inside the corridor, and keeps unique distant structure
   outside it. A sparse pit/staging export may be added with
   `--coverage-pcd`; use `--submap-step` when creating that export so a long
   stationary period does not dominate map size. The output manifest embeds
   every source range, datum, transform, and filter count.
6. **Localize** online against that PCD with `gicp_localization`/`GICP_plusplus`, using the adapter's ENU `/gps_p1/*` streams as IMU + seed. Because the exported map is genuinely ENU, Atlas seeds/GT are frame-correct directly — and `localization/utm_transform_path` must stay **EMPTY** (it exists only for legacy world-frame maps and would double-transform an ENU map).

7. **Prepare a deterministic real-time replay input.** Topic filtering at
   `ros2 bag play` time still makes the player scan unrelated messages in large
   camera/multi-LiDAR bags. Build one compressed MCAP containing only the
   online-localization input contract before the audit:
   ```bash
   python3 scripts/prepare_gicp_replay_bag.py \
     --bag /path/to/DATASET_ROOT/<collection>/<run>/filtered/all \
     --bag /path/to/DATASET_ROOT/<collection>/<run>/navigation_bag \
     --out /path/to/DATASET_ROOT/prep_bag/<run>_front_atlas_gicp
   ```
   The helper refuses cross-dataset inputs and outputs, retains only the
   `/luminar_front/points`, `/gps_p1/imu`, and `/gps_p1/filtered_odom`
   streams with their full message counts, and records input/config hashes
   plus `ros2 bag info`. This step
   changes only the offline I/O envelope; live-car localization still consumes
   those three topics directly.

8. **Audit the compressed map at real time** with the repository runner. It
   derives `DATASET_ROOT` from `--map-dir`, refuses to overwrite an existing
   result, and writes the debug/reference bags, logs, resource samples,
   machine-readable run status and scan scorecard under that dataset's
   `gicp_result/intermediate/`. Promote a run to `gicp_result/` only after
   manual log, bag, status, and metric audit passes:
   ```bash
   scripts/run_gicp_replay_audit.sh \
     --map-dir /path/to/DATASET_ROOT/maps/<compressed-map> \
     --bag /path/to/DATASET_ROOT/prep_bag/<run>_front_atlas_gicp \
     --run-name <run>_compressed_full_1x \
     --overlay /path/to/gicp/install/setup.bash \
     --mode gnss_aided \
     --reference-is-gt-ack \
     --config-path GICP_plusplus/cfg/front_quality_replay.yaml \
     --start-offset 0 \
     --duration <full-overlap-seconds> \
     --rate 1.0 \
     --primary-queue-size 32
   ```
   `gnss_aided` explicitly labels that Atlas participates in localization.
   When the same Atlas odometry is also the score reference, the acknowledgement
   flag is mandatory because that evidence is not independent truth.
   `--mode independent` instead requires a `--reference-topic` distinct from
   the runtime `--gt-topic`; a YAML profile alone cannot make the same aided
   stream independent truth. The optional
   `GICP_plusplus/cfg/front_no_atlas_translation_replay.yaml` removes
   per-scan Atlas translation seeding/gating for a registration A/B, but does
   not relabel its evidence as independent. Acceptance, rejection-streak,
   debug-coverage, and zero-drop gates are explicit runner flags.
   The offline audit uses RELIABLE LiDAR publication/subscription on both
   sides so a large PointCloud2 cannot disappear in DDS without accounting.
   Its 50,000-message rosbag read-ahead queue keeps storage/decompression
   latency out of the 10 Hz delivery schedule. A bounded 32-frame offline
   compute queue absorbs rosbag delivery bursts without hiding registration
   cost: the audit must independently report GICP P95/max below 100 ms and
   zero overload drops. Live sensors keep BEST_EFFORT and the default
   8-frame queue.
   The online localization contract remains front LiDAR only; the map itself
   is built from all configured LiDARs.

   If a recorded odometry stream has documented map-axis translation relative
   to the map datum (for example an ellipsoid/geoid height convention), pass
   `scripts/offset_odom.py` through `--bridge-script` and repeat
   `--bridge-arg` for its explicit input topic, output topic, XYZ offset and
   frame. The runner never embeds a site-specific transform.

9. **Render the audited result from above.** The plotting tool reads the
   runner's two output bags and map directly, writes a full-run image plus
   complete-lap images, and records the exact input hashes and lap boundaries
   in `trajectory_manifest.json`:
   ```bash
   python3 scripts/generate_gicp_topdown.py \
     --debug-bag /path/to/DATASET_ROOT/gicp_result/<result>/debug_topics_bag \
     --reference-bag /path/to/DATASET_ROOT/gicp_result/<result>/reference_topics_bag \
     --localization-log /path/to/DATASET_ROOT/gicp_result/<result>/localization.log \
     --map /path/to/DATASET_ROOT/maps/<compressed-map>/map.pcd \
     --output-dir /path/to/DATASET_ROOT/gicp_result/<result>/topdown \
     --reference-topic /path/to/reference/topic \
     --run-label "<dataset> <run>, compressed map, full 1.0x" \
     --map-label "three-LiDAR consistent map"
   ```

### High-quality mapping profile

For a localization map, generate a run-local configuration instead of editing
the installed GLIM/GLIM-ext JSON files. This keeps topic, calibration, scratch,
and dataset paths explicit and prevents one run's GNSS transform/config from
leaking into another run:

```bash
python3 scripts/generate_glim_mapping_config.py \
  --output-dir /path/to/DATASET_ROOT/maps/<run>/config \
  --offload-dir /tmp/glim_offload_<unique-run-id> \
  --imu-topic /prepared/body_imu \
  --gnss-topic /gnss \
  --keyframes-per-submap 1 \
  --odom-rotation-stddev 0.01 \
  --odom-translation-stddev 0.05 \
  --global-update-interval 10 \
  --optimizer-extra-loop-updates 20 \
  --loop-registration-interval 10 \
  --max-loop-candidates-per-source 1 \
  --loop-max-translation-correction 0.3 \
  --loop-max-rotation-correction-deg 1.0 \
  --loop-detection-sync-timeout 30 \
  --gnss-gravity-prior-sigma-deg 0.5 \
  --t-lidar-imu X Y Z QX QY QZ QW \
  --imu-input-rotation QX QY QZ QW \
  --urdf-path /absolute/path/to/vehicle.urdf \
  --aux-lidar /left/points:left_frame \
  --aux-lidar /right/points:right_frame

ros2 run glim_ros glim_rosbag /path/to/prepared_bag --ros-args \
  -p config_path:=/path/to/DATASET_ROOT/maps/<run>/config \
  -p dump_path:=/path/to/DATASET_ROOT/maps/<run>/dump

python3 scripts/export_glim_dump_to_pcd.py \
  /path/to/DATASET_ROOT/maps/<run>/dump \
  /path/to/DATASET_ROOT/maps/<run>/map.pcd \
  --voxel-size 0.15 \
  --gnss-enu-origin "INPUT_GNSS_LAT,LON,ALT" \
  --enu-origin "OUTPUT_MAP_LAT,LON,ALT"
```

The generated profile follows the validated perception-ws recipe: CPU
LiDAR+IMU odometry (GNSS is a robust global constraint, not the per-scan pose
source), one scan per submap, explicit 5 m loop closure, global gauge damping
`1.0`, a 10 m world/GNSS initialization baseline, bounded covariance-aware GNSS
weights, dense-point disk offload, and a live anchor-divergence rejection gate.
`--keyframes-per-submap` is an explicit scale/quality control: its default
`1` preserves the successful perception-ws one-scan submap profile, while a
full-length run can use `--global-update-interval 10` to amortize iSAM2 updates
without combining ten scans into one rigid submap. Increasing
`--keyframes-per-submap` remains available as an explicit geometric
quality/scale tradeoff, but it is not the recommended long-run shortcut.
`--optimizer-extra-loop-updates 20` bounds no-new-factor,
forced-relinearization after a batch that contains loop closures. Refinement
stops early as soon as iSAM2 reports that no graph variable relinearized. This
lets the nonlinear loop correction settle before the GNSS health callback
judges the trajectory; it does not add or weaken any factor. Laguna Run1
showed why a fixed one-pass policy is insufficient: the live rolling anchor
median still reached `0.90 m`, but the next optimizer update settled the same
factor graph to `0.20 m`. In a 120 s convergence A/B, all 49 loop batches
converged in an average of 3.7 passes, so `20` is only a safety ceiling. The
base JSON keeps the legacy single-pass behavior with `0`.
For a long per-scan run, `--loop-registration-interval 10` independently keeps
one loop-registration source/target and KdTree per ten scans, matching the
validated perception-ws loop-node cadence without removing any pose-graph
nodes, GNSS/gravity factors, trajectory samples, or dense export points. The
generated `quality_profile.json` records all three independent cadence values.
The high-quality profile also separates the between-submap odometry covariance
into `--odom-rotation-stddev 0.01` rad and
`--odom-translation-stddev 0.05` m. The inherited isotropic `0.001` assigned
millimetre and 0.057-degree confidence to every raw LIO step. Over a long
multi-lap graph that made the chain about 76 times stiffer in roll/pitch than a
0.5-degree gravity factor and prevented the per-pose fused-GNSS factors from
correcting loop-induced deformation. The shared JSON retains `0.001/0.001` for
compatibility; only a generated high-quality profile injects the honest
anisotropic values.
`--max-loop-candidates-per-source 1` then deterministically selects the closest
eligible historical pose for GICP validation. This prevents a multi-lap bag
from adding an increasing number of equivalent constraints for each new
source, and removes the timing-dependent random candidate subset documented by
the perception-ws Laguna experiments. Set it to `0` only to reproduce legacy
unlimited proposals.
The correction gates then reject a high-overlap GICP result if it moves more
than `0.3 m` or `1 deg` from the pose-graph initial relative pose. The earlier
perception-ws telemetry warning thresholds of `1 m/5 deg` were too loose as
admission limits for the 25,096-frame Laguna Run1: 1,365 accepted factors could
still pull the latest 100 GNSS anchors to a `0.692 m` median at frame 19,567.
The stricter limits admit only closures consistent with the already
centimetre-anchored trajectory. This closes the repeated-structure failure mode
that an inlier-fraction-only test cannot detect; the base GLIM JSON keeps both
gates disabled for compatibility, while the generated high-quality profile
records the injected limits explicitly.
`--loop-detection-sync-timeout 30` also waits at each optimizer boundary for
the loop registrations already proposed at that boundary. This makes the
factor set inspected by each update deterministic and prevents later-arriving
loop factors from changing the interpretation of an earlier health sample.
Run1 testing showed that synchronization and converged relinearization remove
timing and partial-solve ambiguity, but do not make a geometrically bad loop
valid; the odometry covariance and admission limits above are still required.
The base JSON uses `0` to preserve legacy asynchronous behavior.
`--gnss-gravity-prior-sigma-deg` is independently opt-in. It uses a validated
GNSS/INS quaternion to constrain only the body-Z direction (roll/pitch), never
yaw; leave it at `0` for position-only GNSS publishers or publishers that use
an identity quaternion to mean "orientation unavailable".
The baseline can be injected with `--gnss-min-baseline`; its default matches the
successful perception-ws Laguna configuration. The high-quality profile fits
the newest segment that still spans that baseline on both trajectories while
retaining at least `--gnss-fit-min-samples 20`. It excludes the newest
`--gnss-fit-validation-samples 10` from the fit and must predict that suffix
within `--gnss-fit-max-rms 0.25 m`, in addition to passing the same in-sample
RMS gate. This prevents a two-point/recent-window fit and catches growing
estimate-side heading drift that a rigid in-sample alignment can absorb. Use
`--no-gnss-recent-fit-window` for an all-history training prefix; held-out
validation and the sample minimum still apply.
`--offload-dir` must be an absolute, empty, per-run directory; GLIM refuses
stale contents.

The anchor gate evaluates the updated global-map poses after iSAM2 has applied
the GNSS factors. It deliberately does not compare GNSS with the raw LIO pose
captured before optimization, because that would reject the normal correction
that a global constraint is supposed to make.
It requires five consecutive optimized global updates above the same `0.5 m`
median threshold before rejecting a run. A value back inside the threshold
clears the streak; this filters optimizer convergence transients without
weakening the final geometry limit.

`--imu-input-rotation` is identity by default. Use it only when the prepared
IMU topic's vectors still contain a measured receiver-mount tilt. It rotates
incoming acceleration and gyro into the calibrated frame used by
`--t-lidar-imu`, so the IMU correction is explicit instead of being hidden in
an unrelated LiDAR lever arm.

The exporter distinguishes the GNSS input datum from the delivered map datum.
When they differ, it computes the exact WGS84 ENU-to-ENU transform and records
both origins and the matrix in the manifest. This is not an ICP alignment and
does not hide map deformation. Omit `--gnss-enu-origin` only when input GNSS
and output map intentionally use the same datum.

### Why the offline_viewer step is manual

A reviewer reasonably asks: why not auto-merge the per-submap directories into a single PCD with a script? Because the viewer pass is the QA stage for the mapping output, and skipping it would silently push bad maps into the localizer:

- **Visual inspection** of the assembled map before it's frozen as the localization reference catches drift, ghosting, and bad submaps that would otherwise propagate into GICP at runtime.
- **Post-hoc global optimization** — the viewer prompts "Do optimization?" on load (see `offline_viewer.cpp:191`) and re-runs the iSAM2 backend over the full graph, which can improve the dump beyond what the online pass produced.
- **Manual loop closure** — `manual_loop_close_modal` lets the operator add constraints when the automatic detector misses a loop (common on long highway runs with weak geometry).

A blind merge would skip all three and bake unresolved drift into the PCD. The division of labor (2026-07-10): the **viewer is the QA/optimization stage** operating on the dump; `scripts/export_glim_dump_to_pcd.py` is the **only sanctioned PCD handoff**, because it is the step that applies the world→ENU conversion (`inverse(T_world_utm)`) and records the map manifest. The viewer's own export remains available for visualization but is world-frame and must not be fed to the localizer.

## Build

ROS 2 Jazzy + colcon on Ubuntu 24.04.

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Headline dependencies (per-package READMEs go deeper):

- GTSAM 4.2, gtsam_points (GPU factors), Eigen3, PCL, OpenMP, nlohmann::json, spdlog
- Optional: CUDA 11.8+ (GPU acceleration), Iridescence (viewer), OpenCV

If `ros2 pkg prefix glim` does not point inside this workspace's `install/`, an apt-installed `ros-jazzy-glim-*` package is being picked up instead of this fork — re-source `install/setup.bash` **after** `/opt/ros/jazzy/setup.bash`. The same caveat applies to `gicp_localization` if a sibling workspace is also sourced.

## Quick Reference

```bash
# Normalize a raw bag and map it with GLIM (the default all-in-one route).
python3 scripts/prep_bag.py --input <raw_bag> --output <normalized_bag> \
    --p1-imu-pcap <ins.pcap> --dump-dir <out_dir>

# Run the Atlas adapter standalone.
# Default IMU source is a Point One PCAP (use_p1_imu_pcap:=true), so pass a pcap:
ros2 launch adapter adapter.launch.py local_enu_origin:="<lat,lon,alt>" p1_imu_pcap_path:=/path/to/ins.pcap
# ...or use the live Atlas IMU ROS topic instead:
ros2 launch adapter adapter.launch.py local_enu_origin:="<lat,lon,alt>" use_p1_imu_pcap:=false

# GLIM builds maps OFFLINE only. There is no live `glim_ros.launch.py` — the live
# node intentionally exits when `enable_online_mapping=false` (config_ros.json).
# Use one of the offline entry points below.

# Manual offline bag → map. Use only after prep_bag.py ... --skip-glim,
# otherwise the default all-in-one route above already made the map.
ros2 run glim_ros glim_rosbag <bag_path> --ros-args -p dump_path:=<out_dir>

# Offline pcap → map (raw Luminar pcap + IMU/GNSS from a sibling mcap)
ros2 run glim_ros glim_pcap_rosbag <pcap_file> <mcap_bag> --ros-args -p dump_path:=<out_dir>

# Inspect a saved map
ros2 run glim_ros offline_viewer

# GICP localization against a pre-built PCD map
# (single-source P1 design: IMU + GT odom both from Atlas, at gps_antenna_top)
# map_path is REQUIRED — the checked-in yaml default points at an external,
# untracked map. Point it at the ENU PCD exported in step 5 of the workflow.
ros2 launch gicp_localization localization_with_tf.launch.py rviz:=true \
    map_path:=/path/to/track_map.pcd \
    pointcloud_topic:=/luminar_front/points \
    imu_topic:=/gps_p1/imu \
    gt_odom_topic:=/gps_p1/filtered_odom
```

> **Two localizers, an A/B pair.** `gicp_localization` (vendored DLIO / `nano_gicp`)
> is the current production/default online localizer and is what this quick
> command launches. `GICP_plusplus` is the A/B alternative (a `small_gicp`
> backend with the asynchronous Luminar front worker/aux synchronizer) used for replay comparison; launch it
> with `ros2 launch gicp_plusplus localization_with_tf.launch.py map_path:=… …`.
> They consume the same ENU map + `/gps_p1/*` streams — pick one per run; they
> are not meant to run simultaneously.

---

## Key Changes vs. Upstream

The two packages started from different upstream codebases and diverged for different reasons. This section summarizes the substantive deltas — small config tweaks, log-level changes, and routine refactors aren't enumerated here; consult `git log` for the exhaustive list.

### GLIM (vs. `koide3/glim`, `glim_ext`, `glim_ros2`)

Upstream GLIM publishes `glim`, `glim_ext`, and `glim_ros2` as three sibling repos. This fork keeps them together inside `DLIO_plusplus/GLIM/` and adds:

**Sensor / preprocessing**

- **Multi-LiDAR concatenation (`lidar_concat`).** New module in `glim_ros2` (`include/glim_ros/lidar_concat.hpp`) that subscribes to N aux LiDAR topics, time-aligns each scan to the primary clock, transforms aux points into the primary frame, **rebases per-point timestamps** so the concatenated cloud has a single monotonic time base (anchored on the primary scan's earliest timestamp), and emits a single merged cloud. Includes a validation step that **rolls back the aux-merge append** on a malformed cloud, and the shared **strict merge guard** (`require_all_aux` / `abort_on_merge_failure`, defaults matching GICP — see [Multi-LiDAR merge policy](#multi-lidar-merge-policy)).
- **Offline extrinsic resolution.** Aux-LiDAR (and IMU/GNSS) extrinsics are read from `av24.urdf` — the configs reference *URDF link names* and the loader resolves the URDF at startup **CWD-independently** (walks up from the config dir; `av24.urdf` is installed into `share/glim/config`), with a static-matrix fallback. No live `/tf_static` is needed for offline replay.
- **`flip_points_y` preprocessing** flag (`config_sensors.json` → `glim_ros.cpp`) for mirrored-installed LiDARs.
- **Per-point timestamp rebasing fix** when merging multi-LiDAR clouds (commit `7f5a6d9`). Without this the merged cloud had a non-monotonic stamp field that broke deskewing.

**Mapping / odometry**

- **INS-driven odometry mode** for sparse-feature stretches (commit `c26c8b0`). Lets the optimizer lean on INS odometry when LiDAR geometry is degenerate (e.g. open sky and runway-like surfaces).
- **Race-car drift tuning** in `glim/config/` (commit `20ba88d`). Defaults relaxed to admit higher angular rates and lateral slip than the road-car defaults assume.

**GNSS extension (`glim_ext/modules/mapping/gnss_global`)**

- **GNSS-to-map SE(3) export** (`T_world_utm.txt`) once GNSS alignment initializes, recovered by a 2D Umeyama fit of the submap trajectory to the GNSS input frame. With the adapter feeding **local ENU**, that transform is effectively world↔ENU (the filename/variable keep the historical `utm` name). Downstream `gicp_localization` can optionally consume it for a legacy `utm`-frame mirror, but the operational contract is local ENU — see [Coordinate frames](#coordinate-frames--local-enu).
- **URDF lever-arm support** (commit `50d9c0c`): the IMU→GNSS lever-arm is taken from the URDF rather than from a manual offset in the config.
- **Dual-antenna heading priors (default ON).** `enable_orientation_prior: true` with yaw-only precisions (`[1e-6, 1e-6, 1e2]` ≈ 5.7° sigma) adds a `PoseRotationPrior` per submap from the Atlas heading, pinning map yaw where the position prior can't. Hardened (P5) with a **per-sample yaw-quality gate** (`orientation_prior_max_yaw_sigma_deg: 3.0`): a position-FIXED sample whose reported heading sigma is degraded skips the heading prior (position prior still applied) — the upstream RTK filter qualifies position quality only.
- **Strip stale GNSS rotation priors on graph reload** (commit `6a50632`) so a re-opened graph doesn't double-apply an orientation constraint that no longer matches the live frame.
- **Warn when URDF IMU↔GNSS rotation breaks the lever-arm assumption** (commit `622271f`). The lever-arm math assumes IMU and GNSS share orientation; if the URDF says otherwise the user is told instead of silently getting biased corrections.
- Switched the noise model expression from `Isotropic::Information(diagonal)` (which silently dispatched to `Gaussian::Information(Matrix)` through inheritance) to `Diagonal::Precisions(vector)` (commit `d8b2809`). Same numerical result, with an explicit diagonal-precision signature.

**Offline tooling**

- **`glim_pcap_rosbag`** (`glim_ros2/src/glim_pcap_rosbag.cpp` + `iris_pcap_reader.cpp`). Reads raw Luminar `.pcap` files alongside a sibling mcap (for IMU and GNSS) and runs offline mapping directly, skipping the intermediate "decode pcap into a bag" step. Useful when the live-recorded mcap is missing LiDAR or had a decode hiccup.

### gicp_localization (vs. the vectr-ucla DLIO line)

`gicp_localization` is built around a vendored `nano_gicp`. Compared to a stock DLIO odometry node turned into a localizer, this fork adds:

**Scan-to-map vs. scan-to-submap**

- **Single pre-built PCD map.** No submap stitching at runtime — the map is loaded once and never grows. Trades adaptability for a small, predictable working set.
- **Multi-LiDAR concatenation** (mirrors the GLIM-side feature, same strict-guard flags and defaults). Subscribes to N aux LiDARs, resolves extrinsics offline (URDF → static matrix → live TF), and concatenates onto the primary cloud's clock. The target map is voxel-downsampled at load (`localization/map_voxel_size: 0.3`) to bound the kd-tree memory, and the crop box runs in **sensor frame before deskew**.

**Robustness against degenerate geometry** (reworked in P1, 2026-07)

- **Map-independent deployment gating** on every GICP solve:
  1. A 30% correspondence-support gate plus finite-pose validation.
  2. Absolute and rolling-ratio fitness rejection disabled for
     perception-ws parity; a finite high ceiling still rejects NaN/Inf.
  3. **Degeneracy partial updates** (solution remapping): when the hessian condition proxy trips, the correction is projected onto well-constrained eigen-directions of the vehicle-re-centered, unit-scaled 6×6 hessian (coupled rot/trans null directions included; `degeneracy/full6d`), and the IMU prior is kept along degenerate axes — status `ok_partial` instead of a rejected scan.
  4. **Turn-aware yaw-consistency veto** (`yawGate/*`) and hard physical yaw bounds.
  5. Large-jump reject vs. the IMU-predicted prior (speed/scan-dt-aware thresholds).
- **IMU dead-reckoning fallback.** Rejected scans propagate from the IMU-integrated prior — seeded with the *current* IMU-propagated velocity (P2 fixed a stale-velocity bug that made dead-reckoned priors cut corners).
- **GT-driven pose recovery** (enabled by default, `min_consecutive_failures: 5`). When GICP rejects N scans in a row, snap pose + twist to a time-matched GT odom sample; angular rate backfills from the live gyro and linear velocity from GT finite-differencing when the odom twist is unpopulated (P2).
- **`getFitnessScore` correctness fixes.** Cleared `sq_distances_` per align so stale distances couldn't leak into the score (commit `d949e64`); cached `sq_distances_` reused inside `NanoGICP::getFitnessScore` to avoid recomputing nearest neighbors (commit `1e27b84`).

**Geometric observer**

- **Observer + IMU pipeline aligned to upstream DLIO design** (commit `db5cda8`). The original lift-and-shift had subtle differences in how the geometric observer was driven; this commit brings the data path back in line with DLIO's reference implementation so IMU dead-reckoning is mathematically consistent with scan corrections.
- **Delta-form correction target** (P3, `odom/geo/delta_correction: true`). The GICP measurement is 0.1–0.3 s stale by the time the observer applies it; the legacy absolute target dragged the state backwards proportionally to yaw rate (the "turn error"). The observer now applies the time-free correction `T_meas · T_prior⁻¹` to the *current* state — zero correction at any latency when IMU and GICP agree. Gains unchanged.
- **Single bias-application point** (P3). IMU biases are subtracted once, at buffering, so `propagateState`, the scan prior, and per-point deskew all integrate the same corrected signal (previously the prior/deskew path ran on raw gyro).

**Initialization**

- **GT-bootstrapped initial pose** (commit `add7a54`). The first message on the GT odom topic seeds the localizer; works for any bag start-offset without hand-tuning numerics or clicking in RViz.
- Three init paths in priority order: GT bootstrap → numeric pose from YAML (with `frame: "lidar"` mode that auto-applies `inv(T_base_lidar)` for direct pasting from GLIM's `traj_lidar.txt`) → RViz "2D Pose Estimate".

**Output frames**

- **Local-ENU output (default).** The primary `map`-frame pose/odom/path are already in local ENU because the map and the adapter's seed are ENU — GICP is frame-agnostic and just reports in the map's frame. No transform step needed.
- **Optional UTM mirror (legacy).** Only if `localization/utm_transform_path` points at a `T_world_utm.txt`, the node additionally publishes pose/odom/path in a `utm` frame. Off by default.
- **Offline extrinsic resolution.** Both the aux-LiDAR transforms and the `base_frame ← lidar_frame` lever arm are resolved from `av24.urdf` (with a static-matrix fallback), so replay works without `robot_state_publisher` / `/tf_static`.
- **TF policy:** the `map → base_link` TF broadcast is disabled by default (commit `9cc18d7`) to avoid fighting other publishers; downstream nodes consume the published `nav_msgs/msg/Odometry` instead.

**Operational defaults**

- **Evidence-first defaults** (changed in the P1–P4 review): `localization/debug/enable_pub` and `verbose_scan_log` are **on** by default so every replay produces the per-frame debug topics and `SCAN DEBUG` lines the validation scorecard (`scripts/analyze_scan_debug_log.py`) consumes — measured cost is trivial (14 MB debug bag over a 36 min replay). General INFO verbosity, jump logs, and outgoing point-cloud topics remain off; disable the debug flags only for resource-constrained live deployment.

---

## Repo Layout

```
DLIO_plusplus/
├── adapter/             # Atlas -> local-ENU /gps_p1/* normalization package
├── GLIM/                # SLAM workspace (glim, glim_ext, glim_ros2)
├── gicp_localization/   # Map-based localization package
├── dlio/                # Convenience metapackage
├── av24.urdf            # Vehicle URDF (drives all sensor extrinsics)
├── scripts/             # prep_bag.py (adapter normalization), map export, analysis
├── profiling_logs/      # Resource-profile CSVs + comparison plots
└── README.md            # This file
```

## License

GLIM and `gtsam_points` are MIT-licensed; GTSAM is BSD. See the upstream repositories and `GLIM/README.md` for full attributions.
