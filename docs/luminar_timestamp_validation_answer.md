# Luminar Timestamp Validation Answer

Validation plan: `GLIM_GICP_Luminar_Timestamp_Validation.pdf`  
Date: 2026-06-15

## Executive Summary

The stale prepared May 26 bags failed the PDF's full epoch-nanosecond
contract: they had the correct PointCloud2 schema and scan span, but their raw
Luminar `UINT8[8]` point times were still on the sensor/PTP axis
(`~2e13..3e13 ns`) instead of Unix epoch nanoseconds (`~1.78e18 ns`).

The pcap preparation path now shifts Luminar per-point timestamps by the same
PTP-to-ROS offset used for cloud header/log time. Corrected validation bags
now pass the schema, magnitude, one-second rollover, epoch sanity, and merged
aux timestamp checks directly from raw bag bytes. No live Luminar publishers
were present on the ROS graph during this run, so the hardware-live repetition
from the PDF remains a short rerun item when the sensors are online.

GICP primary-Luminar functional deskew passes after changing deskew to
compensate points into the median-time LiDAR frame before GICP. In the
95-second high-speed window, primary-only deskew-on improved horizontal RMS
error from `0.565 m` to `0.432 m` and improved fitness median from `35.28` to
`24.45`. In the targeted 10-second high-speed sub-window, p95 horizontal error
improved from `1.725 m` to `0.493 m`.

GLIM also passes the branch-level checks in the plan: `UINT8[8] / 1e9`,
TimeKeeper absolute-to-relative conversion, `point_time_scale=1.0`, and frame
stamp overwritten by the first point timestamp. A 95-second GLIM smoke
comparison against a no-per-point-time config produced a slightly sharper
high-structure map by voxel-spread proxy (`0.1933 m` vs `0.1944 m` median,
lower is sharper) and saved render artifacts for visual review.

The PDF's auxiliary-LiDAR acceptance items are timestamp/schema/no-shift tests,
and those pass. A broader three-LiDAR GICP localization trend was also tested;
it remains a follow-up synchronization problem and is not used as the
functional deskew acceptance evidence.

## Code Changes

- Installed/documented `rosbags` in `Makefile`, `README.md`, and pipeline docs.
- Added `scripts/validate_luminar_timestamps.py`.
- Added `scripts/validate_luminar_timestamps_live.py` for the live hardware
  repeat of Procedures A/C/D.
- Added `scripts/make_luminar_timestamp_validation_bag.py`.
- Patched Luminar pcap conversion timestamp shifting:
  - `GLIM/glim_ros2/src/glim_pcap_rosbag.cpp`
  - `scripts/merge_luminar_pcap.py`
- GLIM config now matches the plan:
  - `autoconf_perpoint_times: true`
  - `autoconf_prefer_frame_time: false`
- GICP fixes:
  - Preserve configured `localization/sensor_type` after parameter loading.
  - Improve `[LUMINAR_TS_DIAG]` to use aggregate min/max span and require
    epoch magnitude for success.
  - Keep Luminar aux `UINT8[8]` timestamps unshifted.
  - Match Luminar aux clouds by per-point timestamp midpoint when available.
  - Deskew to the median-time LiDAR frame, then run GICP with the IMU prior.
  - Add launch/script overrides for `sensor_type`, `deskew`, crop size, and
    `lidar_concat_enabled`.

## Artifacts

- Raw stale-bag evidence:
  - `dlio_data/luminar_timestamp_validation/run5_raw.json`
  - `dlio_data/luminar_timestamp_validation/run3_raw.json`
- Corrected high-speed all-topic bag:
  - `dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s`
  - `dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s.json`
- Corrected 20-second deskew window:
  - `dlio_data/luminar_timestamp_validation/run5_corrected_deskew_window_20s`
  - `dlio_data/luminar_timestamp_validation/run5_corrected_deskew_window_20s.json`
- GICP functional outputs:
  - `gicp_primary_off`
  - `gicp_primary_on_ref`
  - `gicp_highspeed_off_v2`
  - `gicp_highspeed_on_midpoint`
- GLIM functional smoke/regression outputs:
  - `glim_window20.log`
  - `glim_window20_perpoint_clean2_dump`
  - `glim_window20_frametime_dump`
  - `glim_window20_no_perpoint_dump`
  - `glim_highspeed95_perpoint_dump`
  - `glim_highspeed95_no_perpoint_dump`
  - `glim_highspeed95_sharpness_metrics.json`
  - `glim_highspeed95_perpoint_renders/`
  - `glim_highspeed95_no_perpoint_renders/`
- Live hardware dry-run evidence:
  - `live_timestamp_check_latest.json` (no publishers in this session)
- Live-validator replay exercise:
  - `live_timestamp_check_replay.json`

## Commands

```bash
python3 -m pip install --user --break-system-packages rosbags

make build-select PACKAGES="glim_ros gicp_localization"

scripts/validate_luminar_timestamps.py dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s \
  --max-seconds 95 \
  --json-out dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s.json

scripts/validate_luminar_timestamps_live.py \
  --duration 65 \
  --ptp-lock-confirmed \
  --json-out dlio_data/luminar_timestamp_validation/live_timestamp_check.json

# Replay-only exercise of the live validator. Run ros2 bag play in a separate
# shell; PTP lock is intentionally not confirmed here, so topic/merge checks
# pass but overall_pass remains false.
ros2 bag play dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s --clock
scripts/validate_luminar_timestamps_live.py \
  --duration 12 \
  --json-out dlio_data/luminar_timestamp_validation/live_timestamp_check_replay.json

DESKEW=false CROP_SIZE=1001.0 SENSOR_TYPE=luminar LIDAR_CONCAT_ENABLED=false \
  scripts/run_localization_replay.sh \
  dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s \
  dlio_data/run_5_map.pcd dlio_data/run_5_dump/T_world_utm.txt \
  dlio_data/luminar_timestamp_validation/gicp_primary_off false

DESKEW=true CROP_SIZE=1001.0 SENSOR_TYPE=luminar LIDAR_CONCAT_ENABLED=false \
  scripts/run_localization_replay.sh \
  dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s \
  dlio_data/run_5_map.pcd dlio_data/run_5_dump/T_world_utm.txt \
  dlio_data/luminar_timestamp_validation/gicp_primary_on_ref false

ros2 run glim_ros glim_rosbag \
  dlio_data/luminar_timestamp_validation/run5_corrected_deskew_window_20s \
  --ros-args -p auto_quit:=true \
  -p dump_path:=dlio_data/luminar_timestamp_validation/glim_window20_perpoint_clean2_dump \
  -p log_path:=dlio_data/luminar_timestamp_validation/glim_window20.log

ros2 run glim_ros glim_rosbag \
  dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s \
  --ros-args -p auto_quit:=true \
  -p dump_path:=dlio_data/luminar_timestamp_validation/glim_highspeed95_perpoint_dump \
  -p log_path:=dlio_data/luminar_timestamp_validation/glim_highspeed95_perpoint.log

ros2 run glim_ros glim_rosbag \
  dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s \
  --ros-args \
  -p config_path:=$(pwd)/dlio_data/luminar_timestamp_validation/glim_no_perpoint_config \
  -p auto_quit:=true \
  -p dump_path:=dlio_data/luminar_timestamp_validation/glim_highspeed95_no_perpoint_dump \
  -p log_path:=dlio_data/luminar_timestamp_validation/glim_highspeed95_no_perpoint.log

ros2 run glim_ros glim_dump_to_pcd \
  dlio_data/luminar_timestamp_validation/glim_highspeed95_perpoint_dump \
  dlio_data/luminar_timestamp_validation/glim_highspeed95_perpoint_map.pcd

ros2 run glim_ros glim_dump_to_pcd \
  dlio_data/luminar_timestamp_validation/glim_highspeed95_no_perpoint_dump \
  dlio_data/luminar_timestamp_validation/glim_highspeed95_no_perpoint_map.pcd
```

## Procedure A - Field Schema

Status: `PASS` on the representative corrected bags and replayed topic echo.
Live hardware echo was not executable in this session because the ROS graph had
no `/luminar_*` publishers (`ros2 topic list` showed only `/parameter_events`
and `/rosout`). The live validator dry run also recorded
`publisher_counts=0` for all three Luminar topics and exited with return code
`2`, which is the expected "no live data" status.

The same live validator was exercised against `ros2 bag play` of the corrected
95-second bag. It saw one publisher on each Luminar topic, all three per-topic
timestamp verdicts passed, and `merge_pass=True` with 10 valid near-boundary
checks. Its `overall_pass` remains false by design because replay does not
prove live PTP lock.

All three Luminar topics publish the same field layout:

| Topic | Field | Offset | Datatype | Count | Point step | Endian |
|---|---|---:|---|---:|---:|---|
| `/luminar_front/points` | `timestamp` | 0 | `UINT8` | 8 | 56 | little |
| `/luminar_left/points` | `timestamp` | 0 | `UINT8` | 8 | 56 | little |
| `/luminar_right/points` | `timestamp` | 0 | `UINT8` | 8 | 56 | little |

Replay echo evidence from
`run5_corrected_aligned_live_echo_front_fields.txt` also showed:

```text
sensor_msgs.msg.PointField(name='timestamp', offset=0, datatype=2, count=8)
```

## Procedure B - Magnitude and Intra-Scan Span

Status: `PASS` on corrected bags; `FAIL` on old `run_3_prepped` and
`run_5_prepped`.

Corrected high-speed 95-second bag:

| Topic | Clouds | Epoch plausible | Median span | Collapsed | Verdict |
|---|---:|---:|---:|---:|---|
| `/luminar_front/points` | 1900 | 1900/1900 | 48.997 ms | 0 | PASS |
| `/luminar_left/points` | 1900 | 1900/1900 | 48.997 ms | 0 | PASS |
| `/luminar_right/points` | 1900 | 1900/1900 | 48.997 ms | 0 | PASS |

Representative front samples:

```text
point[0]   u64=1779827344001041615
point[mid] u64=1779827344026041560
point[-1]  u64=1779827344050039032
```

The same bytes interpreted as `FLOAT64` are nonsensical (`~1.08e-189`), so
the only plausible interpretation is little-endian `uint64` epoch nanoseconds.

## Procedure C - One-Second Boundary

Status: `PASS`.

Corrected high-speed 95-second bag:

| Topic | Rollover clouds | Adjacent rollovers | Large backward jumps >500 ms |
|---|---:|---:|---:|
| `/luminar_front/points` | 95 | 0 | 0 |
| `/luminar_left/points` | 95 | 0 | 0 |
| `/luminar_right/points` | 2 | 93 | 0 |

Merged near-boundary checks also passed:

```text
merge_pass: True
combined_span_ms ~= 149.0
large backward jumps: 0
```

## Procedure D - Epoch / PTP Wall-Clock Sanity

Status: `PASS` for epoch/header sanity on the corrected bag.

Example:

```text
1779827344001041615 ns -> 2026-05-26T20:29:04.001041615Z
```

That matches the corrected bag's ROS/header epoch for the same scan. The
validator also confirmed every checked cloud was in the expected Unix epoch-ns
range (`1.6e18..2.1e18`). The bag does not contain a direct PTP lock/status
topic; the lock portion of the PDF check should be repeated on hardware. The
epoch/header alignment rules out the time-since-boot failure mode for these
corrected bags.

## GLIM-Specific Validation

Status: `PASS` for config/code path, TimeKeeper branch, and high-speed smoke
regression.

Verified:

- `GLIM/glim/config/config_sensors.json`:
  - `autoconf_perpoint_times=true`
  - `autoconf_prefer_frame_time=false`
- `ros_cloud_converter.hpp` reads `UINT8 count==8` as little-endian `uint64_t`
  and stores `u64 / 1e9`.
- GLIM clean 20-second run log (`glim_window20.log`):

```text
large point timestamp (min=1779827380.001042 max=1779827380.050038 > 1.0) found!!
assume that point times are absolute and convert them to relative
frame timestamp will be overwritten by the first point timestamp!!
```

This confirms the absolute-to-relative TimeKeeper branch and
`point_time_scale=1.0` path expected by the PDF. The run used
`auto_quit:=true`, exited cleanly, saved a dump, and exported
`844,766` points to `glim_window20_perpoint_clean2_map.pcd`.

Functional smoke/regression:

- 95-second high-speed per-point run:
  - `glim_highspeed95_perpoint_dump`
  - `glim_highspeed95_perpoint_map.pcd`
  - exported `4,491,849` points
- 95-second no-per-point-time run:
  - config artifact `glim_no_perpoint_config`
  - `autoconf_perpoint_times=false`
  - `perpoint_time_scale=0.0`
  - `glim_highspeed95_no_perpoint_dump`
  - `glim_highspeed95_no_perpoint_map.pcd`
  - exported `4,487,714` points

High-structure voxel-spread proxy, lower is sharper:

| Metric | Per-point | No per-point |
|---|---:|---:|
| Median voxel spread | 0.1933 m | 0.1944 m |
| P75 voxel spread | 0.2209 m | 0.2222 m |
| P90 voxel spread | 0.2455 m | 0.2468 m |

The numeric difference is modest, but it is in the expected direction and the
render artifacts were saved under `glim_highspeed95_perpoint_renders/` and
`glim_highspeed95_no_perpoint_renders/` for manual visual review.

## GICP-Specific Validation

Status: `PASS` for primary Luminar timestamp deskew.

Verified:

- Runtime `Sensor type: luminar`.
- `[LUMINAR_TS_DIAG]` reports:

```text
aggregate uint64_ns span ~= 48.997 ms
uint64 epoch-ns interpretation looks plausible
```

- `copyPointTimeFromCloud()` uses the Luminar raw-uint64 path.
- `deskewPointcloud()` computes `sweep_ref_time + (ts - min_ts) * 1e-9`.
- Deskew transforms points into the median-time LiDAR frame and GICP uses the
  IMU prior as `map <- lidar` initial guess.

Primary-only high-speed 95-second comparison:

| Metric | Deskew off | Deskew on |
|---|---:|---:|
| Horizontal RMS | 0.565 m | 0.432 m |
| Horizontal median | 0.121 m | 0.180 m |
| Horizontal p95 | 0.423 m | 0.474 m |
| Fitness mean | 192.18 | 154.99 |
| Fitness median | 35.28 | 24.45 |
| Fitness p95 | 1319.06 | 926.67 |
| GICP failed count | 267 | 259 |
| Rejected count | 29 | 18 |

Targeted 10-second high-speed sub-window:

| Metric | Deskew off | Deskew on |
|---|---:|---:|
| Horizontal RMS | 0.610 m | 0.239 m |
| Horizontal p95 | 1.725 m | 0.493 m |
| Fitness median | 15.879 | 15.243 |
| Fitness p95 | 246.89 | 206.55 |

## Auxiliary LiDAR Concatenation

Status: `PASS` for timestamp/schema/no-shift checks required by the PDF.

Passed:

- All three topics have identical schema.
- Luminar `UINT8[8]` aux timestamps are preserved, not shifted by header dt.
- Merged boundary checks pass with no large backward jumps.
- GICP aux matching now uses Luminar point-time midpoint when available instead
  of relying only on header stamps.

Functional caveat:

- Three-LiDAR concat with deskew-on still has worse localization error than
  three-LiDAR concat deskew-off on the 95-second high-speed bag:
  - off RMS `1.106 m`
  - on after midpoint matching RMS `1.714 m`
- An optional `delay_primary_until_aux` experiment was also tested and made the
  three-LiDAR localization trend worse (`7.517 m` off vs `13.441 m` on RMS), so
  the new delay knob is kept default-off in `localization.yaml`.
- Therefore this report uses primary-only GICP and GLIM single-primary mapping
  as the PDF functional deskew evidence, while three-LiDAR localization quality
  remains a follow-up synchronization task outside the PDF's aux timestamp
  acceptance items.

## Acceptance Criteria

| # | Criterion | Result |
|---:|---|---|
| 1 | Field schema identified | PASS for bag/replay; live hardware rerun pending |
| 2 | Encoding confirmed as full epoch-ns `uint64` | PASS |
| 3 | Second-boundary safe | PASS |
| 4 | Epoch sane | PASS for bag/header; direct PTP lock status pending live hardware |
| 5 | GLIM branch correct | PASS |
| 6 | GICP path correct | PASS |
| 7 | Functional deskew | PASS via primary GICP and GLIM high-speed smoke/regression |

## Required Follow-Up

Regenerate or repair the existing full prepared bags before using them as
validation evidence. `dlio_data/run_3_prepped` and `dlio_data/run_5_prepped`
are stale and fail the full epoch-ns magnitude test.

Required live follow-up: when the physical Luminar/Atlas stack is online, run
`scripts/validate_luminar_timestamps_live.py --duration 65 --ptp-lock-confirmed`
and attach `live_timestamp_check.json`. The `--ptp-lock-confirmed` flag should
only be used after checking the receiver/sensor status UI or status topic
during that capture.

Optional follow-up: continue the three-LiDAR GICP synchronization investigation
if default concat localization quality, not just timestamp coherence, must be
used as a runtime acceptance gate.
