# Luminar Timestamp Validation Answer

Validation plan: `GLIM_GICP_Luminar_Timestamp_Validation.pdf`
Final update: 2026-06-15
Scope: AV-24 DLIO++ GLIM/GICP Luminar per-point timestamp validation using the
current code plus May 26 `run_5` and `run_3` bag evidence.

## Final Verdict

Bagged/offline validation is complete and passes the PDF's timestamp contract:
all three Luminar topics expose a `UINT8[8]` timestamp field at offset `0`,
the bytes decode as little-endian full epoch nanoseconds, intra-scan span is
preserved, second rollovers are safe, GLIM takes the absolute-to-relative
per-point-time branch, and GICP uses the Luminar raw-uint64 path.

The original prepared May 26 bags exposed a real bug: cloud `header.stamp` and
bag log time were on the ROS/INS epoch axis, but the Luminar `UINT8[8]`
per-point timestamps remained on the sensor/PTP axis (`~2e13..3e13 ns`).
GLIM therefore overwrote frame time with a non-epoch point time and skipped the
mapping data as unsynchronized. The current `scripts/prep_bag.py` repairs this
by adding `header.stamp - min(point_time)` to every Luminar point timestamp,
while preserving the scan span.

The non-live `PIPELINE.md` command coverage was rerun after this fix and after
the localization recovery fixes. `make build`, `make build-all`, wrapper
examples, manual mapping/export/viewer commands, cache-viz smoke commands,
run_5 localization replay, five-process replay, and the concrete run_3
validation example all completed successfully. The large generated bags, maps,
dumps, caches, and replay directories were removed after the successful run;
compact local log archives were retained outside the repository.

The only incomplete PDF item is the live hardware PTP-lock repetition. During
the final check no active Luminar publishers produced samples, so
`scripts/validate_luminar_timestamps_live.py` correctly returned the no-live-
data state with `publisher_counts=0` and `seen_counts=0` for all three topics.

## Root Cause And Fix

The PDF's root-cause statement is confirmed: Luminar Iris does not publish one
single uint64 timestamp on the wire. The sensor data model is split into packet
PTP seconds plus per-ray nanoseconds; the ROS2 driver or prep path must expose
the reconstructed value as one full epoch-nanosecond field in `PointCloud2`.

Current pipeline contract:

- GLIM reads `PointField(datatype=UINT8, count=8)` as little-endian `uint64_t`
  and converts `u64 / 1e9` to epoch seconds.
- GICP's Luminar path stores the raw uint64 nanoseconds and deskews with
  `(ts - min_ts) * 1e-9`.
- `prep_bag.py` repairs Luminar point timestamps onto the same epoch as
  `header.stamp` by default. `--no-lidar-point-time-repair` exists only for
  bags whose point timestamps are already on that epoch.

This repair changes the absolute epoch of the point timestamps; it does not
collapse or stretch the per-point scan timing. The validator evidence below
shows the scan span remains about `48.997 ms`.

## Evidence Sources

Retained local evidence:

- `dlio_pipeline_full_20260615_013431_logs.tgz`
  - full `PIPELINE.md` command coverage logs and status files
  - `validate_noargs_run5_prepped.json`
- `dlio_postfix_validation_logs_20260615_093519.tgz`
  - post-fix pure-GICP high-speed deskew logs
  - targeted run_3 tail replay logs
- `dlio_postfix_validation_summary_20260615_093519.txt`
- `live_timestamp_check_current_20260615_094312.json`

Repository-local validation artifacts:

- `dlio_data/luminar_timestamp_validation/run5_raw.json`
- `dlio_data/luminar_timestamp_validation/run3_raw.json`
- `dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s.json`
- `dlio_data/luminar_timestamp_validation/run5_corrected_deskew_window_20s.json`
- `dlio_data/luminar_timestamp_validation/live_timestamp_check_replay.json`

Important distinction: the metrics that mention regenerated `run_5_prepped` or
`run_3_prepped` refer to isolated retest outputs. Older worktree directories
such as `dlio_data/run_5_prepped` or `dlio_data/run_3_prepped` should not be
used as proof unless they pass `scripts/validate_luminar_timestamps.py` in the
current checkout.

## Procedure A - Field Schema

Status: `PASS` for bagged/replayed data; live hardware repetition pending.

All three Luminar topics have the required time field:

| Topic | Field | Offset | Datatype | Count | Point step | Endian |
|---|---|---:|---|---:|---:|---|
| `/luminar_front/points` | `timestamp` | 0 | `UINT8` | 8 | 56 | little |
| `/luminar_left/points` | `timestamp` | 0 | `UINT8` | 8 | 56 | little |
| `/luminar_right/points` | `timestamp` | 0 | `UINT8` | 8 | 56 | little |

Replay echo evidence also showed:

```text
sensor_msgs.msg.PointField(name='timestamp', offset=0, datatype=2, count=8)
```

The live validator was executed against current ROS2 graph state, but no active
publishers produced Luminar samples. That is not a timestamp-contract failure;
it is an external live-data availability blocker for the PDF's live-hardware
repeat.

## Procedure B - Magnitude And Intra-Scan Span

Status: `PASS` on corrected validation bags and on prepared bags generated by
the current `prep_bag.py`.

Full pipeline regenerated `run_5_prepped` validator:

| Topic | Clouds | Epoch plausible | Median span | Collapsed | >500 ms backward jumps | Verdict |
|---|---:|---:|---:|---:|---:|---|
| `/luminar_front/points` | 1801 | 1801/1801 | 48.997 ms | 0 | 0 | PASS |
| `/luminar_left/points` | 1801 | 1801/1801 | 48.997 ms | 0 | 0 | PASS |
| `/luminar_right/points` | 1801 | 1801/1801 | 48.997 ms | 0 | 0 | PASS |

Corrected high-speed 95-second validation bag:

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

The same bytes interpreted as `FLOAT64` are nonsensical (`~1.08e-189`), so the
only plausible interpretation is little-endian `uint64` epoch nanoseconds.

## Procedure C - One-Second Boundary

Status: `PASS`.

Full pipeline regenerated `run_5_prepped` had `91` one-second rollover clouds
on each Luminar topic, zero adjacent large backward jumps, and `merge_pass=True`.

Corrected high-speed 95-second validation bag:

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

This distinguishes the current full-epoch reconstruction from the failure mode
where only bare sub-second nanoseconds are forwarded and wrap every second.

## Procedure D - Epoch / PTP Wall-Clock Sanity

Status: `PASS` for bag/header epoch sanity; direct live PTP-lock confirmation
pending.

Example:

```text
1779827344001041615 ns -> 2026-05-26T20:29:04.001041615Z
```

That matches the corrected bag's ROS/header epoch for the same scan. The
validator also confirmed every checked cloud is in the expected Unix epoch-ns
range (`1.6e18..2.1e18`).

The bag does not carry a direct PTP lock/status topic. The lock portion of the
PDF check must therefore be repeated on live hardware with the receiver/sensor
status confirmed at capture time. The live validator intentionally requires
`--ptp-lock-confirmed` for that final proof.

## GLIM-Specific Validation

Status: `PASS` for code path, configuration, TimeKeeper branch, and functional
high-speed smoke/regression.

Verified current configuration:

- `GLIM/glim/config/config_sensors.json`
  - `autoconf_perpoint_times=true`
  - `autoconf_prefer_frame_time=false`
  - `perpoint_time_scale=1.0`
- `ros_cloud_converter.hpp` reads `UINT8 count==8` as little-endian `uint64_t`
  and stores `u64 / 1e9`.

GLIM 20-second run log:

```text
large point timestamp (min=1779827380.001042 max=1779827380.050038 > 1.0) found!!
assume that point times are absolute and convert them to relative
frame timestamp will be overwritten by the first point timestamp!!
```

This is the expected PDF branch: full epoch seconds enter GLIM, TimeKeeper
detects absolute timestamps, converts them to relative per-point offsets, and
keeps the scale at `1.0`.

Functional smoke/regression:

| GLIM run | Output | Result |
|---|---|---|
| 20-second corrected window | `glim_window20_perpoint_clean2_dump` | clean exit; `844,766` exported points |
| 95-second per-point run | `glim_highspeed95_perpoint_dump` | `4,491,849` exported points |
| 95-second no-per-point-time run | `glim_highspeed95_no_perpoint_dump` | `4,487,714` exported points |

High-structure voxel-spread proxy, lower is sharper:

| Metric | Per-point | No per-point |
|---|---:|---:|
| Median voxel spread | 0.1933 m | 0.1944 m |
| P75 voxel spread | 0.2209 m | 0.2222 m |
| P90 voxel spread | 0.2455 m | 0.2468 m |

The numeric difference is modest, but it is in the expected direction and the
render artifacts were saved for visual review.

## GICP-Specific Validation

Status: `PASS` for Luminar timestamp decode, per-point offset generation, and
isolated deskew fitness behavior. Production replay health was validated
separately with the default GT recovery/rejection path.

Verified current GICP path:

- Runtime sensor type is `luminar`.
- `[LUMINAR_TS_DIAG]` reports an aggregate `uint64_ns` span of about
  `48.997 ms`.
- `[LUMINAR_TS_DIAG]` verdict: `uint64 epoch-ns interpretation looks plausible`.
- `copyPointTimeFromCloud()` uses the Luminar raw-uint64 path.
- `deskewPointcloud()` uses `sweep_ref_time + (ts - min_ts) * 1e-9`.
- Deskew transforms points into the timestamp reference LiDAR frame before
  registration, and GICP uses the IMU prior as the `map <- lidar` initial guess.

Pure-GICP high-speed 95-second comparison
(`GT_RECOVERY_ENABLED=false`, `GT_REJECTION_ENABLED=false`,
`GT_VETO_ENABLED=false`):

| Metric | Deskew off | Deskew on |
|---|---:|---:|
| Fitness samples | 682 | 518 |
| Fitness mean | 245045.742 | 172208.625 |
| Fitness median | 129806.463 | 33271.592 |
| Fitness p95 | 651644.968 | 642243.413 |
| Fitness max | 669771.491 | 662148.339 |
| GICP failed count | 97 | 77 |
| GT snap / timeout hold | 0 / 0 | 0 / 0 |

This pure-GICP run intentionally disables production recovery. Both replays
eventually diverge during the aggressive window, so odometry RMS from this
specific ablation is not used as the acceptance metric. The relevant result is
that registration fitness improves with deskew on, without GT snap/hold events
masking the GICP behavior.

Default production-style localization replay:

| Replay | Odom samples | Rate | Horizontal RMS | Horizontal p95 | Notes |
|---|---:|---:|---:|---:|---|
| Full run_5 manual replay | 78291 | ~99 Hz | 0.317 m | 0.578 m | `live_error.csv` and `live_error.png` generated |
| Full run_3 validation | 231819 | ~100 Hz | 0.375 m | 0.664 m | concrete run_3 pipeline example |
| run_3 tail replay | 19723 | ~100 Hz | 0.607 m | 0.172 m | previous failure window; post-failure live-error p95 `0.283 m` |

## Auxiliary LiDAR Concatenation

Status: `PASS` for the PDF's aux timestamp/schema/no-shift requirements.

Passed checks:

- Front, left, and right Luminar topics have identical time-field schema.
- `UINT8[8]` aux timestamps remain absolute epoch timestamps and are not
  rebased by header delta during merge.
- Merged second-boundary checks pass with no large backward jumps.
- GICP aux matching uses Luminar point-time midpoint when available instead of
  relying only on header stamps.

The PDF asks for timestamp coherence of concatenated Luminar clouds. It does
not require a separate multi-LiDAR localization ablation. The default full
pipeline replay, which uses the current multi-LiDAR path, passed the run_5 and
run_3 localization checks above.

## Full Pipeline Retest

Status: `PASS` for the non-live `PIPELINE.md` command coverage.

Covered command groups:

- Environment and build:
  - `make install-deps`
  - `make install-gtsam-points-cuda`
  - `make build`
  - `make build-all`
- Wrapper paths:
  - dry run
  - no-arg run_5 pipeline
  - explicit run_5 pipeline
  - run_3 against run_5 map with RViz flag
  - prepped-reuse wrapper
- Manual mapping/export:
  - run_5 prep
  - origin-pinned run_3 prep
  - `glim_rosbag`
  - `eval_traj_vs_gnss.py`
  - `glim_dump_to_pcd`
  - `render_map.py`
  - `export_map_html.py`
  - HTTP `index.html` check
- Viewer/cache smoke:
  - GLIM offline viewer startup
  - cached error viz startup
  - `make viz-cache` startup
- Localization:
  - one-command run_5 replay
  - five-process manual replay
  - concrete run_3 validation replay

Representative outputs:

| Item | Result |
|---|---|
| Full `run_5_prepped` timestamp validator | `overall_pass=True`, `schema_equal=True`, `merge_pass=True` |
| Manual run_5 map eval | horizontal RMS `0.486 m`, p95 `0.984 m` |
| Manual map export | `26,497,474` PCD points; browser export `6,259,150` points; HTTP `200 OK` |
| Manual run_5 localization replay | `78291` odom samples, ~`99 Hz`, horizontal RMS `0.317 m`, p95 `0.578 m` |
| Full run_3 validation replay | `231819` odom samples, ~`100 Hz`, horizontal RMS `0.375 m`, p95 `0.664 m` |

GUI commands are counted as pass because each process started cleanly and was
closed automatically by smoke-test timeout.

## Acceptance Criteria

| # | PDF criterion | Result |
|---:|---|---|
| 1 | Field schema identified | `PASS` for bag/replay on all three Luminars; live hardware rerun pending |
| 2 | Encoding confirmed as full epoch-ns `uint64` | `PASS` |
| 3 | Second-boundary safe | `PASS` for single-topic and merged checks |
| 4 | Epoch sane | `PASS` for bag/header epoch; direct live PTP-lock confirmation pending |
| 5 | GLIM branch correct | `PASS` |
| 6 | GICP path correct | `PASS` |
| 7 | Functional deskew | `PASS` for bagged/offline evidence via GICP fitness and GLIM smoke/regression |

Overall: the PDF validation is complete for bagged/offline evidence and
pipeline replay. It remains partial only for the live hardware PTP-lock item
because no live Luminar publishers were available during the final run.

## Reproduction Commands

Use path variables rather than private absolute paths:

```bash
DATA="${DATA:-dlio_data/luminar_timestamp_validation}"
BAG_ROOT="${BAG_ROOT:-../rosbags}"

python3 scripts/prep_bag.py \
  --input "$BAG_ROOT/putnam/may_26/run_5/filtered/all" \
  --output "$DATA/run_5_prepped"

RUN5_ORIGIN="$(cat "$DATA/run_5_prepped/utm_origin.txt")"
python3 scripts/prep_bag.py \
  --input "$BAG_ROOT/putnam/may_26/run_3/filtered/all" \
  --output "$DATA/run_3_prepped" \
  --utm-origin "$RUN5_ORIGIN"

scripts/validate_luminar_timestamps.py "$DATA/run_5_prepped" \
  --json-out "$DATA/run_5_prepped_timestamp_validation.json"
```

Pure-GICP deskew ablation:

```bash
GT_RECOVERY_ENABLED=false GT_REJECTION_ENABLED=false GT_VETO_ENABLED=false \
  DESKEW=false CROP_SIZE=1001.0 SENSOR_TYPE=luminar LIDAR_CONCAT_ENABLED=false \
  scripts/run_localization_replay.sh \
  dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s \
  dlio_data/run_5_map.pcd dlio_data/run_5_dump/T_world_utm.txt \
  "$DATA/gicp_highspeed_off" false

GT_RECOVERY_ENABLED=false GT_REJECTION_ENABLED=false GT_VETO_ENABLED=false \
  DESKEW=true CROP_SIZE=1001.0 SENSOR_TYPE=luminar LIDAR_CONCAT_ENABLED=false \
  scripts/run_localization_replay.sh \
  dlio_data/luminar_timestamp_validation/run5_corrected_highspeed_95s \
  dlio_data/run_5_map.pcd dlio_data/run_5_dump/T_world_utm.txt \
  "$DATA/gicp_highspeed_on" false
```

Live hardware completion step:

```bash
scripts/validate_luminar_timestamps_live.py \
  --duration 65 \
  --ptp-lock-confirmed \
  --json-out "$DATA/live_timestamp_check.json"
```

Only pass `--ptp-lock-confirmed` after confirming the receiver/sensor PTP lock
through the live status UI or status topic during that capture.

## Remaining Follow-Up

1. Run the live hardware validator once the physical Luminar/Atlas stack is
   publishing and PTP lock is confirmed.
2. Treat any prepared bags generated before the point-time repair as stale
   unless they pass `scripts/validate_luminar_timestamps.py` in this checkout.
