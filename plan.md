# Live DLIO Input Adapter Plan

## Summary
- 新增 C++ `dlio_input_adapter`，作为 raw sensor/rosbag 与 GLIM/GICP 之间的唯一输入归一化层。
- 支持两种输入源，输出完全一致：
  - live hardware drivers: race_common `fusion_engine_driver` + Luminar driver
  - raw rosbag replay: `/atlas/*` + `/luminar_*`
- GLIM/GICP 保持现有 normalized 接口：`/gps_p1/imu`、`/gps_p1/filtered_odom*`、`/luminar_*`，不直接订阅 `/atlas/*`。
- `prep_bag.py` 保留为离线缓存/debug 工具；live path 不再依赖 bag rewrite。

## Key Changes
- **Workspace dependency**
  - DLIO_plusplus build 前必须 source/build race_common selected packages：`fusion_engine_msgs`、`fusion_engine_driver`、`pointonenav_interface`。
  - DLIO_plusplus 增加 env/doc/build checks：source 顺序固定为 `/opt/ros/jazzy` → `race_common/install/setup.bash` → `DLIO_plusplus/install/setup.bash`。
  - `make check-env` 验证 `ros2 pkg prefix fusion_engine_msgs fusion_engine_driver pointonenav_interface` 指向 race_common overlay。

- **race_common driver timestamp fix**
  - 在 `fusion_engine_driver` 增加参数 `imu_output_stamp_source:=arrival|p1_time`，默认 `arrival`，DLIO live launch 显式使用 `p1_time`。
  - `p1_time` 模式下，`/atlas/imu_calibrated.header.stamp` 写 FusionEngine `IMUOutput.p1_time`，即设备采样时刻。
  - adapter 不把裸 P1 monotonic time 直接给 GLIM/GICP；它会用 Pose stream 的 `(arrival_ros, p1_time)` 建立 `p1_time -> ROS epoch` 映射，再发布 `/gps_p1/imu`。

- **new package: `dlio_input_adapter`**
  - C++ `rclcpp` package，依赖 `fusion_engine_msgs`、`sensor_msgs`、`nav_msgs`、`tf2`、`GeographicLib` 或 PROJ。
  - 默认输入：
    - `/atlas/pose_filtered`
    - `/atlas/imu_calibrated`
    - `/dlio_raw/luminar_front/points`
    - `/dlio_raw/luminar_left/points`
    - `/dlio_raw/luminar_right/points`
  - 默认输出：
    - `/gps_p1/imu`
    - `/gps_p1/filtered_odom`
    - `/gps_p1/filtered_odom_rtk_fixed`
    - `/luminar_front/points`
    - `/luminar_left/points`
    - `/luminar_right/points`
    - optional `/gps_p1/filtered_odom_map`

- **launch topology**
  - Rosbag mode:
    - `ros2 bag play raw_bag --remap /luminar_front/points:=/dlio_raw/luminar_front/points ...`
    - launch `dlio_input_adapter use_sim_time:=true`
    - then launch GLIM or GICP unchanged.
  - Hardware mode:
    - launch race_common Atlas + Luminar drivers.
    - remap raw Luminar driver outputs into `/dlio_raw/luminar_*`.
    - launch `dlio_input_adapter use_sim_time:=false`.
    - then launch GLIM/GICP unchanged.
  - Never subscribe and publish the same `/luminar_*` topic in adapter; always use `/dlio_raw/*` input namespace to prevent loops.

## Adapter Behavior
- **P1 clock mapper**
  - Maintain rolling lower-envelope offset from `/atlas/pose_filtered`: `ros_valid_time = p1_time + offset(p1_time)`.
  - Use pose `header.stamp` only as arrival upper bound; use pose `p1_time` as validity time.
  - Clamp all published stamps monotonic per output topic.
  - Publish diagnostics: current offset, drift, delayed samples, retime mode.

- **IMU**
  - `imu_stamp_mode=auto|p1|arrival_retime`.
  - `auto` detects whether `/atlas/imu_calibrated.header.stamp` is P1-like or ROS-epoch-like.
  - P1 path: convert IMU stamp via `P1ClockMapper`.
  - Legacy raw bag path: if IMU is arrival-stamped, use bounded lookahead burst retimer equivalent to `prep_bag.py` backward-min logic.
  - Output `/gps_p1/imu` with `frame_id="gps_antenna_top"` and ROS epoch stamp aligned with LiDAR/odom.

- **Pose/Odom**
  - Convert `/atlas/pose_filtered` to `/gps_p1/filtered_odom`.
  - Use `pose.p1_time` through `P1ClockMapper`.
  - Convert LLA to fixed UTM zone, subtract fixed origin.
  - Origin policy:
    - mapping default: first valid fix rounded down to 10 km grid.
    - localization default: load explicit origin from map/run config.
  - Output `header.frame_id="utm"` and `child_frame_id="gps_antenna_top"`.
  - Preserve pose covariance, rpy covariance, body-FLU velocity, and velocity covariance semantics matching `prep_bag.py`.

- **RTK gate**
  - Publish `/gps_p1/filtered_odom_rtk_fixed` only when:
    - `solution_type == 4`
    - `cov_xx <= 1e-3 m^2`
    - `cov_yy <= 1e-3 m^2`
    - `cov_zz <= 5e-3 m^2`
  - Parameters expose these thresholds.
  - GLIM mapping uses this RTK-fixed topic exactly as today.

- **Luminar**
  - Pass through scan `header.stamp` unless `lidar_time_offset` is configured.
  - Repair `UINT8[8] timestamp` field so `min(point_time) == header.stamp` on ROS epoch, preserving scan internal span.
  - Validate schema at startup/first frame: field name `timestamp`, datatype `UINT8`, count `8`, `point_step=56`; warn/fail based on `strict_luminar_schema`.
  - Support all three sensors and preserve frame IDs.

- **Map-frame odom for GICP**
  - If `T_world_utm_path` is set, adapter also publishes `/gps_p1/filtered_odom_map`.
  - This is the C++ live replacement for `gicp_localization/scripts/utm_to_map_odom.py`.
  - Mapping mode leaves `T_world_utm_path` empty; localization mode requires it.

## GLIM/GICP Compatibility
- **GLIM mapping**
  - Existing `config_ros.json` remains compatible:
    - `/gps_p1/imu`
    - `/luminar_front/points`
    - `/gps_p1/filtered_odom_rtk_fixed`
  - GNSS global factors and INS-driven odometry continue to use RTK-fixed odom.
  - Adapter must produce enough RTK-fixed samples before driving/mapping; otherwise GLIM will skip/pause as it does today.

- **GICP localization**
  - Existing launch defaults remain compatible:
    - pointcloud `/luminar_front/points`
    - IMU `/gps_p1/imu`
    - GT odom `/gps_p1/filtered_odom_map`
  - RViz, live error monitor, cached error viz, eval bags, and replay scripts continue to consume existing GICP topics.
  - LiDAR concat remains supported because adapter republishes all three normalized Luminar topics.

- **Offline parity**
  - For raw bag replay, adapter output must be behaviorally equivalent to `prep_bag.py` output.
  - `prep_bag.py` remains available for cached long runs, but no pipeline functionality should require prepped bags after adapter is implemented.

## Implementation Steps
- **Phase 1: race_common timestamp source**
  - Add `imu_output_stamp_source` parameter to `fusion_engine_driver`.
  - For `IMU_OUTPUT`, if source is `p1_time`, stamp with payload `IMUOutput.p1_time`; otherwise keep `this->now()`.
  - Add unit/smoke check using a small FusionEngine replay or raw bag where possible.

- **Phase 2: adapter package skeleton**
  - Create `dlio_input_adapter` with executable `dlio_input_adapter_node`.
  - Add launch file `dlio_input_adapter.launch.py`.
  - Add params file with topic names, UTM origin, RTK thresholds, Luminar repair flags, `T_world_utm_path`, and mode selection.
  - Add `dlio` metapackage dependency on `dlio_input_adapter`.

- **Phase 3: time + IMU**
  - Implement `P1ClockMapper`.
  - Implement IMU p1-stamp conversion and legacy arrival-stamp fallback.
  - Add diagnostics and monotonicity guards.

- **Phase 4: odom + RTK**
  - Implement Pose→UTM Odometry conversion matching `prep_bag.py`.
  - Implement fixed origin handling and RTK-fixed republisher.
  - Add optional `utm_origin.txt` writer for mapping runs.

- **Phase 5: Luminar**
  - Implement PointCloud2 timestamp schema detection.
  - Implement point timestamp epoch repair.
  - Add input namespace remap support for three Luminar topics.

- **Phase 6: GICP map-frame bridge**
  - Implement `T_world_utm.txt` loader.
  - Publish `/gps_p1/filtered_odom_map` when configured.
  - Match Python `utm_to_map_odom.py` output semantics.

- **Phase 7: docs/scripts**
  - Update `PIPELINE.md` with live adapter path and raw-bag replay path.
  - Update `dlio.env.example` with race_common setup and adapter params.
  - Update wrapper scripts to offer `--live-adapter` / `--raw-live` mode without removing existing `--prepped` path.

## Test Plan
- **Build**
  - Build race_common selected packages.
  - Build DLIO with `make build-all`.
  - Verify package prefixes for race_common and DLIO packages.

- **Unit tests**
  - P1 clock mapper lower-envelope and drift.
  - IMU p1 mode and arrival burst fallback.
  - UTM conversion/origin parity with `prep_bag.py`.
  - RTK gate thresholds.
  - Luminar `UINT8[8]` timestamp repair.
  - `T_world_utm` transform loader and odom transform.

- **Raw bag equivalence**
  - Run adapter on run_5 raw bag and record normalized output.
  - Compare against `prep_bag.py` output:
    - topic counts
    - IMU dt distribution
    - odom stamps/positions
    - RTK-fixed count
    - Luminar point timestamp span and epoch
  - Repeat for run_3.

- **Pipeline smoke**
  - run_5 mapping from raw bag through adapter: GLIM produces dump, PCD, renders, and `T_world_utm.txt`.
  - run_3 GICP localization from raw bag through adapter with RViz: produces odom, `live_error.csv`, `live_error.png`, eval bag, and eval stats.
  - GUI smoke starts and exits automatically.

- **Hardware smoke**
  - With drivers live, verify:
    - `/atlas/imu_calibrated` has P1-mode stamps when enabled.
    - `/gps_p1/imu` has ROS epoch stamps and stable sample interval.
    - `/gps_p1/filtered_odom` tracks pose stream with no timestamp reversals.
    - `/luminar_*` output point timestamps are on ROS epoch.
    - GLIM/GICP can subscribe without remap changes.

## Assumptions
- We may modify both `/home/roar/Documents/race_common` and `/home/roar/Documents/DLIO_plusplus`.
- Public GLIM/GICP topic contracts stay normalized; raw `/atlas/*` support lives in adapter, not inside algorithm nodes.
- Existing race_common `pointonenav_interface` is a reference and dependency, not the final DLIO adapter, because its coordinate/time semantics do not match DLIO pipeline requirements.
- Existing `prep_bag.py` behavior is the reference for raw-bag equivalence.

# ☱Live Adapter Pass/Fail Test Definition

## ☱Summary
- `run_5` 用作建图输入，具体 raw bag 是 `/media/roar/data/rosbags/putnam/may_26/run_5/filtered/all`。
- `run_3` 用作定位输入，具体 raw bag 是 `/media/roar/data/rosbags/putnam/may_26/run_3/filtered/all`。
- “跑通”定义为：raw bag 不经 `prep_bag.py`，通过 live adapter 后能完成 GLIM 建图、导出 map，并用该 map 完成 GICP 定位和误差评估。

## ☱Test Stages
- **Stage 0: Preflight**
  - `ros2 bag info` 确认 raw bag 有：
    - `/atlas/imu_calibrated`
    - `/atlas/pose_filtered`
    - `/luminar_front/points`
    - `/luminar_left/points`
    - `/luminar_right/points`
  - `ros2 pkg prefix` 确认 race_common 和 DLIO overlay 都解析正确。
  - `make build-all` 成功。

- **Stage 1: Adapter Output Contract**
  - 输入：
    - `run_5`: `/media/roar/data/rosbags/putnam/may_26/run_5/filtered/all`
    - `run_3`: `/media/roar/data/rosbags/putnam/may_26/run_3/filtered/all`
  - 启动 raw bag replay + `dlio_input_adapter`，record adapter 输出。
  - 必须产出：
    - `/gps_p1/imu`
    - `/gps_p1/filtered_odom`
    - `/gps_p1/filtered_odom_rtk_fixed`
    - `/luminar_front/points`
    - `/luminar_left/points`
    - `/luminar_right/points`
  - Pass 条件：
    - `/gps_p1/imu` count == raw `/atlas/imu_calibrated` count。
    - `/gps_p1/imu.header.stamp` 严格单调。
    - IMU dt 不再是 raw burst arrival 分布，而是接近 fixed sample period。
    - `/gps_p1/filtered_odom.header.stamp` 单调，使用 P1 validity time 映射到 ROS epoch。
    - `/gps_p1/filtered_odom_rtk_fixed` 非空，且只包含 RTK-fixed/covariance gate 通过样本。
    - Luminar 三路输出 count == raw 输入 count。
    - Luminar point timestamp 修复后 `min(point_time)` 与 `header.stamp` 同 epoch，scan span 保持不塌缩。

- **Stage 2: Reference Equivalence**
  - 对同一个 raw bag 同时跑一次 `prep_bag.py` reference。
  - Adapter 输出必须和 reference 在关键指标上对齐：
    - topic count 一致或差异有明确解释。
    - IMU period/run duration 接近。
    - odom trajectory duration 接近。
    - RTK-fixed sample count 接近。
    - Luminar point timestamp span 接近。
  - 这个 stage 的目的不是 bit-exact，而是证明 live adapter 复刻了 `prep_bag.py` 对算法有影响的语义。

- **Stage 3: GLIM Mapping Pass**
  - 输入：`run_5` raw bag。
  - 禁止使用 prepped bag。
  - 数据流：
    - `run_5 raw bag` → `dlio_input_adapter` → GLIM。
  - Pass 条件：
    - GLIM 正常结束或达到 pipeline 预期结束。
    - 生成 non-empty dump directory。
    - 生成 `T_world_utm.txt`。
    - 生成 non-empty PCD。
    - 生成 trajectory dump。
    - `render_map.py` 能输出 PNG。
    - `eval_traj_vs_gnss.py` 能输出 stats。
  - Fail 条件：
    - GLIM 因 missing IMU/INS coverage 大量跳帧。
    - time sync fatal。
    - no RTK-fixed odom。
    - PCD 空。
    - 没有 `T_world_utm.txt`。

- **Stage 4: GICP Localization Pass**
  - 输入：`run_3` raw bag。
  - 地图：Stage 3 由 `run_5` 生成的 map/dump。
  - 禁止使用 prepped bag。
  - 数据流：
    - `run_3 raw bag` → `dlio_input_adapter` with `T_world_utm.txt` → GICP。
  - Pass 条件：
    - GICP 启动并订阅 adapter 输出。
    - `/gps_p1/filtered_odom_map` 正常发布。
    - GICP odom topic 正常发布，duration 覆盖主要 replay 区间。
    - RViz smoke 启动成功。
    - 生成 `live_error.csv`。
    - 生成 `live_error.png`。
    - 生成 recorded eval bag。
    - `eval_odom_vs_gt.py` 输出 stats。
  - Fail 条件：
    - IMU topic/frame 被 GICP 拒绝。
    - GT odom frame 和 map frame 不一致。
    - GICP 没有持续 odom 输出。
    - eval 脚本无法匹配 odom/GT。
    - RViz/launch fatal。

## ☱Definition of Done
- `run_5` raw bag 能通过 adapter 完成 GLIM 建图并导出可用 map。
- `run_3` raw bag 能通过 adapter 在 `run_5` map 上完成 GICP 定位。
- 全流程没有调用 `prep_bag.py` 作为输入生成步骤；`prep_bag.py` 只用于 reference comparison。
- 测试报告记录每个 stage 的：
  - command
  - exit status
  - runtime
  - log path
  - artifact path
  - pass/fail reason
- 所有巨量临时数据写到 `/media/roar/data/dlio_live_adapter_test_${TEST_ID}`，最终按用户要求保留或清理。
