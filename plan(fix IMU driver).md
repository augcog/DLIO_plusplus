# IMU Driver Fix and Validation Plan v2

## Summary
- Fix the IMU burst problem at the source: `race_common` `fusion_engine_driver` must stop stamping `/atlas/imu_calibrated` with arrival time.
- Use FusionEngine `IMU_OUTPUT.p1_time` as the sample-time source, mapped onto the ROS/LiDAR time axis before publishing.
- Do **not** hard-code 100 Hz or 120 Hz in the driver. Current Putnam rosbag data behaves like ~100 Hz / ~10 ms, but the fixed driver should trust actual adjacent P1 sample times.
- Existing bad ROS bags cannot be perfectly repaired unless raw FusionEngine payload/PCAP is available; legacy bags still need adapter/prep retiming.

## Key Changes
- Extend `fusion_engine_driver` IMU stamping:
  - Keep `imu_output_stamp_source: "arrival"` for backward compatibility.
  - Add/use `imu_output_stamp_source: "p1_time_mapped"` for Atlas live runs.
  - Optional debug mode: `p1_time_raw`, only for inspection, not normal ROS playback.
- Implement `p1_time_mapped` as:
  - `mapped_stamp = p1_time + offset`
  - `offset` is estimated online from the lower envelope of `(arrival_ros - p1_time)` using Pose/IMU FusionEngine messages.
  - Output stamps must be monotonic; drop or fallback with warning if invalid or non-increasing.
- Update IAC Atlas config:
  - Set `src/launch/iac_launch/param/gps_param/atlas/atlas.param.yaml`:
    ```yaml
    imu_output_stamp_source: "p1_time_mapped"
    ```
- Fix `pointonenav_interface` IMU dt handling:
  - First IMU message must not compute dt against `this->now()`.
  - Reject or safely handle `dt <= 0`, NaN, or Inf.
  - `/gps_p1/imu` should preserve the corrected `/atlas/imu_calibrated.header.stamp`.

## Test Plan
- Unit tests:
  - Simulate 100 Hz P1 IMU samples with bursty arrival times; published dt should remain ~10 ms, not burst-shaped.
  - Simulate non-100 Hz samples; driver should follow P1 deltas, proving no hard-coded rate.
  - Invalid P1 timestamp falls back or drops with throttled warning.
  - `pointonenav_interface` produces finite covariance and no negative/zero dt failure.
- Build tests:
  - `colcon build --packages-select fusion_engine_driver pointonenav_interface iac_launch`
  - `colcon test --packages-select fusion_engine_driver pointonenav_interface`
- Offline raw-data test, if PCAP/raw FusionEngine stream exists:
  - Re-decode with fixed driver.
  - Verify `/atlas/imu_calibrated` and `/gps_p1/imu` median dt ≈ 10 ms for current Putnam data.
  - Verify no near-zero burst gaps followed by large flush gaps.
- Live hardware test:
  - Run `ros2 launch iac_launch atlas.launch.py`.
  - Record 5-10 minutes of `/atlas/imu_calibrated`, `/atlas/pose_filtered`, `/gps_p1/imu`, `/gps_p1/filtered_odom`, and Luminar topics.
  - Acceptance: monotonic IMU stamps, median dt near actual device rate, no burst-shaped timestamp distribution, no covariance/dt warnings.
- DLIO/GICP end-to-end:
  - Use a newly recorded fixed-driver bag.
  - Run mapping/localization without `prep_bag` IMU retiming.
  - GICP odom should publish at roughly IMU rate, and GLIM/GICP should not fail due to IMU burst dt.

## Assumptions
- Current Putnam ROS bag IMU effective rate is ~100 Hz / ~10 ms.
- `120 Hz` in `pointonenav_interface.param.yaml` is only a filter configuration assumption and must not drive timestamp reconstruction.
- Existing bad ROS bags lack true IMU P1 sample time in `sensor_msgs/Imu`, so exact recovery is impossible from those topics alone.
- UDP is not part of this fix; current `udp` config path still uses `TcpListener`.
