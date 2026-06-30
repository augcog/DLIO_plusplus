# Changes Based on the Latest Remote `origin/art-jazzy`

Comparison scope:

- Remote base ref: `refs/remotes/origin/art-jazzy`
- Remote base SHA: `aaea8f618cd8412a1f8e927228a32261a8da457e` (`harden lidar merge guard`)
- Head SHA: `dba3c4c09f95c1fbe7b87195f5044e58b377596f` (`deleted previous lidar contact`)
- Branch: `art-jazzy-prepmap`
- Merge-base: `aaea8f618cd8412a1f8e927228a32261a8da457e`
- Diff: 16 files changed, 2959 insertions, 32 deletions

This report answers one question: relative to the fetched latest remote
`origin/art-jazzy`, what changes were made, and why were they made?

## Summary

After the latest remote `origin/art-jazzy` commit, I made four main categories of
changes:

1. Added an offline raw bag -> normalized bag preparation path.
2. Added a Point One Atlas adapter that turns raw Atlas pose/IMU into the
   `/gps_p1/*` streams and optional `/gnss*` local-ENU inputs consumed by
   GLIM/GICP.
3. Adjusted the GICP strict LiDAR merge failure policy while keeping
   `require_all_aux=true`: degraded clouds missing required aux LiDARs are not
   registered, but the localization node is not killed immediately. Instead,
   the current scan is skipped and propagation continues.
4. Added replay/debug launch knobs to make offline validation and reproduction
   easier.

I did not modify the GLIM `harden lidar merge guard` implementation. GLIM's
strict multi-LiDAR merge guard still comes from the remote base commit. This
diff does not change `GLIM/glim_ros2/include/glim_ros/lidar_concat.hpp` or the
related GLIM entrypoints.

## 1. Added Offline Bag Preparation

New files:

- `scripts/prep_bag.py`
- `scripts/export_glim_dump_to_pcd.py`

What `prep_bag.py` does:

- Reads Atlas pose from the raw bag.
- Decodes IMU from the Point One PCAP and uses `IMU_OUTPUT.p1_time` as the IMU
  time source.
- Uses the adapter to generate `/gps_p1/imu`, `/gps_p1/filtered_odom`,
  `/gps_p1/filtered_odom_rtk_fixed`, and optionally `/gnss` and
  `/gnss_rtk_fixed`.
- Copies the raw bag's `/luminar_front/points`, `/luminar_left/points`, and
  `/luminar_right/points` into the final normalized bag.
- Checks topic counts, monotonic header stamps, time gaps, and nearest IMU/LiDAR
  coverage.
- Can optionally call `glim_rosbag` for offline GLIM.

Why:

- Atlas pose/IMU and LiDAR come from different sources in the raw bag/PCAP.
  `prep_bag.py` normalizes the small streams while keeping the raw LiDAR
  measurements unchanged.
- This gives GLIM/GICP a stable input contract: LiDAR topics remain raw
  measurements, while `/gps_p1/*` provides IMU/odom support in the same local
  ENU frame.

Impact on the LiDAR guard:

- `prep_bag.py` does not repair, rename, or restamp LiDAR point clouds.
- LiDAR merge and strict-failure behavior remain owned by GLIM/GICP
  `lidar_concat`.
- `prep_bag.py` only places raw LiDAR messages into the final bag so the guard
  has complete input to check.

## 2. Added the Atlas Adapter

New directory:

- `adapter/`

Current adapter responsibilities:

- Takes `/atlas/pose_filtered` and converts WGS84 LLA into the local ENU `map`
  frame.
- Publishes `/gps_p1/filtered_odom`.
- Publishes `/gps_p1/filtered_odom_rtk_fixed` after the Atlas covariance gate
  passes.
- Optionally publishes `/gnss` and `/gnss_rtk_fixed`.
- Takes Atlas IMU or PCAP replay IMU and publishes `/gps_p1/imu`.
- Supports either a fixed local ENU origin or a TTL-derived origin.

Why:

- GLIM/GICP should not consume raw Atlas FusionEngine pose directly. They should
  consume the repository's normalized `/gps_p1/*` and local ENU contract.
- The adapter keeps sensor-specific normalization at the boundary instead of
  putting Atlas-specific parsing, RTK covariance gating, and P1 time mapping
  into GLIM/GICP.
- Offline `prep_bag.py` and online/replay paths can reuse the same normalization
  logic.

Impact on the LiDAR guard:

- The adapter does not process LiDAR: it has no `PointCloud2`
  publisher/subscriber, no `/luminar_*` parameters, and no LiDAR timestamp
  repair.
- This keeps LiDAR ownership clear: the adapter handles Atlas pose/IMU, and
  GLIM/GICP handle the LiDAR merge guard.

## 3. Adjusted GICP Strict Merge Failure Policy

Modified files:

- `gicp_localization/src/localization.cc`
- `gicp_localization/include/gicp_localization/localization.h`
- `gicp_localization/cfg/localization.yaml`

What changed:

- `mergeAuxClouds()` returns `nullptr` on a strict-required merge failure.
- `callbackPointCloud()` skips the current scan when it receives `nullptr`, so
  the scan does not enter GICP registration.
- After more than `max_consecutive_aux_merge_failures`, the node emits a stronger
  throttled error, but continues running.
- Failure cases include missing XYZ fields on the primary cloud, non-tight or
  padded primary clouds, and missing or late required aux scans.
- `require_all_aux` remains enabled. This does not allow partial-LiDAR
  registration; it only changes the failure response from aborting the node to
  skipping that scan.

Why:

- The core guard requirement is that localization must not silently run on
  primary-only or partial-LiDAR clouds when required aux LiDARs are incomplete.
- The previous "10 consecutive failures then abort" behavior made GICP testing
  brittle: an initial warmup gap or short aux-sync dropout could terminate the
  localization node before the replay had a chance to continue, making the GICP
  test unusable rather than just rejecting degraded scans.
- The skip-scan policy keeps the guard's safety semantics: degraded clouds are
  not registered, while IMU/geometric propagation can continue until a complete
  merged scan arrives. This lets replay-based GICP tests continue far enough to
  evaluate recovery and later valid scans.

Reviewer point:

- Please confirm whether the GICP strict-guard policy should change from
  "stop the node after the 10-scan budget" to "continue skipping degraded scans
  after the budget without stopping the node".
- If the owner expects `harden lidar merge guard` to hard-fail, this part should
  be split out for separate discussion or changed back to an abort policy.

## 4. Added Launch/Debug Knobs

Modified files:

- `gicp_localization/launch/localization_with_tf.launch.py`
- `gicp_localization/cfg/localization.yaml`

New or forwarded parameters:

- `publish_tf`
- `deskew`
- `debug_pub`
- `verbose_scan_log`
- `verbose`

Why:

- Offline replay and GICP debugging often need to toggle TF publishing, deskew,
  debug topics, and log verbosity without editing YAML.
- These launch arguments let the same launch file override common debug options,
  reducing unreproducible temporary config edits.

Impact on the LiDAR guard:

- No change to the GLIM guard.
- For GICP, this only exposes existing runtime parameters at the launch layer so
  guard behavior is easier to reproduce.

## File-Level Summary

| Path | Change | Why |
|---|---|---|
| `adapter/*` | Added Atlas normalization package | Converts raw Atlas pose/IMU into the `/gps_p1/*` and local ENU contract consumed by GLIM/GICP |
| `adapter/scripts/p1_imu_pcap_replay_node.py` | Added PCAP IMU replay node | Uses Point One PCAP `IMU_OUTPUT.p1_time` to generate the correct IMU time axis |
| `scripts/prep_bag.py` | Added normalized bag pipeline | Makes raw bags reproducibly usable by GLIM/GICP: adapter small streams plus raw LiDAR copy |
| `scripts/export_glim_dump_to_pcd.py` | Added GLIM dump exporter | Produces a map PCD from a GLIM dump when the normal PCD export path is unavailable |
| `gicp_localization/src/localization.cc` | Changed strict merge failure to skip the scan | Avoids registering degraded LiDAR scans while preventing replay from stopping on a short aux dropout |
| `gicp_localization/cfg/localization.yaml` | Updated strict-guard comments/default semantics | Keeps config documentation aligned with skip-scan behavior |
| `gicp_localization/include/gicp_localization/localization.h` | Updated counter comments | Keeps code comments aligned with warning-budget semantics |
| `gicp_localization/launch/localization_with_tf.launch.py` | Added replay/debug launch knobs | Avoids editing YAML for common debug overrides |

## Verification

Ran:

```bash
git fetch origin art-jazzy:refs/remotes/origin/art-jazzy
git rev-parse refs/remotes/origin/art-jazzy
git log -1 --oneline refs/remotes/origin/art-jazzy
git merge-base refs/remotes/origin/art-jazzy HEAD
git diff --stat refs/remotes/origin/art-jazzy..HEAD
git diff --name-status refs/remotes/origin/art-jazzy..HEAD
git diff --check refs/remotes/origin/art-jazzy..HEAD
rg -n "enable_lidar_bridge|lidar_bridge|repairLuminar|repair_luminar|strict_luminar|lidar_input_|lidar_time_offset|PointCloud2|PointField|luminar_" adapter || true
source /opt/ros/jazzy/setup.bash
source /home/roar/Documents/race_common/install/setup.bash
colcon build --packages-select adapter --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon build --packages-select gicp_localization --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 -m py_compile scripts/prep_bag.py adapter/scripts/p1_imu_pcap_replay_node.py adapter/launch/adapter.launch.py
```

Results:

- Adapter LiDAR residual grep: no matches.
- `refs/remotes/origin/art-jazzy`: `aaea8f6 harden lidar merge guard`.
- Merge-base: `aaea8f618cd8412a1f8e927228a32261a8da457e`.
- Diff: 16 files changed, 2959 insertions, 32 deletions.
- `git diff --check`: passed.
- `colcon build --packages-select adapter`: passed.
- `colcon build --packages-select gicp_localization`: passed.
- Python compile: passed.
