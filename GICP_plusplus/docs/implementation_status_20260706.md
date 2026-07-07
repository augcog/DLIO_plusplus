# GICP++ Implementation Status — 2026-07-06

This folder is a parallel ROS 2 package named `gicp_plusplus`. It is intended
for A/B replay against the existing `gicp_localization` package; the NanoGICP
package was not edited.

## Implemented Phase 1

- Vendored MIT-licensed `small_gicp` headers under
  `thirdparty/small_gicp/`.
- Added `gicp_plusplus::SmallGicpBackend`, a narrow compatibility wrapper for
  the old matcher calls used by `LocalizationNode`.
- Replaced the copied matcher member with `SmallGicpBackend<PointType,
  PointType>`.
- Kept the existing ROS node architecture: IMU prior, scan-to-map registration,
  ratio/yaw/jump gates, observer, GT recovery, multi-LiDAR concat, debug topics,
  and scorecard schema.
- Implemented in-optimizer DoF restriction and soft IMU-attitude prior through a
  small_gicp general factor.
- Used a local prior-aware LM optimizer so general-factor prior error is included
  when evaluating lambda trial steps.

## Deliberate A/B Differences

- `getFitnessScore()` now reports `small_gicp` final error divided by final
  inliers. Absolute fitness thresholds must be re-baselined; ratio gates are the
  preferred comparison signal.
- Source covariances and kd-tree are rebuilt on every scan. Target covariances
  are computed once after map load.
- The debug parameter is now `localization/debug/small_gicp_lm_debug`; the old
  `nano_gicp_lm_debug` name remains as a compatibility alias.

## Not Yet Implemented

- VGICP/GaussianVoxelMap runtime map backend.
- Phase 2 fixed-lag GTSAM/gtsam_points localizer.
- Replay threshold re-baselining and scorecard acceptance gates for the new
  small_gicp fitness scale.
