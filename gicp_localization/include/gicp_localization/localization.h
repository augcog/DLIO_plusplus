#ifndef GICP_LOCALIZATION_H
#define GICP_LOCALIZATION_H

// DLIO types (PointType is a global typedef, not in dlio namespace)
#include "dlio/dlio.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// BOOST
#include <boost/circular_buffer.hpp>
#include <deque>

// STL
#include <atomic>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace gicp_localization {

// PointType is already defined globally by dlio.h

class LocalizationNode : public rclcpp::Node {

public:

  // IMU measurement structure (needs to be public for function signatures)
  struct ImuMeas {
    double stamp;
    double dt;
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  };

  // Ground-truth odom sample (public so internal helper signatures can reference it).
  // p/q are header.frame_id=map; v_lin_body / v_ang_body are in child_frame_id (gt_body).
  // cov_pos_{xx,yy,zz} are the diagonal position-variance terms from
  // pose.covariance[0,7,14] -- carried per-sample so consumers can decide whether
  // the sample is RTK-FIXED quality (init / calibration / cross-check) or merely
  // Atlas's INS-dead-reckoning quality (snap-recovery accepts either).
  struct GtSample {
    double stamp;
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    Eigen::Vector3f v_lin_body;
    Eigen::Vector3f v_ang_body;
    // Default to +inf so any sample that reaches the RTK gate without having
    // its covariance explicitly populated is treated as NOT RTK-FIXED (the
    // safe direction) rather than reading an uninitialized value. Real
    // samples overwrite these in callbackGtOdom; interpolated samples in
    // getGtPoseAt() carry the conservative max of the bracketing samples.
    double cov_pos_xx = std::numeric_limits<double>::infinity();
    double cov_pos_yy = std::numeric_limits<double>::infinity();
    double cov_pos_zz = std::numeric_limits<double>::infinity();
  };

  LocalizationNode();
  ~LocalizationNode();

  void start();

private:

  void getParams();
  bool loadMap();

  void callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc);
  void callbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose);
  void callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu);
  void callbackGtOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  // Returns true if a GT sample within gt_odom_max_dt_ of `stamp` was found and interpolated into out.
  bool getGtPoseAt(double stamp, GtSample& out);
  // P2#2: world-frame (map) velocity of the gt_body origin by central finite
  // difference of the GT poses bracketing `stamp`. Used by the snap helper
  // when the GT odom's linear twist is unpopulated; returns ~0 at standstill,
  // so it covers both the missing-twist and truly-stationary cases. False when
  // fewer than 2 samples bracket the stamp within gt_odom_max_dt_.
  bool getGtFiniteDiffVelWorld(double stamp, Eigen::Vector3f& v_world_out);
  // Compose T_map_base = T_map_gtbody * inv(T_base_gtbody) using the cached
  // gt_body -> base extrinsic, bringing the GT sample's pose from
  // msg.child_frame_id into base_frame coordinates.  The snap helper, the
  // diagnostic cross-check (gt_pos_err_m, gt_rot_err_deg), and the
  // first-message odom-init path all use this to ensure they operate in
  // the same body reference as state.p / current_pose. Returns false (and
  // leaves p_out / q_out unmodified) if the extrinsic has not been cached
  // yet; for the AV-24 single-source NA config, gt_body_frame == base_frame
  // so the extrinsic is cached as identity on the first GT message and the
  // composition is a no-op (gt_p_in_base == gt.p, gt_q_in_base == gt.q).
  bool composeGtPoseInBase(const GtSample& gt, Eigen::Vector3f& p_out,
                           Eigen::Quaternionf& q_out) const;
  // Compose GT twist from gt_body into base_frame using the cached
  // T_base_gtbody_ extrinsic. Returns false when gt extrinsics are unavailable.
  bool composeGtTwistInBase(const GtSample& gt, Eigen::Vector3f& v_lin_body_out,
                            Eigen::Vector3f& v_ang_body_out) const;
  // GT-driven pose recovery. Returns true when the snap fired (guards passed and
  // a time-matched GT sample with finite extrinsic was applied to the state).
  bool maybeSnapPoseToGT(const char* reason);
  void applyInitialPose(const Eigen::Vector3f& p, const Eigen::Quaternionf& q,
                        const rclcpp::Time& stamp, const std::string& source);
  // RTK-driven calibration: accumulate one residual sample if a time-matched GT
  // exists at `stamp`. Returns true if the calibration window has filled and
  // biases were applied (caller should mark imu_calibrated_).
  bool tryRtkCalibrationStep(double stamp, const Eigen::Vector3f& measured_gyro,
                             const Eigen::Vector3f& measured_accel);

  // Is the GT sample RTK-FIXED quality? Tests Atlas-reported pose covariance
  // against rtk_gate_max_pose_var_xy_ / rtk_gate_max_pose_var_z_. Used by
  // consumers (init/calibration/cross-check) that need cm-level truth.
  // maybeSnapPoseToGT does NOT call this -- it accepts any sample because
  // Atlas's INS dead-reckoning is the next-best fallback to GICP failure.
  bool gtSampleIsRtkFixed(const GtSample& s) const;

  struct MotionConsistencyResult {
    bool evaluated = false;
    bool rejected = false;
    std::string reason = "not_evaluated";
    double dt = -1.0;
    double ds_xy = -1.0;
    double speed_mps = -1.0;
    double signed_forward_m = std::numeric_limits<double>::quiet_NaN();
    double heading_error_deg = std::numeric_limits<double>::quiet_NaN();
    double curvature_1pm = std::numeric_limits<double>::quiet_NaN();
    double yaw_rate_deg_s = std::numeric_limits<double>::quiet_NaN();
    double dyaw_deg = std::numeric_limits<double>::quiet_NaN();
  };
  MotionConsistencyResult evaluateMotionConsistency(
      const Eigen::Matrix4f& reference_pose,
      const Eigen::Matrix4f& candidate_pose,
      double dt) const;
  // Caller must hold pose_mutex. The reference is advanced only by known-good
  // seeds: initial pose, accepted GICP, or GT snap.
  void updateMotionReferenceUnlocked(const Eigen::Matrix4f& pose, double stamp_sec);

  void preprocessPointCloud(pcl::PointCloud<PointType>::Ptr& cloud);
  // Sensor-frame crop box; must run BEFORE deskew (world-frame transform).
  void cropBoxFilterSensorFrame(pcl::PointCloud<PointType>::Ptr& cloud);
  void deskewPointcloud();
  void performLocalization();
  void publishPose();
  void applyInitialPoseFromParams();
  bool loadUTMTransform(const std::string& path);

  // Multi-LiDAR concatenation: pushes incoming aux scans into per-sensor ring
  // buffers, then `mergeAuxClouds` (called from the primary callback) finds
  // the nearest aux scan per sensor, transforms its XYZ into the primary
  // sensor frame, rebases per-point timestamps onto the primary clock, and
  // appends the bytes to a copy of the primary PointCloud2.
  void callbackAuxPointCloud(int aux_index, sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  sensor_msgs::msg::PointCloud2::ConstSharedPtr mergeAuxClouds(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary);

  // Geometric Observer functions
  void propagateState();
  void updateState();

  // IMU integration functions
  bool imuMeasFromTimeRange(double start_time, double end_time,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub;
  rclcpp::CallbackGroup::SharedPtr pointcloud_cb_group, initial_pose_cb_group,
      imu_cb_group, gt_odom_cb_group;

  // Ground-truth odom for divergence cross-check (optional)
  // Latest message and a small ring buffer for time-matched lookup.
  bool gt_odom_enabled_;
  size_t gt_odom_buffer_size_;
  double gt_odom_max_dt_;  // seconds; reject lookups farther than this from scan stamp
  std::deque<GtSample> gt_odom_buffer_;
  std::mutex gt_odom_mtx_;
  std::atomic<bool> gt_odom_received_{false};

  // RTK quality gate for the gt_odom buffer (P1-native). Drops samples whose
  // Atlas-reported pose covariance (pose.covariance[0,7,14] -- xx, yy, zz)
  // exceeds the configured thresholds. The gate inspects the gt_odom message
  // itself; no separate status topic is involved. Replaces the old
  // BESTGNSSPOS-enum gate (removed when the NovAtel path was retired).
  bool rtk_gate_enabled_;
  double rtk_gate_max_pose_var_xy_;  // m^2; reject if cov[0] or cov[7] > this
  double rtk_gate_max_pose_var_z_;   // m^2; reject if cov[14] > this
  // Counter for rate-limited rejection logging.
  std::atomic<uint64_t> rtk_rejected_covariance_{0};

  // GT-driven pose recovery (optional). Mirrors the IMU extrinsic caching pattern
  // in callbackImu: on first GT message we record child_frame_id and look up the
  // base_frame ← gt_body TF once. Snap composes T_map_base = T_map_gtbody * inv(T_base_gtbody).
  bool gt_recovery_enabled_;
  int gt_recovery_min_consecutive_failures_;
  int consecutive_failures_;          // resets to 0 on accept; increments on any non-accept
  bool gt_extrinsics_cached_;
  Eigen::Matrix4f T_base_gtbody_;     // pose of gt_body expressed in base_frame
  std::string gt_body_frame_;          // captured from msg->child_frame_id

  // Multi-LiDAR concatenation
  struct AuxLidar {
    std::string topic;
    std::string frame;                          // header.frame_id of the aux sensor (URDF link)
    Eigen::Matrix4f T_primary_aux;              // p_primary = T * p_aux
    bool extrinsic_cached;                       // true once T_primary_aux is resolved
    std::string extrinsic_source = "tf";        // "urdf" | "static" | "tf" (for logging)
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> buffer;
    std::mutex mtx;
    // P4#3: signed header-time offset stats vs the primary (aux - primary),
    // accumulated over MERGED scans only (scan-callback thread). A stable
    // nonzero mean is the signature of a constant per-aux clock offset vs the
    // P1 timebase — actionable via a per-aux time-offset correction upstream.
    double dt_sum = 0.0;
    double dt_min = std::numeric_limits<double>::infinity();
    double dt_max = -std::numeric_limits<double>::infinity();
    uint64_t dt_count = 0;
  };
  std::vector<std::unique_ptr<AuxLidar>> aux_lidars_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> aux_subs_;
  rclcpp::CallbackGroup::SharedPtr aux_cb_group_;
  bool concat_enabled_;
  double concat_time_threshold_;
  size_t concat_buffer_size_;
  // Offline aux-extrinsic resolution (no live TF needed). Resolved once at
  // startup: URDF (concat_urdf_path_ + concat_primary_frame_) takes priority,
  // then a static per-aux matrix from yaml, then live TF as a last resort.
  std::string concat_primary_frame_;            // URDF link name of the primary LiDAR
  std::string concat_urdf_path_;                // path to av24.urdf ("" = skip URDF)
  // Strict merge guard: when a required multi-LiDAR merge stays incomplete.
  bool concat_require_all_aux_ = false;         // false = localize on available LiDARs; true = skip incomplete scans
  bool concat_abort_on_merge_failure_ = true;   // true = abort node past budget; false = keep skipping non-fatally
  int concat_max_consec_fail_ = 10;             // tolerated consecutive incomplete merges (0 = immediate)
  int concat_consec_fail_ = 0;                  // running counter of consecutive incomplete merges

  // P4#3: per-frame lidar_concat diagnostics (scan-callback thread only).
  // Refreshed by mergeAuxClouds(), published one-sample-per-frame from
  // performLocalization() so the run-report audit gets a per-frame source-set
  // record (the run-12 audit explicitly could not reconstruct this).
  int concat_last_merged_aux_ = -1;             // -1 = concat disabled / not run this frame
  std::vector<double> concat_last_aux_dt_;      // s, aux header - primary header; NaN = not merged
  std::vector<int> concat_last_aux_points_;     // appended points; 0 = not merged
  std::vector<double> concat_aux_time_offsets_; // P3 fix: constant per-aux clock offset (s), order = aux_topics
  double last_scan_time_span_s_ = -1.0;         // merged-scan per-point time span (deskew path)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_merged_aux_count_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_scan_time_span_pub;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> dbg_aux_dt_pubs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> dbg_aux_points_pubs_;
  // Resolve every aux's T_primary_aux without live TF; returns the count resolved.
  void resolveAuxExtrinsicsOffline(const std::vector<std::vector<double>>& static_transforms);

  // Offline base_frame<-lidar_frame lever arm (no live TF): URDF (lidar_concat/
  // urdf_path) then a static yaml matrix. Sets extrinsics.baselink2lidar* and
  // returns true on success; false leaves the caller to fall back to live TF.
  std::vector<double> base_lidar_static_;       // row-major 4x4, "" = unset
  bool resolveBaseLidarExtrinsicOffline(const std::string& lidar_frame);

  // Luminar multi-LiDAR deskew anchor. mergeAuxClouds() captures the PRIMARY
  // scan's earliest per-point timestamp BEFORE appending aux clouds; the deskew
  // LUMINAR branch anchors merged-sweep timing on this instead of the global
  // merged minimum, so an aux scan that began before the primary does not shift
  // the whole sweep late. Reset (valid=false) each scan; only set on the concat
  // path. See deskewPointcloud().
  uint64_t luminar_primary_min_ts_ns_ = 0;
  bool luminar_primary_min_ts_valid_ = false;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localized_odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr utm_odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr utm_path_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dbg_initial_guess_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dbg_final_pose_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dbg_pose_markers_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_fitness_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_gicp_elapsed_ms_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_corr_norm_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_scan_dt_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_imu_age_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_num_correspondences_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_correspondence_ratio_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_final_error_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_guess_to_solution_trans_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_guess_to_solution_rot_deg_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_guess_from_last_trans_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_guess_from_last_rot_deg_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_raw_points_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_preprocessed_points_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_imu_buffer_span_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_scan_to_latest_imu_lag_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_hessian_condition_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_jump_trans_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_jump_rot_deg_pub;
  // P1 gating rework diagnostics (one sample per processed frame, like the rest
  // of the debug/* family, so bag audits keep a single denominator).
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_fitness_ratio_pub;      // fitness / rolling-median baseline (-1 while warming up)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_degen_rot_axes_pub;     // # rotation eigen-axes zeroed by partial update (0-3)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_degen_trans_axes_pub;   // # translation eigen-axes zeroed (0-3)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_yaw_veto_pub;           // 1.0 when the yaw-consistency veto zeroed the yaw correction
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_yaw_innovation_pub;      // raw GICP-vs-IMU yaw disagreement (deg, pre-veto)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_yaw_stiffness_pub;        // marginal yaw information of the scan (Schur, re-centered)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_motion_forward_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_motion_heading_error_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_motion_curvature_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_motion_yaw_rate_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_motion_rejected_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dbg_converged_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_gt_pos_err_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_gt_rot_deg_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_snap_pub;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // Map
  pcl::PointCloud<PointType>::Ptr map_cloud;
  pcl::PointCloud<PointType>::Ptr map_cloud_ds; // downsampled for visualization
  std::shared_ptr<const nano_gicp::CovarianceList> map_normals;

  // Current scan
  pcl::PointCloud<PointType>::Ptr current_scan;
  pcl::PointCloud<PointType>::Ptr original_scan;
  rclcpp::Time scan_stamp;
  double prev_scan_stamp;
  double observer_dt_;
  std::string last_scan_input_frame_;
  size_t last_raw_point_count_;
  size_t last_preprocessed_point_count_;

  // GICP matcher
  nano_gicp::NanoGICP<PointType, PointType> gicp;

  // Current pose estimate
  Eigen::Matrix4f current_pose;
  Eigen::Matrix4f T_prior;  // IMU-based prior transformation
  std::atomic<bool> initialized;
  std::mutex pose_mutex;

  // P3: scan-time IMU prior paired with the basePose measurement, captured on
  // scan acceptance. updateState() uses it to form the time-free world-frame
  // delta T_corr = T_meas * inv(T_prior) and applies THAT to the current
  // state, instead of dragging the (already-advanced) state back toward the
  // 0.1-0.3 s-old measurement pose — the source of the yaw-rate-proportional
  // turn error (accepted-frame gt_err doubled from 1.0 m at <2 deg/s to
  // 2.1 m at >25 deg/s on run 12).
  Eigen::Matrix4f observer_prior_pose_;
  bool geo_delta_correction_;

  // Debug tracking
  Eigen::Matrix4f last_gicp_pose_;
  rclcpp::Time last_gicp_stamp_;
  bool last_gicp_valid_;
  double last_fitness_score_{-1.0};  // -1 = no scan yet
  double last_accepted_scan_stamp_{-1.0};  // s — stamp of last accepted GICP scan (P3 dead-reckon cov)
  Eigen::Matrix4f motion_ref_pose_;
  double motion_ref_stamp_sec_;
  bool motion_ref_valid_;

  // Trajectory. The actual ring of poses lives in path_buffer_ (deque, O(1)
  // pop_front when capping); path_msg is filled only when the path topic has
  // subscribers, so we don't pay an O(N) DDS serialize on every scan.
  nav_msgs::msg::Path path_msg;
  std::deque<geometry_msgs::msg::PoseStamped> path_buffer_;

  // IMU data structures
  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::atomic<bool> first_imu_received;
  bool imu_require_topic_allowlist_{true};
  std::vector<std::string> imu_topic_allowlist_;
  bool imu_require_frame_match_{true};
  // One-shot guard for the defensive IMU header.frame_id consistency check
  // in callbackImu. All-P1 design assumes every IMU message comes from
  // /gps_p1/imu and is referenced at imu_frame (= "gps_antenna_top" in yaml).
  // If someone re-points imu_topic at a different IMU, the code would
  // silently treat its axes as if they were at gps_antenna_top (the TF
  // lookup gps_antenna_top -> gps_antenna_top still returns identity).
  // This flag arms a one-time warning so the misconfiguration surfaces at
  // least once in the log.
  std::atomic<bool> imu_frame_id_checked_{false};

  // IMU calibration state
  std::atomic<bool> imu_calibrated_;
  double imu_calib_time_;           // seconds to accumulate for calibration
  double imu_calib_start_stamp_;
  int imu_calib_count_;
  Eigen::Vector3f imu_calib_gyro_sum_;
  Eigen::Vector3f imu_calib_accel_sum_;

  // RTK-driven IMU calibration (uses GT odom as truth source; allows calibrating
  // while moving). Falls back to the stationary path above if no GT sample
  // arrives within rtk_fallback_timeout_sec_ of the first IMU message.
  enum class InitPhase { WAITING, RTK_CALIBRATING, STATIONARY_CALIBRATING, DONE };
  std::atomic<InitPhase> init_phase_{InitPhase::WAITING};
  bool rtk_init_enabled_;
  double rtk_calib_window_sec_;
  double rtk_fallback_timeout_sec_;
  double first_imu_stamp_;                  // stamp of the first IMU msg (set on receipt)
  double rtk_calib_start_stamp_;            // stamp of the first IMU sample paired with GT
  int rtk_calib_count_;
  Eigen::Vector3f rtk_gyro_bias_sum_;
  Eigen::Vector3f rtk_accel_bias_sum_;
  Eigen::Vector3f rtk_gyro_bias_sq_sum_;    // for residual stddev sanity check
  Eigen::Vector3f rtk_accel_bias_sq_sum_;
  bool has_prev_gt_for_accel_;
  double prev_gt_stamp_;
  Eigen::Vector3f prev_v_world_;
  GtSample latest_rtk_seed_;                // latest GT sample, used to seed state at finalize
  bool has_latest_rtk_seed_;

  // Pose tracking
  struct Pose {
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
  };
  // Tracked base-frame pose in map.
  Pose basePose;
  Eigen::Vector3f prev_vel;

  // Geometric Observer State
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;  // body frame
    Eigen::Vector3f w;  // world frame
  };

  struct Velocity {
    Frames lin;  // linear velocity
    Frames ang;  // angular velocity
  };

  struct State {
    Eigen::Vector3f p;       // position in world frame
    Eigen::Quaternionf q;    // orientation in world frame
    Velocity v;              // velocity
    ImuBias b;               // IMU biases in body frame
  }; State state;

  struct Geo {
    std::atomic<bool> first_opt_done;
    std::mutex mtx;
    uint64_t update_seq;  // Incremented by updateState; checked by propagateState
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  }; Geo geo;

  // Current IMU measurement (for propagateState)
  ImuMeas imu_meas;

  // Sensor Type
  dlio::SensorType sensor;

  // Frames
  std::string map_frame;
  std::string base_frame;
  std::string odom_frame;
  std::string imu_frame;
  std::string lidar_frame;
  std::string utm_frame;

  // UTM transform: T_utm_map = T_world_utm.inverse()
  // Loaded from GLIM's T_world_utm.txt at startup when utm_transform_path is set
  bool utm_enabled_;
  Eigen::Matrix4f T_utm_map_;
  nav_msgs::msg::Path utm_path_msg_;
  std::deque<geometry_msgs::msg::PoseStamped> utm_path_buffer_;

  // Parameters
  std::string map_path_;
  double map_roll_deg_;
  double map_pitch_deg_;
  double map_yaw_deg_;
  bool publish_tf_;
  bool imu_only_mode_;
  bool use_odom_init_;
  bool use_odom_init_applied_{false};
  bool use_param_initial_pose_;
  std::string initial_pose_frame_;  // "lidar" or "base_link"
  bool pending_initial_pose_;  // true when initial pose needs conversion via baselink2lidar_T
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_roll_;
  double initial_pose_pitch_;
  double initial_pose_yaw_;

  // GICP parameters
  int gicp_max_iter_;
  int gicp_corr_randomness_;
  double gicp_max_corr_dist_;
  double gicp_transformation_epsilon_;
  double gicp_rotation_epsilon_;
  double gicp_fitness_reject_threshold_;
  bool gicp_reject_large_jumps_;
  double gicp_hessian_cond_max_;
  double gicp_hessian_fitness_warn_;
  double gicp_hessian_trans_warn_m_;
  double gicp_hessian_rot_warn_deg_;

  // P1 gating rework: per-map-normalized fitness gates + degeneracy-aware
  // partial updates (solution remapping) + turn-aware yaw-consistency veto.
  // Rationale/thresholds: docs/action_plan_turn_error_20260704.md.
  bool   fitness_baseline_enable_;       // maintain rolling-median fitness baseline
  int    fitness_baseline_window_;       // ring size (accepted-frame fitness samples)
  int    fitness_baseline_min_samples_;  // gates stay absolute-only until this many samples
  double fitness_baseline_seed_;         // expected per-map floor used during warm-up (0 = off)
  double fitness_ratio_reject_;          // reject scan when fitness/baseline exceeds this (<=0 off)
  bool   degen_partial_update_enable_;   // project delta instead of binary hessian reject
  bool   degen_full6d_;                  // full coupled 6x6 remapping (true) vs 3x3 blockwise (false)
  double degen_coupling_length_m_;       // characteristic lever arm making rad and m commensurable (full6d)
  double degen_rel_floor_6d_;            // full6d: eigen-axis degenerate if lambda < floor*lambda_max
  double degen_rel_floor_rot_;           // blockwise: rot eigen-axis degenerate if lambda < floor*lambda_max(block)
  double degen_rel_floor_trans_;         // blockwise: trans eigen-axis degenerate likewise
  bool   yaw_gate_enable_;               // turn-aware GICP-vs-IMU yaw consistency veto
  double yaw_gate_max_corr_deg_;         // SOFT veto: yaw corr above this AND ratio above fitnessRatio
  double yaw_gate_fitness_ratio_;        // soft-tier arming ratio (low-confidence match)
  double yaw_gate_hard_max_corr_deg_;    // HARD veto: unconditional yaw-corr bound (<=0 off) — P1 yaw-safety
  double gicp_nonconv_ok_max_trans_m_;   // PR#6: max correction for the non-converged fitness fallback (<=0 off)
  double gicp_nonconv_ok_max_rot_deg_;   // PR#6: max rotation for the non-converged fitness fallback (<=0 off)
  // Algorithmic yaw-defect fixes (optimizer-level, 2026-07-05):
  std::string gicp_dof_mode_;            // "6dof" | "4dof" (fix roll/pitch, default) | "3dof" (fix attitude)
  int gicp_full6dof_every_n_;            // periodic unconstrained scan to re-anchor roll/pitch (0 = never)
  int dof_scan_counter_ = 0;             // scan counter for the periodic 6dof refresh
  double gicp_prior_yaw_info_;           // soft in-optimizer yaw prior info (rad^-2, 0 = off)
  double gicp_prior_rollpitch_info_;     // soft in-optimizer roll/pitch prior info (rad^-2, 0 = off)
  std::deque<double> fitness_history_;   // accepted-frame fitness ring (scan thread only)

  // Preprocessing parameters
  double crop_size_;
  bool vf_use_;
  double vf_res_;

  // IMU and deskewing parameters
  bool deskew_;
  double gravity_;
  int imu_buffer_size_;
  bool flip_y_;

  // Geometric observer parameters
  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_Kz_damping_;
  double geo_abias_max_;
  double geo_gbias_max_;

  // Observer-correction stability bounds (P2#1). The proportional observer applies
  // dt*K corrections; this is forward-Euler and only stable for dt*K < 2. A long
  // scan gap (dropped Luminar frames / high-speed racing) would otherwise inject a
  // huge, unstable correction. Cap the effective timestep, and optionally clamp the
  // per-update position/velocity correction magnitude (0 = clamp disabled).
  double geo_observer_dt_max_;     // s   — cap on dt used in updateState corrections
  double geo_max_pos_correction_;  // m   — clamp per-update position correction (0=off)
  double geo_max_vel_correction_;  // m/s — clamp per-update velocity correction (0=off)
  double geo_max_yaw_correction_deg_;  // deg — clamp per-update yaw error before gain (0=off) — P1 yaw-safety
  double geo_max_rot_correction_deg_;  // deg — clamp per-update total rotation error (0=off)

  // Time/speed-based dead-reckoning covariance growth (P3). During GICP loss the
  // reported position sigma grows with elapsed dead-reckon time and distance
  // travelled (speed*time), not the raw missed-scan count. 0 rates disable growth.
  double dr_cov_time_rate_;        // m of sigma per second of dead reckoning
  double dr_cov_dist_frac_;        // m of sigma per metre travelled while dead reckoning

  // Debug parameters
  bool debug_pub_enabled_;
  bool debug_jump_log_enabled_;
  bool debug_verbose_scan_log_;
  bool debug_lm_print_;
  double debug_jump_trans_m_;
  double debug_jump_rot_deg_;
  // Speed/scan_dt-aware jump-gate scaling (P2#2). The effective large-jump
  // thresholds grow with how far the IMU prior could have drifted: translation
  // with speed*scan_dt, rotation with scan_dt. 0 scales reproduce the fixed
  // thresholds (debug_jump_trans_m_ / debug_jump_rot_deg_).
  double jump_trans_speed_scale_;  // extra trans threshold per (speed*scan_dt) metre
  double jump_rot_dt_scale_deg_;   // extra rot threshold (deg) per second of scan_dt
  // P1 yaw-safety: yaw-specific innovation gate (split from total rotation).
  double jump_yaw_max_deg_;        // base yaw budget vs IMU prior (<=0 disables)
  double jump_yaw_dt_scale_deg_;   // extra yaw budget per second of scan_dt
  double jump_yaw_total_max_deg_;  // absolute cap the dt scaling can never exceed
  bool motion_consistency_enable_;
  double motion_max_speed_mps_;
  double motion_max_reverse_m_;
  double motion_max_heading_error_deg_;
  double motion_max_curvature_1pm_;
  double motion_max_yaw_rate_deg_s_;
  double motion_min_step_for_heading_m_;
  double motion_min_step_for_curvature_m_;
  bool verbose_;

  // Extrinsics
  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselink2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  }; Extrinsics extrinsics;
  bool extrinsics_cached_;  // True once baselink2lidar_T has been populated from TF
  bool imu_extrinsics_cached_;  // True once baselink2imu has been populated from TF

  // Map visualization
  bool visualize_map_;
  double map_voxel_size_vis_;
  double map_voxel_size_ = 0.3;  // GICP target-map voxel leaf (m); 0 disables
  rclcpp::TimerBase::SharedPtr map_pub_timer_;
  rclcpp::TimerBase::SharedPtr input_health_timer_;

  // Pre-localization initial pose republisher (publishes initial guess + TF
  // until GICP produces a real result, so RViz has something to show).
  rclcpp::TimerBase::SharedPtr initial_pose_pub_timer_;

};

} // namespace gicp_localization

#endif // GICP_LOCALIZATION_H
