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
#include <memory>
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
  struct GtSample {
    double stamp;
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    Eigen::Vector3f v_lin_body;
    Eigen::Vector3f v_ang_body;
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

  void preprocessPointCloud(pcl::PointCloud<PointType>::Ptr& cloud);
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
  // legacy BESTGNSSPOS-enum gate (removed with the old NovAtel path).
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
    Eigen::Matrix4f T_primary_aux;              // p_primary = T * p_aux, cached from TF
    bool extrinsic_cached;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> buffer;
    std::mutex mtx;
  };
  std::vector<std::unique_ptr<AuxLidar>> aux_lidars_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> aux_subs_;
  rclcpp::CallbackGroup::SharedPtr aux_cb_group_;
  bool concat_enabled_;
  double concat_time_threshold_;
  size_t concat_buffer_size_;

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

  // Debug tracking
  Eigen::Matrix4f last_gicp_pose_;
  rclcpp::Time last_gicp_stamp_;
  bool last_gicp_valid_;
  double last_fitness_score_{-1.0};  // -1 = no scan yet

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
  double voxel_leaf_size_;
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

  // Debug parameters
  bool debug_pub_enabled_;
  bool debug_jump_log_enabled_;
  bool debug_verbose_scan_log_;
  bool debug_lm_print_;
  double debug_jump_trans_m_;
  double debug_jump_rot_deg_;
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
  rclcpp::TimerBase::SharedPtr map_pub_timer_;
  rclcpp::TimerBase::SharedPtr input_health_timer_;

  // Pre-localization initial pose republisher (publishes initial guess + TF
  // until GICP produces a real result, so RViz has something to show).
  rclcpp::TimerBase::SharedPtr initial_pose_pub_timer_;

};

} // namespace gicp_localization

#endif // GICP_LOCALIZATION_H
