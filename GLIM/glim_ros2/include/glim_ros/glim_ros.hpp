#pragma once

#include <any>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

#include <glim_ros/lidar_concat.hpp>

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions& options);
  ~GlimROS();

  bool needs_wait();
  // False when any quality/safety extension (for example the GNSS anchor
  // divergence gate) has rejected the run.
  bool ok() const;
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  // `epoch_anchor_count` (-1 = single sensor / unused) is the primary scan's
  // point count in a concatenated multi-LiDAR cloud; forwarded to
  // extract_raw_points() so the epoch-axis rebase anchors on the primary scan's
  // earliest time rather than the global merged minimum. See points_callback_live().
  // `ingested` (optional): set true only when the cloud passed extraction and
  // TimeKeeper validation and was inserted into odometry estimation; false when
  // the frame was skipped. Offline readers use it for primary accounting.
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, int epoch_anchor_count = -1, bool* ingested = nullptr);
  void external_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // Live subscription entry point for the primary LiDAR. Merges any buffered
  // auxiliary clouds into the primary (matching the offline glim_rosbag /
  // glim_pcap_rosbag path) and then forwards the result to points_callback().
  void points_callback_live(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // Buffers an auxiliary LiDAR cloud for later time-matched merging.
  void aux_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t aux_index);

  void wait(bool auto_quit = false);
  void save(const std::string& path);
  size_t num_submaps();

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

  // True only when the live subscription-based mapping passway is enabled
  // (glim_ros/enable_online_mapping). Default false: GLIM maps offline only.
  bool online_mapping_enabled() const { return online_mapping_enabled_; }

private:
  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  // Fixed input-vector calibration. Both acceleration and gyro are rotated
  // into the IMU frame used by T_lidar_imu before entering any estimator.
  Eigen::Quaterniond imu_input_rotation = Eigen::Quaterniond::Identity();
  bool dump_on_unload;

  std::string intensity_field, ring_field;
  bool float64_time_is_epoch_ns = false;  // [P2 FIX 2026-07-15] Luminar FLOAT64-epoch-ns opt-in
  bool flip_points_y;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  bool online_mapping_enabled_ = false;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odom_sub;

  // Multi-LiDAR concatenation (primary luminar_front + auxiliary left/right).
  // aux_concat holds the per-sensor buffers; aux_buffers_mutex guards them
  // because the auxiliary subscription callbacks and points_callback_live()
  // (which reads the buffers via merge_clouds) may run on different threads.
  glim_ros::AuxConcatConfig aux_concat;
  std::mutex aux_buffers_mutex;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> aux_points_subs;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
