#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <GeographicLib/UTMUPS.hpp>

#include "dlio_input_adapter/adapter_utils.hpp"
#include "fusion_engine_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kDegToRadSq = kDegToRad * kDegToRad;

}  // namespace

using namespace dlio_input_adapter;

class DlioInputAdapter : public rclcpp::Node {
public:
  DlioInputAdapter()
  : Node("dlio_input_adapter"),
    clock_mapper_(declare_parameter("p1_clock_bin_seconds", 60.0))
  {
    pose_input_topic_ = declare_parameter("pose_input_topic", "/atlas/pose_filtered");
    imu_input_topic_ = declare_parameter("imu_input_topic", "/atlas/imu_calibrated");
    imu_stamp_mode_ = declare_parameter("imu_stamp_mode", "auto");
    p1_like_threshold_sec_ = declare_parameter("p1_like_threshold_sec", 100000000.0);
    imu_lookahead_ = static_cast<size_t>(declare_parameter("imu_arrival_retime_lookahead", 128));
    imu_flush_timeout_sec_ = declare_parameter("imu_flush_timeout_sec", 0.5);
    nominal_imu_period_sec_ = declare_parameter("nominal_imu_period_sec", 0.01);
    imu_period_sec_ = nominal_imu_period_sec_;
    pose_input_reliability_ = declare_parameter("pose_input_reliability", "reliable");
    pose_input_qos_depth_ = declare_parameter("pose_input_qos_depth", 100);
    imu_input_reliability_ = declare_parameter("imu_input_reliability", "best_effort");
    imu_input_qos_depth_ = declare_parameter("imu_input_qos_depth", 100);
    imu_frame_id_ = declare_parameter("imu_frame_id", "gps_antenna_top");
    odom_frame_id_ = declare_parameter("odom_frame_id", "utm");
    body_frame_id_ = declare_parameter("body_frame_id", "gps_antenna_top");
    utm_zone_ = declare_parameter("utm_zone", 0);
    utm_origin_output_path_ = declare_parameter("utm_origin_output_path", "");
    rtk_max_var_xy_ = declare_parameter("rtk_max_var_xy", 1e-3);
    rtk_max_var_z_ = declare_parameter("rtk_max_var_z", 5e-3);
    lidar_time_offset_sec_ = declare_parameter("lidar_time_offset", 0.0);
    repair_luminar_point_time_ = declare_parameter("repair_luminar_point_time", true);
    strict_luminar_schema_ = declare_parameter("strict_luminar_schema", true);
    lidar_input_reliability_ = declare_parameter("lidar_input_reliability", "best_effort");
    lidar_input_qos_depth_ = declare_parameter("lidar_input_qos_depth", 5);
    t_world_utm_path_ = declare_parameter("T_world_utm_path", "");
    summary_output_path_ = declare_parameter("summary_output_path", "");
    imu_p1_sidecar_path_ = declare_parameter("imu_p1_sidecar_path", "");
    imu_p1_sidecar_match_tolerance_sec_ =
      declare_parameter("imu_p1_sidecar_match_tolerance_sec", 0.02);

    if (!imu_p1_sidecar_path_.empty()) {
      loadImuP1Sidecar(imu_p1_sidecar_path_);
    }

    double origin_e = 0.0;
    double origin_n = 0.0;
    const std::string origin_text = declare_parameter("utm_origin", "");
    if (parseOrigin(origin_text, origin_e, origin_n)) {
      utm_origin_e_ = origin_e;
      utm_origin_n_ = origin_n;
      utm_origin_ready_ = true;
    } else if (!origin_text.empty()) {
      throw std::runtime_error("utm_origin must be empty or formatted as 'E,N'");
    }

    if (!t_world_utm_path_.empty()) {
      if (!readMatrix4(t_world_utm_path_, t_world_utm_)) {
        throw std::runtime_error("failed to read T_world_utm_path: " + t_world_utm_path_);
      }
      publish_map_odom_ = true;
    }

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/gps_p1/imu", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gps_p1/filtered_odom", 50);
    odom_rtk_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gps_p1/filtered_odom_rtk_fixed", 50);
    if (publish_map_odom_) {
      odom_map_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gps_p1/filtered_odom_map", 50);
    }

    pose_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lidar_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions pose_options;
    pose_options.callback_group = pose_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_group_;
    pose_sub_ = create_subscription<fusion_engine_msgs::msg::Pose>(
      pose_input_topic_, inputQos(pose_input_reliability_, pose_input_qos_depth_),
      std::bind(&DlioInputAdapter::poseCallback, this, std::placeholders::_1), pose_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic_, inputQos(imu_input_reliability_, imu_input_qos_depth_),
      std::bind(&DlioInputAdapter::imuCallback, this, std::placeholders::_1), imu_options);

    addLidarBridge(
      declare_parameter("luminar_front_input_topic", "/dlio_raw/luminar_front/points"),
      declare_parameter("luminar_front_output_topic", "/luminar_front/points"));
    addLidarBridge(
      declare_parameter("luminar_left_input_topic", "/dlio_raw/luminar_left/points"),
      declare_parameter("luminar_left_output_topic", "/luminar_left/points"));
    addLidarBridge(
      declare_parameter("luminar_right_input_topic", "/dlio_raw/luminar_right/points"),
      declare_parameter("luminar_right_output_topic", "/luminar_right/points"));

    flush_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DlioInputAdapter::flushTimerCallback, this), timer_group_);
    if (!summary_output_path_.empty()) {
      summary_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DlioInputAdapter::writeSummaryFile, this), timer_group_);
    }

    RCLCPP_INFO(
      get_logger(),
      "DLIO input adapter ready: pose=%s imu=%s imu_stamp_mode=%s map_odom=%s",
      pose_input_topic_.c_str(), imu_input_topic_.c_str(), imu_stamp_mode_.c_str(),
      publish_map_odom_ ? "true" : "false");
  }

  ~DlioInputAdapter() override
  {
    writeSummaryFile();
    const std::string summary = summaryText();
    RCLCPP_INFO(get_logger(), "DLIO input adapter summary:\n%s", summary.c_str());
  }

  std::string summaryText() const
  {
    std::scoped_lock lock(core_mutex_, lidar_mutex_);
    std::ostringstream out;
    out << "pose_in=" << pose_in_count_
        << " odom_out=" << odom_out_count_
        << " rtk_out=" << rtk_out_count_
        << " map_odom_out=" << map_odom_out_count_
        << " imu_in=" << imu_in_count_
        << " imu_out=" << imu_out_count_;
    if (!imu_p1_sidecar_.empty()) {
      out << " imu_p1_sidecar_match=" << imu_p1_sidecar_match_count_
          << " imu_p1_sidecar_miss=" << imu_p1_sidecar_miss_count_
          << " imu_p1_sidecar_skip=" << imu_p1_sidecar_skip_count_;
    }
    out << "\n";
    for (const auto& item : lidar_in_count_by_output_) {
      const auto out_it = lidar_out_count_by_output_.find(item.first);
      const uint64_t out_count = out_it == lidar_out_count_by_output_.end() ? 0 : out_it->second;
      out << "lidar " << item.first << " in=" << item.second << " out=" << out_count << "\n";
    }
    return out.str();
  }

  void writeSummaryFile() const
  {
    if (summary_output_path_.empty()) {
      return;
    }
    std::ofstream out(summary_output_path_);
    out << summaryText();
  }

private:
  struct QueuedImu {
    sensor_msgs::msg::Imu msg;
    double arrival = 0.0;
    double p1 = 0.0;
  };

  struct ImuP1SidecarSample {
    double capture_ros = 0.0;
    double p1_time = 0.0;
  };

  static std::vector<std::string> splitCsvLine(const std::string& line)
  {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    return cells;
  }

  void loadImuP1Sidecar(const std::string& path)
  {
    std::ifstream in(path);
    if (!in) {
      throw std::runtime_error("failed to open imu_p1_sidecar_path: " + path);
    }

    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      const auto cells = splitCsvLine(line);
      if (cells.size() < 2) {
        continue;
      }
      try {
        const double capture_ros = std::stod(cells[0]);
        const double p1_time = std::stod(cells[1]);
        if (std::isfinite(capture_ros) && std::isfinite(p1_time) &&
            capture_ros > 0.0 && p1_time > 0.0) {
          imu_p1_sidecar_.push_back({capture_ros, p1_time});
        }
      } catch (const std::exception&) {
        // Header or malformed line.
      }
    }

    std::sort(
      imu_p1_sidecar_.begin(), imu_p1_sidecar_.end(),
      [](const auto& a, const auto& b) { return a.capture_ros < b.capture_ros; });
    if (imu_p1_sidecar_.empty()) {
      throw std::runtime_error("imu_p1_sidecar_path contained no usable samples: " + path);
    }
    RCLCPP_INFO(
      get_logger(),
      "Loaded IMU P1 sidecar: %zu samples from %s (match_tolerance=%.3f ms)",
      imu_p1_sidecar_.size(), path.c_str(), imu_p1_sidecar_match_tolerance_sec_ * 1e3);
  }

  bool lookupSidecarP1(double capture_ros, double& p1_time)
  {
    if (imu_p1_sidecar_.empty()) {
      return false;
    }

    const double tol = std::max(0.0, imu_p1_sidecar_match_tolerance_sec_);
    while (imu_p1_sidecar_index_ < imu_p1_sidecar_.size() &&
           imu_p1_sidecar_[imu_p1_sidecar_index_].capture_ros < capture_ros - tol) {
      ++imu_p1_sidecar_index_;
      ++imu_p1_sidecar_skip_count_;
    }

    size_t best = imu_p1_sidecar_.size();
    double best_abs_dt = std::numeric_limits<double>::infinity();
    const size_t begin = imu_p1_sidecar_index_ > 0 ? imu_p1_sidecar_index_ - 1 : 0;
    const size_t end = std::min(imu_p1_sidecar_.size(), imu_p1_sidecar_index_ + 3);
    for (size_t i = begin; i < end; ++i) {
      const double abs_dt = std::abs(imu_p1_sidecar_[i].capture_ros - capture_ros);
      if (abs_dt <= tol && abs_dt < best_abs_dt) {
        best = i;
        best_abs_dt = abs_dt;
      }
    }

    if (best == imu_p1_sidecar_.size()) {
      ++imu_p1_sidecar_miss_count_;
      return false;
    }

    p1_time = imu_p1_sidecar_[best].p1_time;
    if (best >= imu_p1_sidecar_index_) {
      imu_p1_sidecar_skip_count_ += best - imu_p1_sidecar_index_;
    }
    imu_p1_sidecar_index_ = best + 1;
    ++imu_p1_sidecar_match_count_;
    return true;
  }

  void addLidarBridge(const std::string& input_topic, const std::string& output_topic)
  {
    auto pub = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());
    lidar_pubs_.push_back(pub);
    const size_t index = lidar_pubs_.size() - 1;
    rclcpp::SubscriptionOptions options;
    options.callback_group = lidar_group_;
    lidar_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, lidarInputQos(),
      [this, index, input_topic, output_topic](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        lidarCallback(std::move(msg), lidar_pubs_[index], input_topic, output_topic);
      }, options));
  }

  rclcpp::QoS lidarInputQos() const
  {
    return inputQos(lidar_input_reliability_, lidar_input_qos_depth_);
  }

  rclcpp::QoS inputQos(const std::string& reliability_text, int depth) const
  {
    rclcpp::QoS qos = depth <= 0
      ? rclcpp::QoS(rclcpp::KeepAll())
      : rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(depth)));
    std::string reliability = reliability_text;
    std::transform(reliability.begin(), reliability.end(), reliability.begin(), ::tolower);
    if (reliability == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    qos.durability_volatile();
    return qos;
  }

  void poseCallback(const fusion_engine_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    ++pose_in_count_;
    const double arrival = stampToSec(msg->header.stamp);
    const double p1_time = p1ToSec(msg->p1_time);
    clock_mapper_.addPosePair(arrival, p1_time);

    if (!utm_ready_) {
      initUtm(*msg);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = monotonicStamp("/gps_p1/filtered_odom", clock_mapper_.toRos(p1_time));
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = body_frame_id_;

    double easting = 0.0;
    double northing = 0.0;
    double gamma = 0.0;
    double scale = 0.0;
    int zone = utm_zone_;
    bool northp = utm_northp_;
    GeographicLib::UTMUPS::Forward(
      msg->latitude, msg->longitude, zone, northp, easting, northing, gamma, scale, utm_zone_);

    odom.pose.pose.position.x = easting - utm_origin_e_;
    odom.pose.pose.position.y = northing - utm_origin_n_;
    odom.pose.pose.position.z = msg->altitude;
    const Eigen::Quaterniond q = rpyToQuat(msg->rpy.roll, msg->rpy.pitch, msg->rpy.yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    const auto& pc = msg->position_covariance;
    const auto& rc = msg->rpy_covariance;
    odom.pose.covariance[0] = static_cast<double>(pc[0]);
    odom.pose.covariance[7] = static_cast<double>(pc[4]);
    odom.pose.covariance[14] = static_cast<double>(pc[8]);
    odom.pose.covariance[21] = static_cast<double>(rc[0]) * kDegToRadSq;
    odom.pose.covariance[28] = static_cast<double>(rc[4]) * kDegToRadSq;
    odom.pose.covariance[35] = static_cast<double>(rc[8]) * kDegToRadSq;

    odom.twist.twist.linear.x = msg->velflu.x;
    odom.twist.twist.linear.y = msg->velflu.y;
    odom.twist.twist.linear.z = msg->velflu.z;
    const auto& vc = msg->velflu_covariance;
    odom.twist.covariance[0] = static_cast<double>(vc[0]);
    odom.twist.covariance[7] = static_cast<double>(vc[4]);
    odom.twist.covariance[14] = static_cast<double>(vc[8]);

    odom_pub_->publish(odom);
    ++odom_out_count_;
    if (isRtkFixed(*msg)) {
      auto rtk = odom;
      rtk.header.stamp = monotonicStamp("/gps_p1/filtered_odom_rtk_fixed", stampToSec(odom.header.stamp));
      odom_rtk_pub_->publish(rtk);
      ++rtk_out_count_;
    }

    if (publish_map_odom_) {
      publishMapOdom(odom);
    }

    flushP1ImuQueue();
  }

  void initUtm(const fusion_engine_msgs::msg::Pose& msg)
  {
    if (utm_zone_ == 0) {
      utm_zone_ = static_cast<int>(std::floor((msg.longitude + 180.0) / 6.0)) + 1;
    }
    utm_northp_ = msg.latitude >= 0.0;
    int zone = utm_zone_;
    bool northp = utm_northp_;
    double easting = 0.0;
    double northing = 0.0;
    double gamma = 0.0;
    double scale = 0.0;
    GeographicLib::UTMUPS::Forward(
      msg.latitude, msg.longitude, zone, northp, easting, northing, gamma, scale, utm_zone_);

    if (!utm_origin_ready_) {
      utm_origin_e_ = std::floor(easting / 10000.0) * 10000.0;
      utm_origin_n_ = std::floor(northing / 10000.0) * 10000.0;
      utm_origin_ready_ = true;
    }
    utm_ready_ = true;

    if (!utm_origin_output_path_.empty()) {
      std::ofstream out(utm_origin_output_path_);
      out << "# UTM zone " << utm_zone_ << (utm_northp_ ? "N" : "S")
          << "; subtract this origin from raw UTM\n"
          << std::fixed << std::setprecision(3) << utm_origin_e_ << "," << utm_origin_n_ << "\n";
    }

    RCLCPP_INFO(
      get_logger(), "UTM zone %d%s origin %.3f,%.3f",
      utm_zone_, utm_northp_ ? "N" : "S", utm_origin_e_, utm_origin_n_);
  }

  bool isRtkFixed(const fusion_engine_msgs::msg::Pose& msg) const
  {
    return dlio_input_adapter::posePassesRtkGate(msg, rtk_max_var_xy_, rtk_max_var_z_);
  }

  void publishMapOdom(const nav_msgs::msg::Odometry& odom)
  {
    nav_msgs::msg::Odometry out = dlio_input_adapter::transformOdomToMap(odom, t_world_utm_);
    out.header.stamp = monotonicStamp("/gps_p1/filtered_odom_map", stampToSec(odom.header.stamp));
    odom_map_pub_->publish(out);
    ++map_odom_out_count_;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    ++imu_in_count_;
    last_imu_wall_time_ = std::chrono::steady_clock::now();
    const double stamp = stampToSec(msg->header.stamp);
    if (!imu_p1_sidecar_.empty()) {
      double sidecar_p1 = 0.0;
      if (lookupSidecarP1(stamp, sidecar_p1)) {
        QueuedImu queued;
        queued.msg = *msg;
        queued.p1 = sidecar_p1;
        p1_imu_queue_.push_back(std::move(queued));
        flushP1ImuQueue();
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No IMU P1 sidecar match for capture stamp %.6f; falling back to imu_stamp_mode=%s.",
        stamp, imu_stamp_mode_.c_str());
    }

    const bool p1_like = stamp > 0.0 && stamp < p1_like_threshold_sec_;
    const bool use_p1 = imu_stamp_mode_ == "p1" || (imu_stamp_mode_ == "auto" && p1_like);

    if (use_p1) {
      QueuedImu queued;
      queued.msg = *msg;
      queued.p1 = stamp;
      p1_imu_queue_.push_back(std::move(queued));
      flushP1ImuQueue();
      return;
    }

    if (imu_stamp_mode_ != "auto" && imu_stamp_mode_ != "arrival_retime") {
      RCLCPP_WARN_ONCE(get_logger(), "Unsupported imu_stamp_mode='%s'; using arrival_retime.",
                       imu_stamp_mode_.c_str());
    }

    updateImuPeriod(stamp);
    QueuedImu queued;
    queued.msg = *msg;
    queued.arrival = stamp;
    arrival_imu_queue_.push_back(std::move(queued));
    while (arrival_imu_queue_.size() > imu_lookahead_) {
      publishFrontArrivalRetimedImu();
    }
  }

  void updateImuPeriod(double arrival)
  {
    if (last_imu_arrival_ > 0.0) {
      const double dt = arrival - last_imu_arrival_;
      if (dt > 0.5 * nominal_imu_period_sec_ && dt < 1.5 * nominal_imu_period_sec_) {
        imu_period_samples_.push_back(dt);
        if (imu_period_samples_.size() > 2048) {
          imu_period_samples_.erase(imu_period_samples_.begin());
        }
        auto tmp = imu_period_samples_;
        const size_t mid = tmp.size() / 2;
        std::nth_element(tmp.begin(), tmp.begin() + static_cast<long>(mid), tmp.end());
        imu_period_sec_ = tmp[mid];
      }
    }
    last_imu_arrival_ = arrival;
  }

  std::vector<double> retimeArrivalQueue() const
  {
    std::vector<double> stamps;
    stamps.reserve(arrival_imu_queue_.size());
    for (const auto& q : arrival_imu_queue_) {
      stamps.push_back(q.arrival);
    }
    return dlio_input_adapter::retimeArrivalStamps(stamps, imu_period_sec_);
  }

  void publishFrontArrivalRetimedImu()
  {
    if (arrival_imu_queue_.empty()) {
      return;
    }
    const auto stamps = retimeArrivalQueue();
    auto queued = arrival_imu_queue_.front();
    arrival_imu_queue_.pop_front();
    publishImu(queued.msg, stamps.front());
  }

  void flushP1ImuQueue()
  {
    if (!clock_mapper_.ready()) {
      return;
    }
    while (!p1_imu_queue_.empty()) {
      auto queued = p1_imu_queue_.front();
      p1_imu_queue_.pop_front();
      publishImu(queued.msg, clock_mapper_.toRos(queued.p1));
    }
  }

  void flushTimerCallback()
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    flushP1ImuQueue();
    if (arrival_imu_queue_.empty()) {
      return;
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_imu_wall_time_).count();
    if (elapsed < imu_flush_timeout_sec_) {
      return;
    }
    while (!arrival_imu_queue_.empty()) {
      publishFrontArrivalRetimedImu();
    }
  }

  void publishImu(sensor_msgs::msg::Imu msg, double stamp)
  {
    msg.header.stamp = monotonicStamp("/gps_p1/imu", stamp);
    msg.header.frame_id = imu_frame_id_;
    imu_pub_->publish(msg);
    ++imu_out_count_;
  }

  builtin_interfaces::msg::Time monotonicStamp(const std::string& topic, double sec)
  {
    double& last = last_stamp_by_topic_[topic];
    if (last > 0.0 && sec <= last) {
      sec = last + 1e-6;
    }
    last = sec;
    return secToStamp(sec);
  }

  void lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg,
                     const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                     const std::string& input_topic,
                     const std::string& output_topic)
  {
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      ++lidar_in_count_by_output_[output_topic];
    }
    sensor_msgs::msg::PointCloud2 out = *msg;
    if (std::abs(lidar_time_offset_sec_) > 1e-12) {
      out.header.stamp = secToStamp(stampToSec(out.header.stamp) + lidar_time_offset_sec_);
    }

    if (repair_luminar_point_time_ && !repairLuminarPointTime(out, input_topic)) {
      if (strict_luminar_schema_) {
        return;
      }
    }

    pub->publish(out);
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      ++lidar_out_count_by_output_[output_topic];
    }
    (void)output_topic;
  }

  bool repairLuminarPointTime(sensor_msgs::msg::PointCloud2& msg, const std::string& topic)
  {
    std::string reason;
    if (!dlio_input_adapter::repairLuminarPointTimestamps(msg, &reason)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s %s.", topic.c_str(), reason.c_str());
      return false;
    }
    return true;
  }

  std::string pose_input_topic_;
  std::string imu_input_topic_;
  std::string imu_stamp_mode_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string body_frame_id_;
  std::string utm_origin_output_path_;
  std::string t_world_utm_path_;
  std::string summary_output_path_;
  std::string pose_input_reliability_;
  std::string imu_input_reliability_;
  std::string lidar_input_reliability_;
  std::string imu_p1_sidecar_path_;

  P1ClockMapper clock_mapper_;
  double p1_like_threshold_sec_ = 100000000.0;
  double imu_p1_sidecar_match_tolerance_sec_ = 0.02;
  size_t imu_lookahead_ = 128;
  double imu_flush_timeout_sec_ = 0.5;
  double nominal_imu_period_sec_ = 0.01;
  double imu_period_sec_ = 0.01;
  double last_imu_arrival_ = 0.0;
  double rtk_max_var_xy_ = 1e-3;
  double rtk_max_var_z_ = 5e-3;
  double lidar_time_offset_sec_ = 0.0;
  int pose_input_qos_depth_ = 100;
  int imu_input_qos_depth_ = 100;
  int lidar_input_qos_depth_ = 5;
  bool repair_luminar_point_time_ = true;
  bool strict_luminar_schema_ = true;
  bool utm_ready_ = false;
  bool utm_origin_ready_ = false;
  bool utm_northp_ = true;
  bool publish_map_odom_ = false;
  int utm_zone_ = 0;
  double utm_origin_e_ = 0.0;
  double utm_origin_n_ = 0.0;
  Eigen::Matrix4d t_world_utm_ = Eigen::Matrix4d::Identity();
  std::chrono::steady_clock::time_point last_imu_wall_time_ = std::chrono::steady_clock::now();

  std::vector<double> imu_period_samples_;
  std::deque<QueuedImu> arrival_imu_queue_;
  std::deque<QueuedImu> p1_imu_queue_;
  std::vector<ImuP1SidecarSample> imu_p1_sidecar_;
  size_t imu_p1_sidecar_index_ = 0;
  std::map<std::string, double> last_stamp_by_topic_;

  rclcpp::CallbackGroup::SharedPtr pose_group_;
  rclcpp::CallbackGroup::SharedPtr imu_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_rtk_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_map_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_pubs_;

  rclcpp::Subscription<fusion_engine_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_subs_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;

  uint64_t pose_in_count_ = 0;
  uint64_t odom_out_count_ = 0;
  uint64_t rtk_out_count_ = 0;
  uint64_t map_odom_out_count_ = 0;
  uint64_t imu_in_count_ = 0;
  uint64_t imu_out_count_ = 0;
  uint64_t imu_p1_sidecar_match_count_ = 0;
  uint64_t imu_p1_sidecar_miss_count_ = 0;
  uint64_t imu_p1_sidecar_skip_count_ = 0;
  std::map<std::string, uint64_t> lidar_in_count_by_output_;
  std::map<std::string, uint64_t> lidar_out_count_by_output_;

  mutable std::mutex core_mutex_;
  mutable std::mutex lidar_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DlioInputAdapter>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
