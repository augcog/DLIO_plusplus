#include "dlio_input_adapter/adapter_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace dlio_input_adapter {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr uint8_t kRtkFixed = 4;

}  // namespace

double stampToSec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

int64_t stampToNs(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time secToStamp(double sec)
{
  const int64_t ns = static_cast<int64_t>(std::llround(sec * 1e9));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

double p1ToSec(const fusion_engine_msgs::msg::Timestamp& stamp)
{
  return static_cast<double>(stamp.seconds) + static_cast<double>(stamp.fraction_ns) * 1e-9;
}

Eigen::Quaterniond rpyToQuat(double roll_deg, double pitch_deg, double yaw_deg)
{
  const Eigen::AngleAxisd roll(roll_deg * kDegToRad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(pitch_deg * kDegToRad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(yaw_deg * kDegToRad, Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(yaw * pitch * roll).normalized();
}

bool parseOrigin(const std::string& text, double& easting, double& northing)
{
  if (text.empty()) {
    return false;
  }

  std::string normalized = text;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::istringstream in(normalized);
  return static_cast<bool>(in >> easting >> northing);
}

bool readMatrix4(const std::string& path, Eigen::Matrix4d& matrix)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }

  std::vector<std::array<double, 4>> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line.find("T_world_utm") != std::string::npos) {
      continue;
    }
    std::istringstream ls(line);
    std::array<double, 4> row{};
    if (ls >> row[0] >> row[1] >> row[2] >> row[3]) {
      rows.push_back(row);
    }
  }

  if (rows.size() != 4) {
    return false;
  }
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      matrix(r, c) = rows[static_cast<size_t>(r)][static_cast<size_t>(c)];
    }
  }
  return true;
}

const sensor_msgs::msg::PointField* findTimeField(const sensor_msgs::msg::PointCloud2& msg)
{
  static const std::array<std::string, 4> kNames = {"t", "time", "time_stamp", "timestamp"};
  for (const auto& field : msg.fields) {
    if (std::find(kNames.begin(), kNames.end(), field.name) != kNames.end()) {
      return &field;
    }
  }
  return nullptr;
}

bool repairLuminarPointTimestamps(sensor_msgs::msg::PointCloud2& msg, std::string* reason)
{
  const auto* field = findTimeField(msg);
  if (field == nullptr) {
    if (reason != nullptr) {
      *reason = "point cloud has no timestamp field";
    }
    return false;
  }
  if (field->name != "timestamp" ||
      field->datatype != sensor_msgs::msg::PointField::UINT8 ||
      field->count != 8 ||
      msg.point_step != 56) {
    if (reason != nullptr) {
      std::ostringstream out;
      out << "unsupported Luminar schema: field=" << field->name
          << " datatype=" << static_cast<unsigned>(field->datatype)
          << " count=" << field->count
          << " point_step=" << msg.point_step;
      *reason = out.str();
    }
    return false;
  }

  const size_t count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (count == 0) {
    if (reason != nullptr) {
      *reason = "empty point cloud";
    }
    return false;
  }

  const size_t offset = field->offset;
  const size_t point_step = msg.point_step;
  uint64_t min_ts = std::numeric_limits<uint64_t>::max();
  for (size_t i = 0; i < count; ++i) {
    uint64_t raw = 0;
    std::memcpy(&raw, &msg.data[i * point_step + offset], sizeof(raw));
    min_ts = std::min(min_ts, raw);
  }

  const int64_t shift_ns = stampToNs(msg.header.stamp) - static_cast<int64_t>(min_ts);
  for (size_t i = 0; i < count; ++i) {
    const size_t pos = i * point_step + offset;
    uint64_t raw = 0;
    std::memcpy(&raw, &msg.data[pos], sizeof(raw));
    const int64_t shifted = static_cast<int64_t>(raw) + shift_ns;
    if (shifted < 0) {
      if (reason != nullptr) {
        *reason = "point timestamp repair would make a timestamp negative";
      }
      return false;
    }
    const uint64_t repaired = static_cast<uint64_t>(shifted);
    std::memcpy(&msg.data[pos], &repaired, sizeof(repaired));
  }
  if (reason != nullptr) {
    reason->clear();
  }
  return true;
}

std::vector<double> retimeArrivalStamps(const std::vector<double>& arrivals, double imu_period_sec)
{
  std::vector<double> stamps = arrivals;
  if (stamps.empty()) {
    return stamps;
  }
  for (int i = static_cast<int>(stamps.size()) - 2; i >= 0; --i) {
    const double ceil = stamps[static_cast<size_t>(i + 1)] - imu_period_sec;
    if (stamps[static_cast<size_t>(i)] > ceil) {
      stamps[static_cast<size_t>(i)] = ceil;
    }
  }
  return stamps;
}

bool posePassesRtkGate(const fusion_engine_msgs::msg::Pose& msg,
                       double max_var_xy,
                       double max_var_z)
{
  return msg.solution_type == kRtkFixed &&
         msg.position_covariance[0] <= max_var_xy &&
         msg.position_covariance[4] <= max_var_xy &&
         msg.position_covariance[8] <= max_var_z;
}

nav_msgs::msg::Odometry transformOdomToMap(const nav_msgs::msg::Odometry& odom,
                                           const Eigen::Matrix4d& t_world_utm)
{
  nav_msgs::msg::Odometry out = odom;
  out.header.frame_id = "map";

  const Eigen::Matrix3d R = t_world_utm.block<3, 3>(0, 0);
  const Eigen::Vector3d t = t_world_utm.block<3, 1>(0, 3);
  const Eigen::Vector3d p(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z);
  const Eigen::Vector3d pm = R * p + t;
  out.pose.pose.position.x = pm.x();
  out.pose.pose.position.y = pm.y();
  out.pose.pose.position.z = pm.z();

  const auto& q_in = odom.pose.pose.orientation;
  const Eigen::Quaterniond q(q_in.w, q_in.x, q_in.y, q_in.z);
  const Eigen::Quaterniond qm(R * q.normalized().toRotationMatrix());
  out.pose.pose.orientation.x = qm.x();
  out.pose.pose.orientation.y = qm.y();
  out.pose.pose.orientation.z = qm.z();
  out.pose.pose.orientation.w = qm.w();

  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      cov(r, c) = odom.pose.covariance[static_cast<size_t>(r * 6 + c)];
    }
  }
  cov.block<3, 3>(0, 0) = R * cov.block<3, 3>(0, 0) * R.transpose();
  cov.block<3, 3>(3, 3) = R * cov.block<3, 3>(3, 3) * R.transpose();
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      out.pose.covariance[static_cast<size_t>(r * 6 + c)] = cov(r, c);
    }
  }

  return out;
}

P1ClockMapper::P1ClockMapper(double bin_seconds) : bin_seconds_(bin_seconds) {}

void P1ClockMapper::addPosePair(double arrival_ros, double p1_time)
{
  if (!std::isfinite(first_p1_)) {
    first_p1_ = p1_time;
  }

  const int bin = static_cast<int>(std::floor((p1_time - first_p1_) / bin_seconds_));
  if (bin < 0) {
    return;
  }
  if (static_cast<size_t>(bin) >= bins_.size()) {
    bins_.resize(static_cast<size_t>(bin) + 1);
  }

  auto& b = bins_[static_cast<size_t>(bin)];
  b.count++;
  b.center_sum += p1_time;
  b.min_lag = std::min(b.min_lag, arrival_ros - p1_time);
}

bool P1ClockMapper::ready() const
{
  return std::any_of(bins_.begin(), bins_.end(), [](const Bin& b) { return b.count > 0; });
}

double P1ClockMapper::toRos(double p1_time) const
{
  std::vector<std::pair<double, double>> envelope;
  envelope.reserve(bins_.size());
  for (const auto& b : bins_) {
    if (b.count > 0) {
      envelope.emplace_back(b.center_sum / static_cast<double>(b.count), b.min_lag);
    }
  }
  if (envelope.empty()) {
    return p1_time;
  }
  if (envelope.size() == 1 || offsetDrift(envelope) < 0.005) {
    std::vector<double> offsets;
    offsets.reserve(envelope.size());
    for (const auto& p : envelope) {
      offsets.push_back(p.second);
    }
    const size_t mid = offsets.size() / 2;
    std::nth_element(offsets.begin(), offsets.begin() + static_cast<long>(mid), offsets.end());
    return p1_time + offsets[mid];
  }

  if (p1_time <= envelope.front().first) {
    return p1_time + envelope.front().second;
  }
  for (size_t i = 1; i < envelope.size(); ++i) {
    if (p1_time <= envelope[i].first) {
      const double t0 = envelope[i - 1].first;
      const double t1 = envelope[i].first;
      const double u = (p1_time - t0) / std::max(1e-9, t1 - t0);
      return p1_time + envelope[i - 1].second * (1.0 - u) + envelope[i].second * u;
    }
  }
  return p1_time + envelope.back().second;
}

double P1ClockMapper::driftMs() const
{
  std::vector<std::pair<double, double>> envelope;
  for (const auto& b : bins_) {
    if (b.count > 0) {
      envelope.emplace_back(b.center_sum / static_cast<double>(b.count), b.min_lag);
    }
  }
  return offsetDrift(envelope) * 1e3;
}

double P1ClockMapper::offsetDrift(const std::vector<std::pair<double, double>>& envelope)
{
  if (envelope.empty()) {
    return 0.0;
  }
  auto [min_it, max_it] = std::minmax_element(
    envelope.begin(), envelope.end(),
    [](const auto& a, const auto& b) { return a.second < b.second; });
  return max_it->second - min_it->second;
}

}  // namespace dlio_input_adapter
