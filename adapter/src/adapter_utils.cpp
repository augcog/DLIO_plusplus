#include "adapter/adapter_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace adapter {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr uint8_t kRtkFixed = 4;

}  // namespace

double stampToSec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
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

bool parseLocalEnuOrigin(const std::string& text, double& lat_deg, double& lon_deg, double& alt_m)
{
  if (text.empty()) {
    return false;
  }

  std::string normalized = text;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::istringstream in(normalized);
  if (!(in >> lat_deg >> lon_deg >> alt_m)) {
    return false;
  }
  return std::isfinite(lat_deg) && std::isfinite(lon_deg) && std::isfinite(alt_m);
}

bool parseLocalEnuOriginTtl(const std::string& path, double& lat_deg, double& lon_deg, double& alt_m)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    if (cells.size() < 3) {
      return false;
    }
    try {
      lat_deg = std::stod(cells[cells.size() - 3]);
      lon_deg = std::stod(cells[cells.size() - 2]);
      alt_m = std::stod(cells[cells.size() - 1]);
    } catch (const std::exception&) {
      return false;
    }
    return std::isfinite(lat_deg) && std::isfinite(lon_deg) && std::isfinite(alt_m);
  }
  return false;
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

}  // namespace adapter
