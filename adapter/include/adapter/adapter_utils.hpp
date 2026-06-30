#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "builtin_interfaces/msg/time.hpp"
#include "fusion_engine_msgs/msg/pose.hpp"
#include "fusion_engine_msgs/msg/timestamp.hpp"

namespace adapter {

double stampToSec(const builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time secToStamp(double sec);
double p1ToSec(const fusion_engine_msgs::msg::Timestamp& stamp);
Eigen::Quaterniond rpyToQuat(double roll_deg, double pitch_deg, double yaw_deg);

bool parseLocalEnuOrigin(const std::string& text, double& lat_deg, double& lon_deg, double& alt_m);
bool parseLocalEnuOriginTtl(const std::string& path, double& lat_deg, double& lon_deg, double& alt_m);

std::vector<double> retimeArrivalStamps(const std::vector<double>& arrivals, double imu_period_sec);

bool posePassesRtkGate(const fusion_engine_msgs::msg::Pose& msg,
                       double max_var_xy,
                       double max_var_z);

class P1ClockMapper {
public:
  explicit P1ClockMapper(double bin_seconds);

  void addPosePair(double arrival_ros, double p1_time);
  bool ready() const;
  double toRos(double p1_time) const;
  double driftMs() const;

private:
  struct Bin {
    size_t count = 0;
    double center_sum = 0.0;
    double min_lag = std::numeric_limits<double>::infinity();
  };

  static double offsetDrift(const std::vector<std::pair<double, double>>& envelope);

  double bin_seconds_;
  double first_p1_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<Bin> bins_;
};

}  // namespace adapter
