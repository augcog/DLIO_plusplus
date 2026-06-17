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
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace dlio_input_adapter {

double stampToSec(const builtin_interfaces::msg::Time& stamp);
int64_t stampToNs(const builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time secToStamp(double sec);
double p1ToSec(const fusion_engine_msgs::msg::Timestamp& stamp);
Eigen::Quaterniond rpyToQuat(double roll_deg, double pitch_deg, double yaw_deg);

bool parseOrigin(const std::string& text, double& easting, double& northing);
bool readMatrix4(const std::string& path, Eigen::Matrix4d& matrix);

const sensor_msgs::msg::PointField* findTimeField(const sensor_msgs::msg::PointCloud2& msg);
bool repairLuminarPointTimestamps(sensor_msgs::msg::PointCloud2& msg, std::string* reason = nullptr);

std::vector<double> retimeArrivalStamps(const std::vector<double>& arrivals, double imu_period_sec);

bool posePassesRtkGate(const fusion_engine_msgs::msg::Pose& msg,
                       double max_var_xy,
                       double max_var_z);

nav_msgs::msg::Odometry transformOdomToMap(const nav_msgs::msg::Odometry& odom,
                                           const Eigen::Matrix4d& t_world_utm);

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

}  // namespace dlio_input_adapter
