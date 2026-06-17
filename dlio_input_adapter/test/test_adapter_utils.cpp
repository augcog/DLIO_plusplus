#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "dlio_input_adapter/adapter_utils.hpp"

using dlio_input_adapter::P1ClockMapper;

TEST(AdapterUtils, ParsesOrigin)
{
  double e = 0.0;
  double n = 0.0;
  EXPECT_TRUE(dlio_input_adapter::parseOrigin("520000.0,4380000.0", e, n));
  EXPECT_DOUBLE_EQ(e, 520000.0);
  EXPECT_DOUBLE_EQ(n, 4380000.0);
  EXPECT_TRUE(dlio_input_adapter::parseOrigin("520001.5 4380002.5", e, n));
  EXPECT_DOUBLE_EQ(e, 520001.5);
  EXPECT_DOUBLE_EQ(n, 4380002.5);
  EXPECT_FALSE(dlio_input_adapter::parseOrigin("", e, n));
}

TEST(AdapterUtils, MapsConstantP1Offset)
{
  P1ClockMapper mapper(60.0);
  mapper.addPosePair(1779827000.100, 7000.000);
  mapper.addPosePair(1779827001.120, 7001.000);

  EXPECT_TRUE(mapper.ready());
  EXPECT_LT(mapper.driftMs(), 25.0);
  EXPECT_NEAR(mapper.toRos(7002.0), 1779827002.100, 1e-6);
}

TEST(AdapterUtils, InterpolatesP1Drift)
{
  P1ClockMapper mapper(1.0);
  mapper.addPosePair(100.000, 0.000);
  mapper.addPosePair(110.100, 10.000);

  EXPECT_NEAR(mapper.driftMs(), 100.0, 1e-6);
  EXPECT_NEAR(mapper.toRos(5.0), 105.050, 1e-6);
}

TEST(AdapterUtils, RetimesArrivalBurst)
{
  const std::vector<double> arrivals = {1.0, 1.0, 1.0, 1.03};
  const auto out = dlio_input_adapter::retimeArrivalStamps(arrivals, 0.01);
  ASSERT_EQ(out.size(), arrivals.size());
  EXPECT_NEAR(out[0], 0.98, 1e-12);
  EXPECT_NEAR(out[1], 0.99, 1e-12);
  EXPECT_NEAR(out[2], 1.00, 1e-12);
  EXPECT_NEAR(out[3], 1.03, 1e-12);
}

TEST(AdapterUtils, RepairsLuminarUint8TimestampEpoch)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = dlio_input_adapter::secToStamp(10.0);
  cloud.width = 3;
  cloud.height = 1;
  cloud.point_step = 56;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  sensor_msgs::msg::PointField field;
  field.name = "timestamp";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::UINT8;
  field.count = 8;
  cloud.fields.push_back(field);

  const std::vector<uint64_t> raw = {100, 120, 150};
  for (size_t i = 0; i < raw.size(); ++i) {
    std::memcpy(&cloud.data[i * cloud.point_step], &raw[i], sizeof(uint64_t));
  }

  std::string reason;
  EXPECT_TRUE(dlio_input_adapter::repairLuminarPointTimestamps(cloud, &reason)) << reason;

  std::vector<uint64_t> repaired(raw.size(), 0);
  for (size_t i = 0; i < repaired.size(); ++i) {
    std::memcpy(&repaired[i], &cloud.data[i * cloud.point_step], sizeof(uint64_t));
  }
  EXPECT_EQ(repaired[0], 10000000000ULL);
  EXPECT_EQ(repaired[1] - repaired[0], 20ULL);
  EXPECT_EQ(repaired[2] - repaired[0], 50ULL);
}

TEST(AdapterUtils, AppliesRtkGate)
{
  fusion_engine_msgs::msg::Pose pose;
  pose.solution_type = 4;
  pose.position_covariance[0] = 1e-4;
  pose.position_covariance[4] = 2e-4;
  pose.position_covariance[8] = 4e-3;
  EXPECT_TRUE(dlio_input_adapter::posePassesRtkGate(pose, 1e-3, 5e-3));

  pose.position_covariance[4] = 2e-3;
  EXPECT_FALSE(dlio_input_adapter::posePassesRtkGate(pose, 1e-3, 5e-3));
}

TEST(AdapterUtils, TransformsOdomToMapFrame)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "utm";
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.covariance[0] = 1.0;
  odom.pose.covariance[7] = 4.0;
  odom.pose.covariance[14] = 9.0;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = 10.0;
  T(1, 3) = 20.0;
  T(2, 3) = 30.0;

  const auto out = dlio_input_adapter::transformOdomToMap(odom, T);
  EXPECT_EQ(out.header.frame_id, "map");
  EXPECT_NEAR(out.pose.pose.position.x, 8.0, 1e-12);
  EXPECT_NEAR(out.pose.pose.position.y, 21.0, 1e-12);
  EXPECT_NEAR(out.pose.pose.position.z, 33.0, 1e-12);
  EXPECT_NEAR(out.pose.covariance[0], 4.0, 1e-12);
  EXPECT_NEAR(out.pose.covariance[7], 1.0, 1e-12);
  EXPECT_NEAR(out.pose.covariance[14], 9.0, 1e-12);
}
