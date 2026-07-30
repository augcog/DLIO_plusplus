#include <gtest/gtest.h>

#include <deque>
#include <limits>
#include <vector>

#include "gicp_plusplus/imu_range.hpp"

namespace {

struct Sample {
  double stamp;
  int id;
};

using Buffer = std::deque<Sample>;

TEST(ImuRange, ReturnsNearestBracketsInForwardOrder) {
  const Buffer buffer{{4.0, 4}, {3.0, 3}, {2.0, 2}, {1.0, 1}};
  std::vector<Sample> out;
  ASSERT_TRUE(gicp_plusplus::selectBracketedImuRange(
      buffer, 1.5, 3.2, 0.002, out));
  ASSERT_EQ(out.size(), 4U);
  EXPECT_EQ(out.front().id, 1);
  EXPECT_EQ(out.back().id, 4);
}

TEST(ImuRange, ExactBoundaryStillReturnsTwoSamples) {
  const Buffer buffer{{3.0, 3}, {2.0, 2}, {1.0, 1}};
  std::vector<Sample> out;
  ASSERT_TRUE(gicp_plusplus::selectBracketedImuRange(
      buffer, 2.0, 2.0, 0.002, out));
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].id, 2);
  EXPECT_EQ(out[1].id, 3);
}

TEST(ImuRange, AllowsBoundedStartSideExtrapolation) {
  const Buffer buffer{{2.0, 2}, {1.001, 1}};
  std::vector<Sample> out;
  EXPECT_TRUE(gicp_plusplus::selectBracketedImuRange(
      buffer, 1.0, 2.0, 0.002, out));
}

TEST(ImuRange, RejectsMissingOlderOrNewerBracket) {
  std::vector<Sample> out;
  EXPECT_FALSE(gicp_plusplus::selectBracketedImuRange(
      Buffer{{2.0, 2}, {1.003, 1}}, 1.0, 2.0, 0.002, out));
  EXPECT_FALSE(gicp_plusplus::selectBracketedImuRange(
      Buffer{{1.9, 2}, {1.0, 1}}, 1.0, 2.0, 0.002, out));
}

TEST(ImuRange, RejectsEmptyInvalidAndNonMonotoneInputs) {
  std::vector<Sample> out;
  EXPECT_FALSE(gicp_plusplus::selectBracketedImuRange(
      Buffer{}, 1.0, 2.0, 0.002, out));
  EXPECT_FALSE(gicp_plusplus::selectBracketedImuRange(
      Buffer{{2.0, 2}, {1.0, 1}}, 2.0, 1.0, 0.002, out));
  EXPECT_FALSE(gicp_plusplus::selectBracketedImuRange(
      Buffer{{2.0, 2}, {2.0, 1}}, 1.0, 2.0, 0.002, out));
  EXPECT_FALSE(gicp_plusplus::selectBracketedImuRange(
      Buffer{{2.0, 2}, {1.0, 1}},
      std::numeric_limits<double>::quiet_NaN(), 2.0, 0.002, out));
}

}  // namespace
