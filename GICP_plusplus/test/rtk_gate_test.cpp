// Tests for the RTK covariance-quality gate primitives (P1 FIX 2026-07-14):
// a variance component qualifies only when FINITE, NONNEGATIVE, and at most
// the configured limit. The former plain `<= threshold` comparison accepted
// the finite negative sentinel (-1 = "covariance not populated") as
// RTK-quality.

#include <gtest/gtest.h>

#include <limits>

#include "gicp_plusplus/rtk_gate.hpp"

namespace {

constexpr double kMaxXY = 0.25;  // shipped defaults (m^2)
constexpr double kMaxZ = 1.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

TEST(RtkGate, ValidRtkFixedCovariancePasses) {
  // Known-RTK-fixed bag reference values (run_2 medians).
  EXPECT_TRUE(gicp_plusplus::rtkPositionCovarianceOk(2.8e-5, 4.2e-5, 1.0e-4, kMaxXY, kMaxZ));
}

TEST(RtkGate, NegativeSentinelFailsEveryComponent) {
  // -1 = "covariance not populated" must NOT pass as RTK-quality.
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(-1.0, kMaxXY));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(-1.0, 4.2e-5, 1.0e-4, kMaxXY, kMaxZ));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(2.8e-5, -1.0, 1.0e-4, kMaxXY, kMaxZ));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(2.8e-5, 4.2e-5, -1.0, kMaxXY, kMaxZ));
  // Any negative value, not just the -1 sentinel.
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(-1e-12, kMaxXY));
}

TEST(RtkGate, NaNFailsClosed) {
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(kNaN, kMaxXY));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(kNaN, 4.2e-5, 1.0e-4, kMaxXY, kMaxZ));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(2.8e-5, kNaN, 1.0e-4, kMaxXY, kMaxZ));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(2.8e-5, 4.2e-5, kNaN, kMaxXY, kMaxZ));
}

TEST(RtkGate, InfinityFailsClosed) {
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(kInf, kMaxXY));
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(-kInf, kMaxXY));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(2.8e-5, kInf, 1.0e-4, kMaxXY, kMaxZ));
}

TEST(RtkGate, ThresholdBoundaryIsInclusive) {
  // Exactly at the limit passes (<=, matching the documented contract) ...
  EXPECT_TRUE(gicp_plusplus::rtkCovarianceComponentOk(kMaxXY, kMaxXY));
  EXPECT_TRUE(gicp_plusplus::rtkPositionCovarianceOk(kMaxXY, kMaxXY, kMaxZ, kMaxXY, kMaxZ));
  // ... just above fails.
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(std::nextafter(kMaxXY, 1.0), kMaxXY));
}

TEST(RtkGate, ZeroCovarianceFailsClosedByDefault) {
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(0.0, 0.0, 0.0, kMaxXY, kMaxZ));
}

TEST(RtkGate, ZeroCovarianceRequiresExplicitCompatibilityEscape) {
  EXPECT_TRUE(gicp_plusplus::rtkPositionCovarianceOk(
      0.0, 0.0, 0.0, kMaxXY, kMaxZ, true));
}

TEST(RtkGate, PerAxisThresholdsApply) {
  // z uses max_var_z, not max_var_xy.
  EXPECT_TRUE(gicp_plusplus::rtkPositionCovarianceOk(1e-4, 1e-4, 0.9, kMaxXY, kMaxZ));
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(0.9, 1e-4, 1e-4, kMaxXY, kMaxZ));
}

// [P1 FIX 2026-07-14b] Interpolated-covariance combine: a single valid
// bracketing endpoint must never dominate an unknown/invalid one.
TEST(RtkGateInterp, ValidPairReturnsMax) {
  EXPECT_DOUBLE_EQ(gicp_plusplus::rtkCombineInterpolatedVariance(2.8e-5, 1.0e-4), 1.0e-4);
  EXPECT_DOUBLE_EQ(gicp_plusplus::rtkCombineInterpolatedVariance(1.0e-4, 2.8e-5), 1.0e-4);
  EXPECT_DOUBLE_EQ(gicp_plusplus::rtkCombineInterpolatedVariance(0.0, 0.0), 0.0);
}

TEST(RtkGateInterp, NegativeSentinelEitherOrderFailsClosed) {
  // std::max(valid, -1) == valid was the bypass; both argument orders must
  // now yield +inf (which rtkCovarianceComponentOk rejects).
  const double v1 = gicp_plusplus::rtkCombineInterpolatedVariance(2.8e-5, -1.0);
  const double v2 = gicp_plusplus::rtkCombineInterpolatedVariance(-1.0, 2.8e-5);
  EXPECT_TRUE(std::isinf(v1) && v1 > 0.0);
  EXPECT_TRUE(std::isinf(v2) && v2 > 0.0);
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(v1, kMaxXY));
  EXPECT_FALSE(gicp_plusplus::rtkCovarianceComponentOk(v2, kMaxXY));
}

TEST(RtkGateInterp, NaNEitherOrderFailsClosed) {
  // std::max(valid, NaN) returns the valid FIRST argument under C++ max
  // semantics; both orders must now yield +inf.
  const double v1 = gicp_plusplus::rtkCombineInterpolatedVariance(2.8e-5, kNaN);
  const double v2 = gicp_plusplus::rtkCombineInterpolatedVariance(kNaN, 2.8e-5);
  EXPECT_TRUE(std::isinf(v1) && v1 > 0.0);
  EXPECT_TRUE(std::isinf(v2) && v2 > 0.0);
}

TEST(RtkGateInterp, InfinityAndBothInvalidFailClosed) {
  EXPECT_TRUE(std::isinf(gicp_plusplus::rtkCombineInterpolatedVariance(kInf, 2.8e-5)));
  EXPECT_TRUE(std::isinf(gicp_plusplus::rtkCombineInterpolatedVariance(2.8e-5, kInf)));
  EXPECT_TRUE(std::isinf(gicp_plusplus::rtkCombineInterpolatedVariance(-1.0, kNaN)));
  EXPECT_TRUE(std::isinf(gicp_plusplus::rtkCombineInterpolatedVariance(kNaN, -1.0)));
}

TEST(RtkGateInterp, CombinedWithGateEndToEnd) {
  // Interpolating between an RTK-FIXED sample and an unpopulated (-1) sample
  // must NOT qualify as RTK, in either bracketing order.
  const double xx = gicp_plusplus::rtkCombineInterpolatedVariance(2.8e-5, -1.0);
  const double yy = gicp_plusplus::rtkCombineInterpolatedVariance(4.2e-5, 4.2e-5);
  const double zz = gicp_plusplus::rtkCombineInterpolatedVariance(1.0e-4, 1.0e-4);
  EXPECT_FALSE(gicp_plusplus::rtkPositionCovarianceOk(xx, yy, zz, kMaxXY, kMaxZ));
  // Both endpoints valid RTK -> passes.
  const double xx_ok = gicp_plusplus::rtkCombineInterpolatedVariance(2.8e-5, 3.0e-5);
  EXPECT_TRUE(gicp_plusplus::rtkPositionCovarianceOk(xx_ok, yy, zz, kMaxXY, kMaxZ));
}

}  // namespace
