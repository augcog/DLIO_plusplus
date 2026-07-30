#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <glim_ext/gnss_alignment.hpp>
#include <glim_ext/gnss_factor_delivery.hpp>

namespace {

using glim::gnss_detail::fit_planar_alignment;
using glim::gnss_detail::select_alignment_window;

TEST(GNSSAlignment, WaitsForTrainingAndValidationSamples) {
  std::vector<Eigen::Vector3d> estimate(29, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> gnss(29, Eigen::Vector3d::Zero());
  for (size_t i = 0; i < estimate.size(); ++i) {
    estimate[i].x() = static_cast<double>(i);
    gnss[i].x() = static_cast<double>(i);
  }

  const auto window =
    select_alignment_window(estimate, gnss, 5.0, 20, 10, true);
  EXPECT_FALSE(window.ready);
}

TEST(GNSSAlignment, RecentWindowRetainsMinimumTrainingSamples) {
  std::vector<Eigen::Vector3d> estimate(80, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> gnss(80, Eigen::Vector3d::Zero());
  for (size_t i = 0; i < estimate.size(); ++i) {
    estimate[i].x() = static_cast<double>(i);
    gnss[i].x() = static_cast<double>(i);
  }

  const auto window =
    select_alignment_window(estimate, gnss, 5.0, 20, 10, true);
  ASSERT_TRUE(window.ready);
  EXPECT_GE(window.training_count(), 20u);
  EXPECT_EQ(window.validation_count(), 10u);
  EXPECT_EQ(window.training_count(), 20u);
}

TEST(GNSSAlignment, HoldoutRejectsGrowingHeadingDrift) {
  constexpr size_t kCount = 60;
  std::vector<Eigen::Vector3d> estimate(kCount, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> gnss(kCount, Eigen::Vector3d::Zero());

  double x = 0.0;
  double y = 0.0;
  for (size_t i = 0; i < kCount; ++i) {
    const double distance = 5.0 * static_cast<double>(i) /
                            static_cast<double>(kCount - 1);
    const double heading_error = 0.50 * distance / 5.0;
    if (i > 0) {
      const double step = 5.0 / static_cast<double>(kCount - 1);
      x += step * std::cos(heading_error);
      y += step * std::sin(heading_error);
    }
    estimate[i] = Eigen::Vector3d(x, y, 0.0);
    gnss[i] = Eigen::Vector3d(distance, 0.0, 0.0);
  }

  const auto window =
    select_alignment_window(estimate, gnss, 2.5, 20, 10, false);
  ASSERT_TRUE(window.ready);
  const auto fit = fit_planar_alignment(estimate, gnss, window);
  ASSERT_TRUE(fit.valid);
  EXPECT_LT(fit.training_rms, 0.25);
  EXPECT_GT(fit.validation_rms, 0.25);
}

TEST(GNSSAlignment, StableRigidTransformPassesTrainingAndHoldout) {
  constexpr size_t kCount = 50;
  std::vector<Eigen::Vector3d> estimate(kCount, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> gnss(kCount, Eigen::Vector3d::Zero());
  const Eigen::Rotation2Dd rotation(0.2);
  for (size_t i = 0; i < kCount; ++i) {
    estimate[i] =
      Eigen::Vector3d(0.25 * i, 0.02 * std::sin(0.2 * i), 0.01 * i);
    gnss[i].head<2>() =
      rotation * estimate[i].head<2>() + Eigen::Vector2d(3.0, -2.0);
    gnss[i].z() = estimate[i].z() + 0.4;
  }

  const auto window =
    select_alignment_window(estimate, gnss, 5.0, 20, 10, false);
  ASSERT_TRUE(window.ready);
  const auto fit = fit_planar_alignment(estimate, gnss, window);
  ASSERT_TRUE(fit.valid);
  EXPECT_LT(fit.training_rms, 1.0e-10);
  EXPECT_LT(fit.validation_rms, 1.0e-10);
}

TEST(GNSSFactorDelivery, ConfirmsExactFactorIdentities) {
  int factor_a = 1;
  int factor_b = 2;
  int other = 3;
  const std::vector<const void*> graph = {&other, &factor_a, &factor_b};
  const std::vector<size_t> indices = {0, 1, 2};
  const std::vector<const void*> expected = {&factor_a, &factor_b};

  EXPECT_TRUE(glim::gnss_detail::factor_batch_committed(
    1, expected, indices, graph.size(),
    [&](size_t index) { return graph[index]; }));
}

TEST(GNSSFactorDelivery, RejectsMissingOrDifferentFactor) {
  int factor_a = 1;
  int factor_b = 2;
  int replacement = 3;
  const std::vector<const void*> graph = {&factor_a, &replacement};
  const std::vector<size_t> indices = {0, 1};
  const std::vector<const void*> expected = {&factor_a, &factor_b};

  EXPECT_FALSE(glim::gnss_detail::factor_batch_committed(
    0, expected, indices, graph.size(),
    [&](size_t index) { return graph[index]; }));
  EXPECT_FALSE(glim::gnss_detail::factor_batch_committed(
    1, expected, indices, graph.size(),
    [&](size_t index) { return graph[index]; }));
}

}  // namespace
