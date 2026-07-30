#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace glim {
namespace gnss_detail {

struct AlignmentWindow {
  bool ready = false;
  size_t begin = 0;
  size_t training_end = 0;
  size_t end = 0;
  double estimate_baseline = 0.0;
  double gnss_baseline = 0.0;

  size_t training_count() const { return training_end - begin; }
  size_t validation_count() const { return end - training_end; }
};

struct AlignmentFit {
  bool valid = false;
  Eigen::Isometry3d T_gnss_estimate = Eigen::Isometry3d::Identity();
  double training_rms = std::numeric_limits<double>::infinity();
  double validation_rms = std::numeric_limits<double>::infinity();
};

inline AlignmentWindow select_alignment_window(
  const std::vector<Eigen::Vector3d>& estimates,
  const std::vector<Eigen::Vector3d>& gnss,
  double min_baseline,
  size_t min_training_samples,
  size_t validation_samples,
  bool recent_window) {
  AlignmentWindow window;
  if (
    estimates.size() != gnss.size() || min_training_samples < 3 ||
    validation_samples < 1 ||
    estimates.size() < min_training_samples + validation_samples) {
    return window;
  }

  window.begin = 0;
  window.training_end = estimates.size() - validation_samples;
  window.end = estimates.size();

  const auto baselines_from = [&](size_t begin) {
    const size_t last_training = window.training_end - 1;
    return std::pair<double, double>{
      (estimates[last_training] - estimates[begin]).norm(),
      (gnss[last_training] - gnss[begin]).norm()};
  };

  auto baselines = baselines_from(window.begin);
  if (baselines.first <= min_baseline || baselines.second <= min_baseline) {
    return window;
  }

  if (recent_window) {
    while (window.begin + 1 + min_training_samples <= window.training_end) {
      const auto candidate_baselines = baselines_from(window.begin + 1);
      if (
        candidate_baselines.first <= min_baseline ||
        candidate_baselines.second <= min_baseline) {
        break;
      }
      ++window.begin;
      baselines = candidate_baselines;
    }
  }

  window.estimate_baseline = baselines.first;
  window.gnss_baseline = baselines.second;
  window.ready = true;
  return window;
}

inline double alignment_rms(
  const std::vector<Eigen::Vector3d>& estimates,
  const std::vector<Eigen::Vector3d>& gnss,
  const Eigen::Isometry3d& T_gnss_estimate,
  size_t begin,
  size_t end) {
  if (begin >= end || end > estimates.size() || estimates.size() != gnss.size()) {
    return std::numeric_limits<double>::infinity();
  }

  double sum_sq = 0.0;
  for (size_t i = begin; i < end; ++i) {
    const Eigen::Vector3d prediction = T_gnss_estimate * estimates[i];
    sum_sq += (prediction - gnss[i]).squaredNorm();
  }
  return std::sqrt(sum_sq / static_cast<double>(end - begin));
}

inline AlignmentFit fit_planar_alignment(
  const std::vector<Eigen::Vector3d>& estimates,
  const std::vector<Eigen::Vector3d>& gnss,
  const AlignmentWindow& window) {
  AlignmentFit fit;
  if (
    !window.ready || estimates.size() != gnss.size() ||
    window.training_count() < 3 || window.validation_count() < 1 ||
    window.end > estimates.size()) {
    return fit;
  }

  Eigen::Vector3d mean_estimate = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gnss = Eigen::Vector3d::Zero();
  for (size_t i = window.begin; i < window.training_end; ++i) {
    mean_estimate += estimates[i];
    mean_gnss += gnss[i];
  }
  mean_estimate /= static_cast<double>(window.training_count());
  mean_gnss /= static_cast<double>(window.training_count());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t i = window.begin; i < window.training_end; ++i) {
    covariance +=
      (gnss[i] - mean_gnss) * (estimates[i] - mean_estimate).transpose();
  }
  covariance /= static_cast<double>(window.training_count());

  const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
    covariance.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix2d U = svd.matrixU();
  const Eigen::Matrix2d V = svd.matrixV();
  Eigen::Matrix2d reflection = Eigen::Matrix2d::Identity();
  if (U.determinant() * V.determinant() < 0.0) {
    reflection(1, 1) = -1.0;
  }

  fit.T_gnss_estimate.linear().block<2, 2>(0, 0) =
    U * reflection * V.transpose();
  fit.T_gnss_estimate.translation() =
    mean_gnss - fit.T_gnss_estimate.linear() * mean_estimate;
  fit.training_rms = alignment_rms(
    estimates, gnss, fit.T_gnss_estimate, window.begin, window.training_end);
  fit.validation_rms = alignment_rms(
    estimates, gnss, fit.T_gnss_estimate, window.training_end, window.end);
  fit.valid =
    fit.T_gnss_estimate.matrix().allFinite() &&
    std::isfinite(fit.training_rms) && std::isfinite(fit.validation_rms);
  return fit;
}

}  // namespace gnss_detail
}  // namespace glim
