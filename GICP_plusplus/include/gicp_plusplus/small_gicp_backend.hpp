#ifndef GICP_PLUSPLUS_SMALL_GICP_BACKEND_HPP
#define GICP_PLUSPLUS_SMALL_GICP_BACKEND_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_proxy.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_result.hpp>
#include <small_gicp/registration/termination_criteria.hpp>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace gicp_plusplus {

inline Eigen::Vector3d so3LogVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  Eigen::Vector3d axis = aa.axis();
  if (!axis.allFinite() || std::abs(aa.angle()) < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return aa.angle() * axis;
}

struct GroundVehicleGeneralFactor {
  GroundVehicleGeneralFactor() {
    dof_mask.setOnes();
    rotation_prior_R.setIdentity();
    rotation_prior_info.setZero();
  }

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree>
  void update_linearized_system(
      const TargetPointCloud&,
      const SourcePointCloud&,
      const TargetTree&,
      const Eigen::Isometry3d& T,
      Eigen::Matrix<double, 6, 6>* H,
      Eigen::Matrix<double, 6, 1>* b,
      double* e) const {
    if (!H || !b || !e) {
      return;
    }

    // small_gicp uses right-multiplicative se(3) perturbations ordered
    // [rx, ry, rz, tx, ty, tz]. Soft rotation prior first, exact mask last
    // (so a fixed axis stays fixed even where the prior touches it).
    if ((rotation_prior_info.array() > 0.0).any()) {
      const Eigen::Vector3d r = so3LogVector(rotation_prior_R.transpose() * T.linear());
      const Eigen::Matrix3d W = rotation_prior_info.asDiagonal();
      H->template block<3, 3>(0, 0) += W;
      b->template head<3>() += W * r;
      *e += 0.5 * r.transpose() * W * r;
    }

    // [REVIEW FIX 2026-07-08 P2/P3] EXACT delta masking, not soft damping.
    // The previous dof_lambda=1e9 diagonal boost only shrank the masked
    // increments; there was no residual back to the initial guess and no
    // exact zeroing, so the "fixed" axes could still creep across LM
    // iterations — and 3dof was not a hard yaw lock even though the caller
    // documents it as one. Zeroing the masked rows/cols of H and entries of
    // b (with the diagonal pinned for conditioning) makes each LM step's
    // delta EXACTLY zero on those axes: with right-multiplicative
    // perturbations the masked DoF then hold the values of init_T for the
    // whole solve, which is precisely the documented "fixed to the IMU
    // prior" contract.
    if ((dof_mask.array() < 1.0).any()) {
      const double pin = std::max(1.0, H->diagonal().cwiseAbs().maxCoeff());
      for (int axis = 0; axis < 6; ++axis) {
        if (dof_mask(axis) >= 1.0) continue;
        H->row(axis).setZero();
        H->col(axis).setZero();
        (*H)(axis, axis) = pin;
        (*b)(axis) = 0.0;
      }
    }
  }

  template <typename TargetPointCloud, typename SourcePointCloud>
  void update_error(
      const TargetPointCloud&,
      const SourcePointCloud&,
      const Eigen::Isometry3d& T,
      double* e) const {
    if (!e || !(rotation_prior_info.array() > 0.0).any()) {
      return;
    }
    const Eigen::Vector3d r = so3LogVector(rotation_prior_R.transpose() * T.linear());
    *e += 0.5 * r.transpose() * rotation_prior_info.asDiagonal() * r;
  }

  Eigen::Array<double, 6, 1> dof_mask;
  Eigen::Matrix3d rotation_prior_R;
  Eigen::Vector3d rotation_prior_info;
};

struct PriorAwareLevenbergMarquardtOptimizer {
  PriorAwareLevenbergMarquardtOptimizer()
  : verbose(false),
    max_iterations(20),
    max_inner_iterations(10),
    max_time_ms(0.0),
    timeout_flag(nullptr),
    init_lambda(1e-3),
    lambda_factor(10.0) {}

  template <
      typename TargetPointCloud,
      typename SourcePointCloud,
      typename TargetTree,
      typename CorrespondenceRejector,
      typename TerminationCriteria,
      typename Reduction,
      typename Factor,
      typename GeneralFactor>
  small_gicp::RegistrationResult optimize(
      const TargetPointCloud& target,
      const SourcePointCloud& source,
      const TargetTree& target_tree,
      const CorrespondenceRejector& rejector,
      const TerminationCriteria& criteria,
      Reduction& reduction,
      const Eigen::Isometry3d& init_T,
      std::vector<Factor>& factors,
      GeneralFactor& general_factor) const {
    if (verbose) {
      std::cout << "--- small_gicp prior-aware LM optimization ---" << std::endl;
    }

    double lambda = init_lambda;
    small_gicp::RegistrationResult result(init_T);
    if (timeout_flag) {
      *timeout_flag = false;
    }
    const auto start = std::chrono::steady_clock::now();
    const auto deadline_exceeded = [&]() {
      if (max_time_ms <= 0.0) {
        return false;
      }
      const double elapsed_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - start)
              .count();
      return elapsed_ms >= max_time_ms;
    };
    const auto mark_timeout = [&]() {
      if (timeout_flag) {
        *timeout_flag = true;
      }
    };

    for (int i = 0; i < max_iterations && !result.converged; ++i) {
      if (deadline_exceeded()) {
        mark_timeout();
        break;
      }
      auto [H, b, e] = reduction.linearize(
          target, source, target_tree, rejector, result.T_target_source, factors);
      result.iterations = static_cast<size_t>(i);
      result.H = H;
      result.b = b;
      result.error = e;
      if (deadline_exceeded()) {
        mark_timeout();
        break;
      }
      general_factor.update_linearized_system(
          target, source, target_tree, result.T_target_source, &H, &b, &e);

      bool success = false;
      for (int j = 0; j < max_inner_iterations; ++j) {
        if (deadline_exceeded()) {
          mark_timeout();
          break;
        }
        const Eigen::Matrix<double, 6, 1> delta =
            (H + lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(-b);
        const Eigen::Isometry3d new_T = result.T_target_source * small_gicp::se3_exp(delta);

        double new_e = reduction.error(target, source, new_T, factors);
        general_factor.update_error(target, source, new_T, &new_e);

        if (verbose) {
          std::cout << "iter=" << i << " inner=" << j
                    << " e=" << e << " new_e=" << new_e
                    << " lambda=" << lambda
                    << " dt=" << delta.tail<3>().norm()
                    << " dr=" << delta.head<3>().norm()
                    << std::endl;
        }

        if (new_e <= e) {
          result.converged = criteria.converged(delta);
          result.T_target_source = new_T;
          lambda /= lambda_factor;
          success = true;
          e = new_e;
          break;
        }

        lambda *= lambda_factor;
      }

      if (timeout_flag && *timeout_flag) {
        break;
      }
      result.iterations = static_cast<size_t>(i);
      result.H = H;
      result.b = b;
      result.error = e;

      if (!success) {
        break;
      }
    }

    // [REVIEW FIX 2026-07-08 P3] Re-linearize at the FINAL pose so
    // result.H / result.b / result.error describe the accepted output, not
    // the linearization point BEFORE the last accepted step (with bounded
    // non-converged accepts, that step is exactly where it may not be tiny).
    //
    // [REVIEW FIX 2026-07-08 P1] Deliberately WITHOUT the general factor:
    // result.H must be the raw LiDAR-geometry Hessian. The ground-vehicle
    // factor pins the masked axes (exact delta masking) and the rotation
    // prior adds its information matrix — with the shipped 4dof default that
    // artificial roll/pitch stiffness would dominate lambda_max, distort the
    // relFloor* degeneracy floors, and make "well-constrained" axes reflect
    // the prior/DoF mask instead of map evidence in hessian_condition, the
    // eigen-projection, and yaw_marginal_stiffness. Likewise result.error
    // stays pure point residual, so fitness (= error / num_inliers) measures
    // map agreement, not prior disagreement. The augmented system exists only
    // inside the LM iterations above.
    if (deadline_exceeded()) {
      mark_timeout();
    }
    if (!(timeout_flag && *timeout_flag)) {
      auto [H_final, b_final, e_final] = reduction.linearize(
          target, source, target_tree, rejector, result.T_target_source, factors);
      if (deadline_exceeded()) {
        mark_timeout();
      } else {
        result.H = H_final;
        result.b = b_final;
        result.error = e_final;
      }
    }

    result.num_inliers = static_cast<size_t>(std::count_if(
        factors.begin(), factors.end(), [](const auto& factor) { return factor.inlier(); }));
    return result;
  }

  bool verbose;
  int max_iterations;
  int max_inner_iterations;
  double max_time_ms;
  bool* timeout_flag;
  double init_lambda;
  double lambda_factor;
};

template <typename PointSource, typename PointTarget>
class SmallGicpBackend {
 public:
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  SmallGicpBackend()
  : num_threads_(1),
    k_correspondences_(20),
    max_corr_dist_(1.0),
    max_iterations_(20),
    max_optimization_time_ms_(0.0),
    transformation_epsilon_(1e-3),
    rotation_epsilon_(0.1 * 3.14159265358979323846 / 180.0),
    debug_print_(false),
    timed_out_(false),
    has_rotation_prior_(false),
    final_transformation_(Eigen::Matrix4f::Identity()),
    final_fitness_(std::numeric_limits<double>::infinity()),
    num_correspondences(0) {
    dof_mask_.setOnes();
    rotation_prior_R_.setIdentity();
    rotation_prior_info_.setZero();
  }

  void setNumThreads(int n) { num_threads_ = std::max(1, n); }
  void setCorrespondenceRandomness(int k) { k_correspondences_ = std::max(5, k); }
  void setMaxCorrespondenceDistance(double corr) { max_corr_dist_ = std::max(0.0, corr); }
  void setMaximumIterations(int iter) { max_iterations_ = std::max(1, iter); }
  void setMaximumOptimizationTimeMs(double time_ms) {
    max_optimization_time_ms_ = std::max(0.0, time_ms);
  }
  void setTransformationEpsilon(double eps) { transformation_epsilon_ = std::max(0.0, eps); }
  void setRotationEpsilon(double eps) { rotation_epsilon_ = std::max(0.0, eps); }
  void setDebugPrint(bool enabled) { debug_print_ = enabled; }

  void setInputTarget(const PointCloudTargetConstPtr& cloud) {
    if (target_ == cloud && target_tree_) {
      return;
    }
    target_ = cloud;
    target_covs_.clear();
    if (target_ && !target_->empty()) {
      target_tree_ = std::make_shared<small_gicp::KdTree<PointCloudTarget>>(
          target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    } else {
      target_tree_.reset();
    }
  }

  void setInputSource(const PointCloudSourceConstPtr& cloud) {
    input_ = cloud;
    source_covs_.clear();
    source_tree_.reset();
  }

  bool calculateTargetCovariances() {
    if (!target_ || target_->empty() || !target_tree_) {
      return false;
    }
    target_covs_.clear();
    small_gicp::PointCloudProxy<PointTarget> target_proxy(*target_, target_covs_);
    small_gicp::estimate_covariances_omp(
        target_proxy, *target_tree_, k_correspondences_, num_threads_);
    return target_covs_.size() == target_->size();
  }

  void setDoFMask(bool fix_roll, bool fix_pitch, bool fix_yaw) {
    dof_mask_.setOnes();
    if (fix_roll) dof_mask_(0) = 0.0;
    if (fix_pitch) dof_mask_(1) = 0.0;
    if (fix_yaw) dof_mask_(2) = 0.0;
  }

  void setRotationPrior(const Eigen::Matrix3d& R_target, const Eigen::Vector3d& info) {
    has_rotation_prior_ = true;
    rotation_prior_R_ = R_target;
    rotation_prior_info_ = info.cwiseMax(Eigen::Vector3d::Zero());
  }

  void clearRotationPrior() {
    has_rotation_prior_ = false;
    rotation_prior_info_.setZero();
  }

  // [REVIEW FIX 2026-07-08 P1] Evaluate the pure point-residual fitness
  // (error / inliers) and correspondence support at an ARBITRARY pose in the
  // same solution space align() used. Needed because degeneracy projection /
  // the yaw veto can modify the applied pose AFTER the solve: gating that
  // modified pose on the raw optimizer fitness would validate a pose nobody
  // is applying. Requires a prior align() on the same source/target (reuses
  // its covariances); returns false when evaluation is impossible.
  bool evaluateFitnessAt(const Eigen::Matrix4f& T, double* fitness, int* inliers) {
    if (!target_ || target_->empty() || !target_tree_ || !input_ || input_->empty() ||
        source_covs_.size() != input_->size() || target_covs_.size() != target_->size() ||
        !T.allFinite()) {
      return false;
    }
    small_gicp::PointCloudProxy<PointSource> source_proxy(*input_, source_covs_);
    small_gicp::PointCloudProxy<PointTarget> target_proxy(*target_, target_covs_);
    std::vector<small_gicp::GICPFactor> factors(input_->size());
    small_gicp::ParallelReductionOMP reduction;
    reduction.num_threads = num_threads_;
    small_gicp::DistanceRejector rejector;
    rejector.max_dist_sq = max_corr_dist_ * max_corr_dist_;
    const auto [H, b, e] = reduction.linearize(
        target_proxy, source_proxy, *target_tree_, rejector,
        Eigen::Isometry3d(T.cast<double>()), factors);
    (void)H;
    (void)b;
    const size_t n = static_cast<size_t>(std::count_if(
        factors.begin(), factors.end(), [](const auto& f) { return f.inlier(); }));
    if (inliers) *inliers = static_cast<int>(n);
    const double f = (n > 0) ? e / static_cast<double>(n)
                             : std::numeric_limits<double>::infinity();
    if (fitness) {
      *fitness = std::isfinite(f) ? f : std::numeric_limits<double>::infinity();
    }
    // [REVIEW FIX 2026-07-08 P2] Non-finite error/fitness = evaluation
    // failed; callers treat `false` as fail-closed (fitness forced to +inf).
    return n > 0 && std::isfinite(f);
  }

  void align(PointCloudSource& output, const Eigen::Matrix4f& guess) {
    const auto cooperative_budget_start = std::chrono::steady_clock::now();
    source_tree_ms_ = 0.0;
    source_covariance_ms_ = 0.0;
    target_covariance_ms_ = 0.0;
    optimizer_ms_ = 0.0;
    timeout_stage_ = "none";
    converged_ = false;
    final_transformation_ = guess;
    final_fitness_ = std::numeric_limits<double>::infinity();
    final_error_ = std::numeric_limits<double>::infinity();
    num_correspondences = 0;
    timed_out_ = false;
    result_ = small_gicp::RegistrationResult(Eigen::Isometry3d(guess.cast<double>()));

    if (!target_ || target_->empty() || !target_tree_ || !input_ || input_->empty()) {
      output.clear();
      return;
    }
    const auto elapsed_budget_ms = [&]() {
      return std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - cooperative_budget_start)
          .count();
    };
    const auto budget_exhausted = [&]() {
      return max_optimization_time_ms_ > 0.0 &&
             elapsed_budget_ms() >= max_optimization_time_ms_;
    };
    const auto fail_timeout = [&](const char* stage) {
      timed_out_ = true;
      timeout_stage_ = stage;
      converged_ = false;
      final_transformation_ = guess;
      final_error_ = std::numeric_limits<double>::infinity();
      final_fitness_ = std::numeric_limits<double>::infinity();
      num_correspondences = 0;
      output.clear();
    };

    small_gicp::PointCloudProxy<PointSource> source_proxy(*input_, source_covs_);
    small_gicp::PointCloudProxy<PointTarget> target_proxy(*target_, target_covs_);

    if (!source_tree_) {
      const auto stage_start = std::chrono::steady_clock::now();
      source_tree_ = std::make_shared<small_gicp::KdTree<PointCloudSource>>(
          input_, small_gicp::KdTreeBuilderOMP(num_threads_));
      source_tree_ms_ = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - stage_start)
                            .count();
    }
    if (budget_exhausted()) {
      fail_timeout("source_tree");
      return;
    }
    if (source_covs_.size() != input_->size()) {
      const auto stage_start = std::chrono::steady_clock::now();
      small_gicp::estimate_covariances_omp(
          source_proxy, *source_tree_, k_correspondences_, num_threads_);
      source_covariance_ms_ =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - stage_start)
              .count();
    }
    if (budget_exhausted()) {
      fail_timeout("source_covariance");
      return;
    }
    if (target_covs_.size() != target_->size()) {
      const auto stage_start = std::chrono::steady_clock::now();
      small_gicp::estimate_covariances_omp(
          target_proxy, *target_tree_, k_correspondences_, num_threads_);
      target_covariance_ms_ =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - stage_start)
              .count();
    }
    if (budget_exhausted()) {
      fail_timeout("target_covariance");
      return;
    }

    GroundVehicleGeneralFactor general_factor;
    general_factor.dof_mask = dof_mask_;
    if (has_rotation_prior_) {
      general_factor.rotation_prior_R = rotation_prior_R_;
      general_factor.rotation_prior_info = rotation_prior_info_;
    }

    small_gicp::Registration<
        small_gicp::GICPFactor,
        small_gicp::ParallelReductionOMP,
        GroundVehicleGeneralFactor,
        small_gicp::DistanceRejector,
        PriorAwareLevenbergMarquardtOptimizer>
        registration;
    registration.criteria.rotation_eps = rotation_epsilon_;
    registration.criteria.translation_eps = transformation_epsilon_;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq = max_corr_dist_ * max_corr_dist_;
    registration.optimizer.verbose = debug_print_;
    registration.optimizer.max_iterations = max_iterations_;
    registration.optimizer.max_time_ms =
        max_optimization_time_ms_ > 0.0
            ? max_optimization_time_ms_ - elapsed_budget_ms()
            : 0.0;
    if (max_optimization_time_ms_ > 0.0 &&
        registration.optimizer.max_time_ms <= 0.0) {
      fail_timeout("pre_optimizer");
      return;
    }
    registration.optimizer.timeout_flag = &timed_out_;
    registration.general_factor = general_factor;

    const auto optimizer_start = std::chrono::steady_clock::now();
    result_ = registration.align(
        target_proxy, source_proxy, *target_tree_, Eigen::Isometry3d(guess.cast<double>()));
    optimizer_ms_ = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - optimizer_start)
                        .count();

    if (timed_out_) {
      timeout_stage_ = "optimizer";
      result_ = small_gicp::RegistrationResult(
          Eigen::Isometry3d(guess.cast<double>()));
      converged_ = false;
      final_transformation_ = guess;
      final_error_ = std::numeric_limits<double>::infinity();
      num_correspondences = 0;
      final_fitness_ = std::numeric_limits<double>::infinity();
      output.clear();
      return;
    }

    converged_ = result_.converged;
    final_transformation_ = result_.T_target_source.matrix().cast<float>();
    final_error_ = result_.error;
    num_correspondences = static_cast<int>(result_.num_inliers);
    final_fitness_ = num_correspondences > 0
        ? result_.error / static_cast<double>(num_correspondences)
        : std::numeric_limits<double>::infinity();
    // [REVIEW FIX 2026-07-08 P2] NaN error (degenerate covariances, NaN
    // points) must fail CLOSED: a NaN fitness makes every `>` threshold
    // comparison false downstream, silently accepting the scan.
    if (!std::isfinite(final_fitness_)) {
      final_fitness_ = std::numeric_limits<double>::infinity();
    }

    pcl::transformPointCloud(*input_, output, final_transformation_);
  }

  double getFitnessScore(double = std::numeric_limits<double>::max()) const {
    return final_fitness_;
  }

  double getFitnessScoreAtFinal(double = std::numeric_limits<double>::max()) const {
    return final_fitness_;
  }

  double getFinalError() const { return final_error_; }
  bool hasConverged() const { return converged_; }
  bool hasTimedOut() const { return timed_out_; }
  double getSourceTreeMs() const { return source_tree_ms_; }
  double getSourceCovarianceMs() const { return source_covariance_ms_; }
  double getTargetCovarianceMs() const { return target_covariance_ms_; }
  double getOptimizerMs() const { return optimizer_ms_; }
  const char* getTimeoutStage() const { return timeout_stage_; }
  const Eigen::Matrix<double, 6, 6>& getFinalHessian() const { return result_.H; }
  Eigen::Matrix4f getFinalTransformation() const { return final_transformation_; }
  const small_gicp::RegistrationResult& getRegistrationResult() const { return result_; }

  int num_correspondences;

 private:
  int num_threads_;
  int k_correspondences_;
  double max_corr_dist_;
  int max_iterations_;
  double max_optimization_time_ms_;
  double transformation_epsilon_;
  double rotation_epsilon_;
  bool debug_print_;
  bool timed_out_;
  double source_tree_ms_ = 0.0;
  double source_covariance_ms_ = 0.0;
  double target_covariance_ms_ = 0.0;
  double optimizer_ms_ = 0.0;
  const char* timeout_stage_ = "none";

  PointCloudSourceConstPtr input_;
  PointCloudTargetConstPtr target_;
  std::shared_ptr<small_gicp::KdTree<PointCloudSource>> source_tree_;
  std::shared_ptr<small_gicp::KdTree<PointCloudTarget>> target_tree_;
  std::vector<Eigen::Matrix4d> source_covs_;
  std::vector<Eigen::Matrix4d> target_covs_;

  Eigen::Array<double, 6, 1> dof_mask_;
  bool has_rotation_prior_;
  Eigen::Matrix3d rotation_prior_R_;
  Eigen::Vector3d rotation_prior_info_;

  bool converged_;
  Eigen::Matrix4f final_transformation_;
  double final_fitness_;
  double final_error_;
  small_gicp::RegistrationResult result_;
};

}  // namespace gicp_plusplus

#endif  // GICP_PLUSPLUS_SMALL_GICP_BACKEND_HPP
