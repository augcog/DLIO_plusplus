#pragma once

#include <any>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <gtsam_points/util/gtsam_migration.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/mapping/global_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class StreamTempBufferRoundRobin;

class GaussianVoxelMap;
class NearestNeighborSearch;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;

/**
 * @brief GlobalMappingPoseGraph parameters.
 */
struct GlobalMappingPoseGraphParams {
public:
  GlobalMappingPoseGraphParams();
  ~GlobalMappingPoseGraphParams();

public:
  bool enable_optimization;

  std::string registration_type;

  double min_travel_dist;
  double max_neighbor_dist;
  double min_inliear_fraction;

  int subsample_target;
  double subsample_rate;
  double gicp_max_correspondence_dist;
  double vgicp_voxel_resolution;

  double odom_factor_stddev;
  /**
   * Rotation (radians) and translation (metres) standard deviations for the
   * between-submap odometry factor.
   *
   * Both default to odom_factor_stddev for backwards compatibility. Keeping
   * them independent lets a GNSS/INS mapping profile give per-pose position
   * and gravity factors enough authority without also weakening yaw.
   */
  double odom_rotation_stddev;
  double odom_translation_stddev;
  double loop_factor_stddev;
  double loop_factor_robust_width;

  int loop_candidate_buffer_size;
  int loop_candidate_eval_per_thread;

  /**
   * Maximum deterministic proximity candidates proposed for each loop source.
   *
   * Zero preserves the legacy unlimited behavior. A value of one selects the
   * closest eligible historical pose and prevents repeated laps from adding a
   * growing number of equivalent loop factors for the same source.
   */
  int max_loop_candidates_per_source;

  /**
   * Reject a loop registration when its optimized relative pose moves farther
   * than these limits from the odometry/GNSS pose-graph initial guess.
   *
   * Non-positive values disable the corresponding gate. The inlier fraction
   * alone cannot distinguish a correct racetrack revisit from a nearby
   * repeated structure with the wrong longitudinal phase.
   */
  double loop_max_translation_correction;
  double loop_max_rotation_correction_deg;

  /**
   * Wait for every loop candidate proposed before an optimizer update to
   * finish registration before collecting loop factors.
   *
   * This prevents the GNSS health callback from observing a timing-dependent
   * partial loop-factor set. Zero preserves the legacy asynchronous behavior;
   * a positive value is the maximum wait in seconds.
   */
  double loop_detection_sync_timeout_sec;

  /**
   * Keep one loop-registration source/target for every N geometric submaps.
   *
   * This controls only loop-search density and its bounded point/KdTree
   * working set. Every submap remains an independent pose-graph node and keeps
   * its odometry, GNSS, gravity, dense-map export, and trajectory output.
   */
  int loop_registration_interval;

  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  double init_pose_damping_scale;

  /**
   * Number of inserted submaps accumulated before one global iSAM2 update.
   *
   * This deliberately does not change submap geometry. In particular, a
   * one-keyframe submap can preserve the perception-ws mapping quality while a
   * long offline run amortizes optimizer and callback overhead over a bounded
   * batch. optimize() and save() always flush a partial final batch.
   */
  int optimizer_update_interval;

  /**
   * Maximum no-new-factor iSAM2 updates after a batch containing loops.
   *
   * Loop factors can produce a large nonlinear delta in one update. Forced
   * relinearization continues until iSAM2 reports no relinearized variables or
   * this bound is reached, before extensions inspect the trajectory. Zero
   * preserves the legacy single-pass behavior.
   */
  int optimizer_extra_loop_updates;

  /**
   * Optional directory used to spill the dense point payload of each submap.
   *
   * Pose-graph loop detection only needs the bounded registration sample.  The
   * full-resolution cloud is restored when save() writes the final GLIM dump.
   * Keeping these two representations separate makes per-scan submaps practical
   * on development machines without reducing the exported map density.
   */
  std::string offload_points_dir;

  int num_threads;
};

/// @brief Submap target
struct SubMapTarget {
  using Ptr = std::shared_ptr<SubMapTarget>;
  using ConstPtr = std::shared_ptr<const SubMapTarget>;

  SubMap::ConstPtr submap;
  gtsam_points::PointCloud::ConstPtr registration_target;
  gtsam_points::PointCloud::ConstPtr subsampled;
  std::shared_ptr<gtsam_points::NearestNeighborSearch> tree;
  std::shared_ptr<gtsam_points::GaussianVoxelMap> voxels;
  double travel_dist;
};

/// @brief Loop candidate
struct LoopCandidate {
  SubMapTarget::ConstPtr target;
  SubMapTarget::ConstPtr source;
  Eigen::Isometry3d init_T_target_source;
};

/**
 * @brief Global mapping with the old conventional pose graph optimization.
 * @note  We recommend using GlobalMapping instead of this class if accuracy matters.
 */
class GlobalMappingPoseGraph : public GlobalMappingBase {
public:
  GlobalMappingPoseGraph(const GlobalMappingPoseGraphParams& params = GlobalMappingPoseGraphParams());
  virtual ~GlobalMappingPoseGraph();

  virtual void insert_submap(const SubMap::Ptr& submap) override;

  virtual void optimize() override;

  virtual void save(const std::string& path) override;
  virtual gtsam_points::PointCloud::Ptr export_points() override;
  virtual size_t num_submaps() const override { return submaps.size(); }

private:
  void insert_submap(int current, const SubMap::Ptr& submap);
  void update_optimizer();

  std::shared_ptr<gtsam::NonlinearFactorGraph> create_odometry_factors(int current) const;
  void find_loop_candidates(int current);
  std::shared_ptr<gtsam::NonlinearFactorGraph> collect_detected_loops();
  void synchronize_loop_detection();

  void update_submaps();

  /**
   * Stop accepting loop candidates, drain every candidate already queued, and
   * join the detector thread.  save() must call this before its final optimize
   * so accepted loops cannot arrive after the serialized graph snapshot.
   */
  void finish_loop_detection();
  void loop_detection_task();
  void restore_offloaded_points(size_t index, const std::string& output_submap_dir);

private:
  using Params = GlobalMappingPoseGraphParams;
  Params params;

  // Default-seeded engine keeps registration subsampling reproducible.
  std::mt19937 mt;

  std::atomic_bool loop_detection_finalized;
  std::thread loop_detection_thread;
  ConcurrentVector<LoopCandidate> loop_candidates;
  ConcurrentVector<gtsam_points::shared_ptr<gtsam::NonlinearFactor>> detected_loops;
  std::atomic_uint64_t loop_candidates_proposed;
  std::atomic_uint64_t loop_candidates_evaluated;
  std::atomic_uint64_t loop_candidates_completed;
  std::atomic_uint64_t loop_candidates_dropped;
  std::atomic_uint64_t loop_candidates_sanity_rejected;
  std::atomic_uint64_t loop_detection_sync_timeouts;
  std::atomic_uint64_t loop_factors_accepted;
  std::atomic<double> loop_accepted_max_translation_correction;
  std::atomic<double> loop_accepted_max_rotation_correction_deg;
  std::atomic_uint64_t optimizer_extra_loop_updates_executed;
  std::atomic_uint64_t optimizer_loop_batches_refined;
  std::atomic_uint64_t optimizer_loop_refinement_converged;
  std::atomic_uint64_t optimizer_loop_refinement_cap_exhausted;
  std::mutex loop_detection_state_mutex;
  std::condition_variable loop_detection_state_cv;

  std::vector<SubMap::Ptr> submaps;
  std::vector<SubMapTarget::Ptr> submap_targets;
  std::vector<std::string> offloaded_point_dirs;

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_points::ISAM2Ext> isam2;

  std::shared_ptr<void> tbb_task_arena;
};
}  // namespace glim
