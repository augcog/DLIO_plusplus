#include <glim/mapping/global_mapping_pose_graph.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/util/parallelism.hpp>

#include <glim/util/config.hpp>
#include <glim/util/serialization.hpp>
#include <glim/mapping/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

GlobalMappingPoseGraphParams::GlobalMappingPoseGraphParams() {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));

  enable_optimization = config.param<bool>("global_mapping", "enable_optimization", true);
  registration_type = config.param<std::string>("global_mapping", "registration_type", "GICP");

  min_travel_dist = config.param<double>("global_mapping", "min_travel_dist", 100.0);
  max_neighbor_dist = config.param<double>("global_mapping", "max_neighbor_dist", 10.0);
  min_inliear_fraction = config.param<double>("global_mapping", "min_inliear_fraction", 0.5);

  subsample_target = config.param<int>("global_mapping", "subsample_target", 10000);
  subsample_rate = config.param<double>("global_mapping", "subsample_rate", 0.1);
  gicp_max_correspondence_dist = config.param<double>("global_mapping", "gicp_max_correspondence_dist", 2.0);
  vgicp_voxel_resolution = config.param<double>("global_mapping", "vgicp_voxel_resolution", 2.0);

  odom_factor_stddev = config.param<double>("global_mapping", "odom_factor_stddev", 1e-3);
  odom_rotation_stddev =
    config.param<double>("global_mapping", "odom_rotation_stddev", odom_factor_stddev);
  odom_translation_stddev =
    config.param<double>("global_mapping", "odom_translation_stddev", odom_factor_stddev);
  if (!std::isfinite(odom_rotation_stddev) || odom_rotation_stddev <= 0.0) {
    throw std::invalid_argument("global_mapping.odom_rotation_stddev must be finite and positive");
  }
  if (!std::isfinite(odom_translation_stddev) || odom_translation_stddev <= 0.0) {
    throw std::invalid_argument("global_mapping.odom_translation_stddev must be finite and positive");
  }
  loop_factor_stddev = config.param<double>("global_mapping", "loop_factor_stddev", 0.1);
  loop_factor_robust_width = config.param<double>("global_mapping", "loop_factor_robust_width", 1.0);

  loop_candidate_buffer_size = config.param<int>("global_mapping", "loop_candidate_buffer_size", 100);
  loop_candidate_eval_per_thread = config.param<int>("global_mapping", "loop_candidate_eval_per_thread", 2);
  max_loop_candidates_per_source = std::max(0, config.param<int>("global_mapping", "max_loop_candidates_per_source", 0));
  loop_max_translation_correction = std::max(0.0, config.param<double>("global_mapping", "loop_max_translation_correction", 0.0));
  loop_max_rotation_correction_deg = std::max(0.0, config.param<double>("global_mapping", "loop_max_rotation_correction_deg", 0.0));
  loop_detection_sync_timeout_sec = std::max(0.0, config.param<double>("global_mapping", "loop_detection_sync_timeout_sec", 0.0));
  loop_registration_interval = std::max(1, config.param<int>("global_mapping", "loop_registration_interval", 1));

  use_isam2_dogleg = config.param<bool>("global_mapping", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("global_mapping", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1);

  init_pose_damping_scale = config.param<double>("global_mapping", "init_pose_damping_scale", 1e10);
  optimizer_update_interval = std::max(1, config.param<int>("global_mapping", "optimizer_update_interval", 1));
  optimizer_extra_loop_updates = std::max(0, config.param<int>("global_mapping", "optimizer_extra_loop_updates", 0));
  offload_points_dir = config.param<std::string>("global_mapping", "offload_points_dir", "");

  num_threads = config.param<int>("global_mapping", "num_threads", 2);
}

GlobalMappingPoseGraphParams::~GlobalMappingPoseGraphParams() {}

GlobalMappingPoseGraph::GlobalMappingPoseGraph(const GlobalMappingPoseGraphParams& params) : params(params) {
  if (!params.offload_points_dir.empty()) {
    const boost::filesystem::path offload_dir(params.offload_points_dir);
    if (!offload_dir.is_absolute()) {
      throw std::invalid_argument("global_mapping.offload_points_dir must be an absolute path");
    }
    if (boost::filesystem::exists(offload_dir) && !boost::filesystem::is_empty(offload_dir)) {
      throw std::runtime_error(
        "global_mapping.offload_points_dir is not empty: " + offload_dir.string() + " (use a unique per-run directory so stale point payloads cannot enter a map)");
    }
    boost::filesystem::create_directories(offload_dir);
    logger->info("dense submap point offload enabled: {}", offload_dir.string());
  }

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }

#ifdef GTSAM_USE_TBB
  tbb_task_arena.reset(new tbb::task_arena(params.num_threads));
#endif

  loop_detection_finalized = false;
  loop_candidates_proposed = 0;
  loop_candidates_evaluated = 0;
  loop_candidates_completed = 0;
  loop_candidates_dropped = 0;
  loop_candidates_sanity_rejected = 0;
  loop_detection_sync_timeouts = 0;
  loop_factors_accepted = 0;
  loop_accepted_max_translation_correction = 0.0;
  loop_accepted_max_rotation_correction_deg = 0.0;
  optimizer_extra_loop_updates_executed = 0;
  optimizer_loop_batches_refined = 0;
  optimizer_loop_refinement_converged = 0;
  optimizer_loop_refinement_cap_exhausted = 0;
  loop_detection_thread = std::thread([this] { loop_detection_task(); });
}

GlobalMappingPoseGraph::~GlobalMappingPoseGraph() {
  finish_loop_detection();
}

void GlobalMappingPoseGraph::finish_loop_detection() {
  if (loop_detection_finalized.exchange(true)) {
    return;
  }

  loop_candidates.submit_end_of_data();
  if (loop_detection_thread.joinable()) {
    loop_detection_thread.join();
  }

  logger->info(
    "loop closure summary: proposed={} evaluated={} accepted={} dropped={} sanity_rejected={}",
    loop_candidates_proposed.load(),
    loop_candidates_evaluated.load(),
    loop_factors_accepted.load(),
    loop_candidates_dropped.load(),
    loop_candidates_sanity_rejected.load());
}

void GlobalMappingPoseGraph::insert_submap(const SubMap::Ptr& submap) {
  const int current = submaps.size();
  const int last = current - 1;
  insert_submap(current, submap);

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  if (current == 0) {
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  } else {
    new_factors->add(*create_odometry_factors(current));

    find_loop_candidates(current);
  }

  if ((current + 1) % params.optimizer_update_interval == 0) {
    update_optimizer();
  }
}

void GlobalMappingPoseGraph::update_optimizer() {
  synchronize_loop_detection();
  const auto loop_factors = collect_detected_loops();
  const bool has_loop_factors = !loop_factors->empty();
  new_factors->add(*loop_factors);

  // Extension modules can enqueue factors asynchronously after the final
  // insertion batch.  Give them a chance to flush before deciding that there
  // is no optimizer work at an explicit optimize/save boundary.
  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  if (new_values->empty() && new_factors->empty()) {
    return;
  }

  gtsam_points::ISAM2ResultExt result;
  try {
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      result = isam2->update(*new_factors, *new_values);
#ifdef GTSAM_USE_TBB
    });
#endif

    Callbacks::on_smoother_update_result(*isam2, result);

    // A loop-closure batch can move a long chain far enough that one
    // incremental update leaves a large nonlinear delta. Run bounded
    // no-new-factor refinement before publishing updated submaps to the GNSS
    // health callback. A fixed one-pass experiment still produced a transient
    // 0.90 m anchor residual at Laguna Run1 frame 19.6k, while the next update
    // settled the same graph to 0.20 m. Stop on iSAM2's direct convergence
    // evidence instead of assuming a fixed number of passes is sufficient.
    if (has_loop_factors && params.optimizer_extra_loop_updates > 0) {
      optimizer_loop_batches_refined.fetch_add(1);
      bool refinement_converged = false;
      size_t remaining_relinearized = result.variablesRelinearized;
      for (int i = 0; i < params.optimizer_extra_loop_updates; ++i) {
        gtsam_points::ISAM2ResultExt refinement_result;
        gtsam::ISAM2UpdateParams refinement_params;
        refinement_params.force_relinearize = true;
#ifdef GTSAM_USE_TBB
        arena->execute([&] {
#endif
          refinement_result =
            isam2->update(gtsam::NonlinearFactorGraph(), gtsam::Values(), refinement_params);
#ifdef GTSAM_USE_TBB
        });
#endif
        optimizer_extra_loop_updates_executed.fetch_add(1);
        Callbacks::on_smoother_update_result(*isam2, refinement_result);
        remaining_relinearized = refinement_result.variablesRelinearized;
        if (remaining_relinearized == 0) {
          refinement_converged = true;
          optimizer_loop_refinement_converged.fetch_add(1);
          break;
        }
      }
      if (!refinement_converged) {
        optimizer_loop_refinement_cap_exhausted.fetch_add(1);
        logger->warn(
          "loop refinement reached its {}-update bound with {} variables still "
          "relinearized; publishing the bounded result to map-health callbacks",
          params.optimizer_extra_loop_updates,
          remaining_relinearized);
      }
    }

  } catch (const std::exception& e) {
    logger->error("an exception was caught during global map optimization!!");
    logger->error(e.what());
    // Extensions may have appended factors in on_smoother_update(). Tell them
    // explicitly that this transaction did not complete; otherwise handoff
    // accounting can claim factors that were discarded with new_factors below.
    Callbacks::on_smoother_update_failure(*isam2, e.what());
  }
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMappingPoseGraph::optimize() {
  // Flush a partial optimizer batch at explicit optimize/save boundaries.
  // update_optimizer() also drains loop factors accepted since the last
  // insertion batch.
  update_optimizer();
}

void GlobalMappingPoseGraph::save(const std::string& path) {
  // Loop detection is asynchronous.  Taking the final graph snapshot before
  // draining it silently loses every accepted factor still in the detector's
  // local/worker queues.  This is especially damaging for per-scan submaps,
  // where a lap boundary can enqueue many candidates near end-of-bag.
  finish_loop_detection();
  optimize();

  boost::filesystem::create_directories(path);

  gtsam::NonlinearFactorGraph serializable_factors = isam2->getFactorsUnsafe();

  logger->info("serializing factor graph to {}/graph.bin", path);
  serializeToBinaryFile(serializable_factors, path + "/graph.bin");
  serializeToBinaryFile(isam2->calculateEstimate(), path + "/values.bin");

  std::ofstream ofs(path + "/graph.txt");
  ofs << "num_submaps: " << submaps.size() << std::endl;
  ofs << "num_all_frames: " << std::accumulate(submaps.begin(), submaps.end(), 0, [](int sum, const SubMap::ConstPtr& submap) { return sum + submap->frames.size(); }) << std::endl;

  ofs << "num_matching_cost_factors: " << 0 << std::endl;
  ofs << "num_loop_candidates_proposed: " << loop_candidates_proposed.load() << std::endl;
  ofs << "num_loop_candidates_evaluated: " << loop_candidates_evaluated.load() << std::endl;
  ofs << "num_loop_candidates_completed: " << loop_candidates_completed.load() << std::endl;
  ofs << "num_loop_candidates_dropped: " << loop_candidates_dropped.load() << std::endl;
  ofs << "num_loop_candidates_sanity_rejected: " << loop_candidates_sanity_rejected.load() << std::endl;
  ofs << "num_loop_detection_sync_timeouts: " << loop_detection_sync_timeouts.load() << std::endl;
  ofs << "num_loop_closure_factors: " << loop_factors_accepted.load() << std::endl;
  ofs << "loop_accepted_max_translation_correction: "
      << loop_accepted_max_translation_correction.load() << std::endl;
  ofs << "loop_accepted_max_rotation_correction_deg: "
      << loop_accepted_max_rotation_correction_deg.load() << std::endl;
  ofs << "odom_rotation_stddev: " << params.odom_rotation_stddev << std::endl;
  ofs << "odom_translation_stddev: " << params.odom_translation_stddev << std::endl;
  ofs << "optimizer_extra_loop_updates: " << params.optimizer_extra_loop_updates << std::endl;
  ofs << "optimizer_extra_loop_updates_executed: " << optimizer_extra_loop_updates_executed.load() << std::endl;
  ofs << "optimizer_loop_batches_refined: " << optimizer_loop_batches_refined.load() << std::endl;
  ofs << "optimizer_loop_refinement_converged: " << optimizer_loop_refinement_converged.load() << std::endl;
  ofs << "optimizer_loop_refinement_cap_exhausted: " << optimizer_loop_refinement_cap_exhausted.load() << std::endl;
  ofs << "max_loop_candidates_per_source: " << params.max_loop_candidates_per_source << std::endl;
  ofs << "loop_max_translation_correction: " << params.loop_max_translation_correction << std::endl;
  ofs << "loop_max_rotation_correction_deg: " << params.loop_max_rotation_correction_deg << std::endl;
  ofs << "loop_detection_sync_timeout_sec: " << params.loop_detection_sync_timeout_sec << std::endl;
  ofs << "loop_registration_interval: " << params.loop_registration_interval << std::endl;
  ofs << "num_loop_registration_submaps: "
      << (submaps.size() + static_cast<size_t>(params.loop_registration_interval) - 1) /
           static_cast<size_t>(params.loop_registration_interval)
      << std::endl;

  std::ofstream odom_lidar_ofs(path + "/odom_lidar.txt");
  std::ofstream traj_lidar_ofs(path + "/traj_lidar.txt");

  std::ofstream odom_imu_ofs(path + "/odom_imu.txt");
  std::ofstream traj_imu_ofs(path + "/traj_imu.txt");

  const auto write_tum_frame = [](std::ofstream& ofs, const double stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d trans(pose.translation());
    ofs << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % stamp % trans.x() % trans.y() % trans.z() % quat.x() % quat.y() % quat.z() % quat.w() << std::endl;
  };

  bool dense_restore_failed = false;
  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->odom_frames) {
      write_tum_frame(odom_lidar_ofs, frame->stamp, frame->T_world_lidar);
      write_tum_frame(odom_imu_ofs, frame->stamp, frame->T_world_imu);
    }

    const Eigen::Isometry3d T_world_endpoint_L = submaps[i]->T_world_origin * submaps[i]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_lidar0 = submaps[i]->frames.front()->T_world_lidar;
    const Eigen::Isometry3d T_odom_imu0 = submaps[i]->frames.front()->T_world_imu;

    for (const auto& frame : submaps[i]->frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();

      write_tum_frame(traj_imu_ofs, frame->stamp, T_world_imu);
      write_tum_frame(traj_lidar_ofs, frame->stamp, T_world_lidar);
    }

    const std::string output_submap_dir = (boost::format("%s/%06d") % path % i).str();
    submaps[i]->save(output_submap_dir);
    try {
      restore_offloaded_points(i, output_submap_dir);
    } catch (const std::exception& e) {
      dense_restore_failed = true;
      logger->error(
          "failed to restore dense points for submap {}: {}. "
          "Continuing so the partial dump remains inspectable.",
          i, e.what());
    }
  }
  if (dense_restore_failed) {
    throw std::runtime_error(
        "one or more dense submap payloads failed to restore; partial dump retained");
  }
}

gtsam_points::PointCloud::Ptr GlobalMappingPoseGraph::export_points() {
  return std::make_shared<gtsam_points::PointCloudCPU>();
}

void GlobalMappingPoseGraph::insert_submap(int current, const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={}", submap->id);

  submap->voxelmaps.clear();

  submaps.push_back(submap);

  auto target = std::make_shared<SubMapTarget>();
  target->submap = submap;
  const bool is_loop_registration_submap = current % params.loop_registration_interval == 0;

  // Dense per-scan submaps are the quality lever. Persist every dense payload
  // first, independent of whether this node participates in loop registration.
  if (!params.offload_points_dir.empty()) {
    const std::string offload_submap_dir = (boost::format("%s/%06d") % params.offload_points_dir % current).str();
    boost::filesystem::create_directories(offload_submap_dir);
    submap->frame->save_compact(offload_submap_dir);
    offloaded_point_dirs.push_back(offload_submap_dir);
  } else {
    offloaded_point_dirs.emplace_back();
  }

  if (is_loop_registration_submap) {
    // Subsample points only for loop-registration nodes. Separating this
    // working set from geometric submap cadence keeps per-scan mapping quality
    // without retaining one KdTree and sample for every scan of a long run.
    if (params.subsample_target > 0) {
      const double sampling_rate = std::min(1.0, static_cast<double>(params.subsample_target) / submap->frame->size());
      target->subsampled = sampling_rate < 1.0 ? gtsam_points::random_sampling(submap->frame, sampling_rate, mt) : submap->frame;
    } else {
      if (params.subsample_rate > 0.99) {
        target->subsampled = submap->frame;
      } else {
        target->subsampled = gtsam_points::random_sampling(submap->frame, params.subsample_rate, mt);
      }
    }

    target->registration_target = params.offload_points_dir.empty() ? submap->frame : target->subsampled;

    // Create nearest neighbor search only for retained loop-registration nodes.
    if (params.registration_type == "GICP") {
      target->tree = std::make_shared<gtsam_points::KdTree>(target->registration_target->points, target->registration_target->size());
    } else if (params.registration_type == "VGICP") {
      target->voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(params.vgicp_voxel_resolution);
      target->voxels->insert(*target->registration_target);
    } else {
      logger->warn("unknown registration type: {}", params.registration_type);
    }
  }

  if (!params.offload_points_dir.empty()) {
    // save() first serializes the lightweight in-memory representation, then
    // restore_offloaded_points() overwrites it with the original dense compact
    // payload. Non-registration nodes therefore retain no point sample at all.
    submap->frame = is_loop_registration_submap
                      ? std::const_pointer_cast<gtsam_points::PointCloud>(target->registration_target)
                      : std::make_shared<gtsam_points::PointCloudCPU>();
  }

  if (current == 0) {
    target->travel_dist = 0.0;
  } else {
    const double displacement = (submaps[current - 1]->T_world_origin.translation() - submaps[current]->T_world_origin.translation()).norm();
    target->travel_dist = submap_targets.back()->travel_dist + displacement;
  }

  submap_targets.push_back(target);
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingPoseGraph::create_odometry_factors(int current) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const int last = current - 1;
  const gtsam::Pose3 T_last_current = gtsam::Pose3((submaps[last]->origin_frame()->T_world_sensor().inverse() * submaps[current]->origin_frame()->T_world_sensor()).matrix());
  gtsam::Vector6 odom_sigmas;
  odom_sigmas <<
    params.odom_rotation_stddev,
    params.odom_rotation_stddev,
    params.odom_rotation_stddev,
    params.odom_translation_stddev,
    params.odom_translation_stddev,
    params.odom_translation_stddev;
  factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    X(last),
    X(current),
    T_last_current,
    gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas));

  return factors;
}

void GlobalMappingPoseGraph::find_loop_candidates(int current) {
  if (current % params.loop_registration_interval != 0) {
    return;
  }

  std::vector<LoopCandidate> new_candidates;
  for (int i = 0; i < submaps.size() - 1; i++) {
    if (i % params.loop_registration_interval != 0) {
      continue;
    }

    // Skip if the direct distance between submaps is too far.
    const double direct_dist = (submaps[current]->T_world_origin.translation() - submaps[i]->T_world_origin.translation()).norm();
    if (direct_dist > params.max_neighbor_dist) {
      // Fast forward if the direct distance is too far.
      if (i != 0 && direct_dist > params.max_neighbor_dist * 2) {
        const int average_window = 3;
        const int left = std::max(0, i - average_window);
        const double travel_dist_avg = (submap_targets[i]->travel_dist - submap_targets[left]->travel_dist) / std::max(i - left, 1);
        const int step = 0.8 * direct_dist / std::min(travel_dist_avg, 100.0);

        i += std::min(10, step);
      }

      continue;
    }

    // Break if the travel distance is too short.
    const double travel_dist = submap_targets[current]->travel_dist - submap_targets[i]->travel_dist;
    if (travel_dist < params.min_travel_dist) {
      break;
    }

    // Add a loop candidate.
    const Eigen::Isometry3d T_target_source = submaps[i]->T_world_origin.inverse() * submaps[current]->T_world_origin;
    new_candidates.emplace_back(LoopCandidate{submap_targets[i], submap_targets[current], T_target_source});
  }

  if (params.max_loop_candidates_per_source > 0 &&
      new_candidates.size() > static_cast<size_t>(params.max_loop_candidates_per_source)) {
    // The legacy random buffer selected a timing-dependent subset and allowed
    // equivalent constraints to multiply with every repeated lap. Keep the
    // closest pose-prior candidates deterministically; GICP still validates
    // their actual point overlap before any factor is accepted.
    std::stable_sort(
      new_candidates.begin(),
      new_candidates.end(),
      [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
        return lhs.init_T_target_source.translation().squaredNorm() <
               rhs.init_T_target_source.translation().squaredNorm();
      });
    new_candidates.resize(static_cast<size_t>(params.max_loop_candidates_per_source));
  }

  loop_candidates_proposed.fetch_add(new_candidates.size());
  loop_candidates.insert(new_candidates);
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingPoseGraph::collect_detected_loops() {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();

  factors->add(detected_loops.get_all_and_clear());

  return factors;
}

void GlobalMappingPoseGraph::synchronize_loop_detection() {
  if (params.loop_detection_sync_timeout_sec <= 0.0) {
    return;
  }

  // Only wait for work proposed before this optimizer boundary. insert_submap()
  // is single-threaded, so the target cannot advance while this method waits.
  // Completion is published only after the detector has inserted every
  // accepted factor (and accounted for every rejection) into detected_loops.
  const uint64_t target = loop_candidates_proposed.load();
  if (loop_candidates_completed.load() >= target) {
    return;
  }

  std::unique_lock<std::mutex> lock(loop_detection_state_mutex);
  const auto timeout = std::chrono::duration<double>(params.loop_detection_sync_timeout_sec);
  const bool complete = loop_detection_state_cv.wait_for(
    lock,
    timeout,
    [this, target] {
      return loop_candidates_completed.load() >= target;
    });
  if (!complete) {
    loop_detection_sync_timeouts.fetch_add(1);
    logger->warn(
      "loop detector synchronization timed out after {:.3f}s: proposed={} completed={}; "
      "continuing with the completed factor set",
      params.loop_detection_sync_timeout_sec,
      target,
      loop_candidates_completed.load());
  }
}

void GlobalMappingPoseGraph::loop_detection_task() {
  // Keep candidate selection reproducible and independent from registration
  // point subsampling, which uses the class-level engine.
  std::mt19937 loop_mt;
  std::deque<LoopCandidate> candidates_buffer;  // Local loop candidate buffer

  while (true) {
    logger->debug("wait for loop candidates");
    auto new_candidates = loop_candidates.get_all_and_clear_wait();
    candidates_buffer.insert(candidates_buffer.end(), new_candidates.begin(), new_candidates.end());

    logger->debug("|candidates_buffer|={}", candidates_buffer.size());

    // An empty batch is only returned after submit_end_of_data().  Keep
    // iterating while the local buffer is non-empty so EOF drains it instead
    // of abandoning candidates during save/destruction.
    if (new_candidates.empty() && candidates_buffer.empty()) {
      break;
    }

    const size_t buffer_limit = static_cast<size_t>(std::max(1, params.loop_candidate_buffer_size));
    if (candidates_buffer.size() > buffer_limit) {
      // Uniform sampling preserves candidate diversity when legacy unlimited
      // per-source proposals are enabled.  The deterministic per-source cap
      // above normally prevents this overflow in the high-quality profile;
      // this fallback must not introduce a newest/oldest ordering bias.
      std::shuffle(candidates_buffer.begin(), candidates_buffer.end(), loop_mt);
      const size_t num_dropped = candidates_buffer.size() - buffer_limit;
      loop_candidates_dropped.fetch_add(num_dropped);
      // Dropped proposals are terminal too. Count them as completed so a
      // synchronization boundary never waits for work intentionally removed
      // by the bounded detector buffer.
      loop_candidates_completed.fetch_add(num_dropped);
      loop_detection_state_cv.notify_all();
      candidates_buffer.resize(buffer_limit);
    }

    // Take a subset of the candidates to evaluate.
    const int eval_count = std::max(1, params.loop_candidate_eval_per_thread * params.num_threads);
    std::vector<LoopCandidate> candidates;
    if (candidates_buffer.size() < eval_count) {
      candidates.assign(candidates_buffer.begin(), candidates_buffer.end());
      candidates_buffer.clear();
    } else {
      candidates.assign(candidates_buffer.begin(), candidates_buffer.begin() + eval_count);
      candidates_buffer.erase(candidates_buffer.begin(), candidates_buffer.begin() + eval_count);
    }
    loop_candidates_evaluated.fetch_add(candidates.size());

    std::vector<double> inlier_fractions(candidates.size(), 0.0);
    std::vector<gtsam::Pose3> T_target_source(candidates.size());

    const auto evaluate_candidate = [&](int i) {
      const auto candidate = candidates[i];
      const auto target = candidates[i].target;
      const auto source = candidates[i].source;

      gtsam::Values values;
      values.insert(0, gtsam::Pose3(candidates[i].init_T_target_source.matrix()));

      double error, inlier_fraction;

      if (params.registration_type == "GICP") {
        auto factor =
          gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(gtsam::Pose3(), 0, candidate.target->registration_target, candidate.source->subsampled, candidate.target->tree);
        factor->set_max_correspondence_distance(params.gicp_max_correspondence_dist);

        gtsam::NonlinearFactorGraph graph;
        graph.add(factor);

        gtsam_points::LevenbergMarquardtExtParams lm_params;
        lm_params.setMaxIterations(10);
        values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

        error = factor->error(values);
        inlier_fraction = factor->inlier_fraction();
      } else if (params.registration_type == "VGICP") {
        auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, candidate.target->voxels, candidate.source->subsampled);

        gtsam::NonlinearFactorGraph graph;
        graph.add(factor);

        gtsam_points::LevenbergMarquardtExtParams lm_params;
        lm_params.setMaxIterations(10);

        values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

        error = factor->error(values);
        inlier_fraction = factor->inlier_fraction();
      } else {
        logger->warn("unknown registration type: {}", params.registration_type);
        return;
      }

      logger->debug("target={}, source={}, error={}, inlier_fraction={}", target->submap->id, source->submap->id, error, inlier_fraction);

      inlier_fractions[i] = inlier_fraction;
      T_target_source[i] = values.at<gtsam::Pose3>(0);
    };

    // Evaluate loop candidates in parallel.
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      if (gtsam_points::is_omp_default()) {
#pragma omp parallel for num_threads(params.num_threads) schedule(dynamic)
        for (int i = 0; i < candidates.size(); i++) {
          evaluate_candidate(i);
        }
      } else {
#ifdef GTSAM_POINTS_USE_TBB
        tbb::parallel_for(tbb::blocked_range<int>(0, candidates.size(), 2), [&](const tbb::blocked_range<int>& range) {
          for (int i = range.begin(); i < range.end(); i++) {
            evaluate_candidate(i);
          }
        });
#else
      std::cerr << "error : TBB is not enabled" << std::endl;
      abort();
#endif
      }

#ifdef GTSAM_USE_TBB
    });
#endif

    // Check the matching results.
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
    for (int i = 0; i < candidates.size(); i++) {
      // Check if the inlier fraction (overlap with target) is large enough.
      if (inlier_fractions[i] < params.min_inliear_fraction) {
        continue;
      }

      const Eigen::Isometry3d optimized_T_target_source(T_target_source[i].matrix());
      const Eigen::Isometry3d correction =
        candidates[i].init_T_target_source.inverse() * optimized_T_target_source;
      const double translation_correction = correction.translation().norm();
      const double rotation_correction_deg =
        Eigen::AngleAxisd(correction.linear()).angle() * 180.0 / M_PI;
      const bool translation_sanity_failed =
        params.loop_max_translation_correction > 0.0 &&
        translation_correction > params.loop_max_translation_correction;
      const bool rotation_sanity_failed =
        params.loop_max_rotation_correction_deg > 0.0 &&
        rotation_correction_deg > params.loop_max_rotation_correction_deg;
      if (translation_sanity_failed || rotation_sanity_failed) {
        loop_candidates_sanity_rejected.fetch_add(1);
        logger->warn(
          "loop candidate sanity rejected: target={} source={} correction={:.3f}m/{:.3f}deg "
          "limits={:.3f}m/{:.3f}deg inlier_fraction={:.3f}",
          candidates[i].target->submap->id,
          candidates[i].source->submap->id,
          translation_correction,
          rotation_correction_deg,
          params.loop_max_translation_correction,
          params.loop_max_rotation_correction_deg,
          inlier_fractions[i]);
        continue;
      }

      const auto update_max = [](std::atomic<double>& maximum, double value) {
        double observed = maximum.load();
        while (value > observed &&
               !maximum.compare_exchange_weak(observed, value)) {
        }
      };
      update_max(
        loop_accepted_max_translation_correction,
        translation_correction);
      update_max(
        loop_accepted_max_rotation_correction_deg,
        rotation_correction_deg);

      // Create factor.
      gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Isotropic::Sigma(6, params.loop_factor_stddev);
      noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(params.loop_factor_robust_width), noise_model);
      factors.emplace_back(
        gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(candidates[i].target->submap->id), X(candidates[i].source->submap->id), T_target_source[i], noise_model));
    }

    loop_factors_accepted.fetch_add(factors.size());
    detected_loops.insert(factors);
    loop_candidates_completed.fetch_add(candidates.size());
    loop_detection_state_cv.notify_all();
  }
}

void GlobalMappingPoseGraph::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

void GlobalMappingPoseGraph::restore_offloaded_points(size_t index, const std::string& output_submap_dir) {
  if (index >= offloaded_point_dirs.size() || offloaded_point_dirs[index].empty()) {
    return;
  }

  const boost::filesystem::path source_dir(offloaded_point_dirs[index]);
  const boost::filesystem::path destination_dir(output_submap_dir);
  if (boost::filesystem::absolute(source_dir) == boost::filesystem::absolute(destination_dir)) {
    return;
  }

  size_t restored_files = 0;
  for (boost::filesystem::directory_iterator it(source_dir), end; it != end; ++it) {
    if (!boost::filesystem::is_regular_file(it->path())) {
      continue;
    }
    const std::string filename = it->path().filename().string();
    if (filename.size() < 12 || filename.compare(filename.size() - 12, 12, "_compact.bin") != 0) {
      continue;
    }
    boost::filesystem::copy_file(it->path(), destination_dir / it->path().filename(), boost::filesystem::copy_options::overwrite_existing);
    ++restored_files;
  }

  if (restored_files == 0) {
    throw std::runtime_error("dense point offload payload is missing for submap " + std::to_string(index) + ": " + source_dir.string());
  }

  const boost::filesystem::path points_path =
      destination_dir / "points_compact.bin";
  if (!boost::filesystem::is_regular_file(points_path)) {
    throw std::runtime_error(
        "restored dense payload has no points_compact.bin for submap " +
        std::to_string(index));
  }
  const auto points_bytes = boost::filesystem::file_size(points_path);
  if (points_bytes == 0 || points_bytes % (3 * sizeof(float)) != 0) {
    throw std::runtime_error(
        "restored points_compact.bin has invalid byte size for submap " +
        std::to_string(index) + ": " + std::to_string(points_bytes));
  }
  const auto dense_point_count = points_bytes / (3 * sizeof(float));
  const boost::filesystem::path metadata_path = destination_dir / "data.txt";
  std::ofstream metadata(metadata_path.string(), std::ios::app);
  if (!metadata) {
    throw std::runtime_error(
        "cannot append dense-payload authority marker to " +
        metadata_path.string());
  }
  metadata << "points_compact_authoritative: true\n";
  metadata << "points_compact_count: " << dense_point_count << "\n";
  metadata.flush();
  if (!metadata) {
    throw std::runtime_error(
        "failed writing dense-payload authority marker to " +
        metadata_path.string());
  }

  logger->debug(
      "restored {} dense compact point files ({} authoritative points) for "
      "submap {} from {}",
      restored_files, dense_point_count, index, source_dir.string());
  offloaded_point_dirs[index] = destination_dir.string();

  // The dense payload is now durable in the dump. Reclaim only the temporary
  // per-run offload copy; never remove an earlier dump used as the source of a
  // subsequent save.
  if (source_dir.parent_path() == boost::filesystem::path(params.offload_points_dir)) {
    boost::filesystem::remove_all(source_dir);
  }
}

}  // namespace glim
