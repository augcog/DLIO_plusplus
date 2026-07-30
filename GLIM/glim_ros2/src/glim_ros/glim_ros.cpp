#include <stdexcept>
#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <thread>
#include <iostream>
#include <functional>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>

#include <glim/util/debug.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/ros_qos.hpp>
#include <glim/util/urdf_transforms.hpp>

namespace glim {

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  // Setup logger
  auto logger = spdlog::stdout_color_mt("glim");
  logger->sinks().push_back(get_ringbuffer_sink());
  spdlog::set_default_logger(logger);

  bool debug = false;
  this->declare_parameter<bool>("debug", false);
  this->get_parameter<bool>("debug", debug);

  if (debug) {
    spdlog::info("enable debug printing");
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/glim_log.log", true);
    logger->sinks().push_back(file_sink);
    logger->set_level(spdlog::level::trace);

    print_system_info(logger);
  }

  dump_on_unload = false;
  this->declare_parameter<bool>("dump_on_unload", false);
  this->get_parameter<bool>("dump_on_unload", dump_on_unload);

  if (dump_on_unload) {
    spdlog::info("dump_on_unload={}", dump_on_unload);
  }

  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  logger->info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  keep_raw_points = config_ros.param<bool>("glim_ros", "keep_raw_points", false);
  imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  points_time_offset = config_ros.param<double>("glim_ros", "points_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 0.0);

  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  intensity_field = config_sensors.param<std::string>("sensors", "intensity_field", "intensity");
  ring_field = config_sensors.param<std::string>("sensors", "ring_field", "");
  imu_input_rotation = config_sensors.param<Eigen::Quaterniond>("sensors", "imu_input_rotation", Eigen::Quaterniond::Identity());
  if (!imu_input_rotation.coeffs().allFinite() || imu_input_rotation.norm() < 1e-9) {
    throw std::invalid_argument("sensors.imu_input_rotation must be a finite, non-zero quaternion [x,y,z,w]");
  }
  imu_input_rotation.normalize();
  if (std::abs(imu_input_rotation.w() - 1.0) > 1e-12 || imu_input_rotation.vec().norm() > 1e-12) {
    logger->info(
      "IMU input calibration enabled: q_input_to_calibrated=[{:.9f}, {:.9f}, {:.9f}, {:.9f}]",
      imu_input_rotation.x(),
      imu_input_rotation.y(),
      imu_input_rotation.z(),
      imu_input_rotation.w());
  }
  // [P2 FIX 2026-07-15] Explicit Luminar-contract opt-in: when true, a FLOAT64
  // per-point time field is decoded as raw uint64 PTP epoch nanoseconds (the
  // documented Luminar driver variant) instead of IEEE-754 seconds. Default
  // false so ordinary FLOAT64-seconds sensors (relative offsets, or Hesai
  // absolute epoch seconds) are never misdecoded.
  float64_time_is_epoch_ns = config_sensors.param<bool>("sensors", "float64_time_is_epoch_ns", false);
  flip_points_y = config_sensors.param<bool>("sensors", "flip_points_y", false);

  // Multi-LiDAR concatenation. Live glim_ros must merge the auxiliary (left /
  // right) Luminar clouds into the primary luminar_front frame just like the
  // offline glim_rosbag / glim_pcap_rosbag tools do; otherwise live mapping
  // would silently use only the front LiDAR.
  aux_concat = glim_ros::load_aux_sensors_from_config(config_sensors);

  // Override T_lidar_imu from URDF if configured
  const std::string urdf_path = config_sensors.param<std::string>("sensors", "urdf_path", "");
  const std::string urdf_lidar_frame = config_sensors.param<std::string>("sensors", "urdf_lidar_frame", "");
  const std::string urdf_imu_frame = config_sensors.param<std::string>("sensors", "urdf_imu_frame", "");
  if (!urdf_path.empty() && !urdf_lidar_frame.empty() && !urdf_imu_frame.empty()) {
    try {
      auto urdf_transforms = glim::parse_urdf_transforms(urdf_path);
      Eigen::Isometry3d T_lidar_imu = glim::compute_transform(urdf_transforms, urdf_lidar_frame, urdf_imu_frame);
      std::stringstream ss;
      ss << T_lidar_imu.matrix();
      logger->info("URDF override T_lidar_imu ({} -> {}):\n{}", urdf_lidar_frame, urdf_imu_frame, ss.str());

      // Write override into config_sensors.json so all modules pick it up
      const std::string config_sensors_path = glim::GlobalConfig::get_config_path("config_sensors");
      config_sensors.override_param<Eigen::Isometry3d>("sensors", "T_lidar_imu", T_lidar_imu);
      config_sensors.save(config_sensors_path);
      // [P3 FIX 2026-07-10] The override only reaches the other modules VIA
      // DISK (each constructs its own Config from this path). Config::save
      // does not check the stream: on a read-only install prefix the write
      // silently fails, the INFO above still claims the override, and the
      // estimator runs with the stale checked-in extrinsic. Verify the
      // round-trip and fail LOUDLY — the extrinsic is safety-relevant.
      {
        glim::Config verify(config_sensors_path);
        const auto readback = verify.param<Eigen::Isometry3d>("sensors", "T_lidar_imu");
        if (!readback || !readback->isApprox(T_lidar_imu, 1e-9)) {
          logger->critical(
            "URDF T_lidar_imu override did NOT persist to {} (read-only install prefix?) — "
            "modules would silently use the stale checked-in extrinsic; aborting",
            config_sensors_path);
          throw std::runtime_error("config_sensors.json override write failed");
        }
      }
    } catch (const std::exception& e) {
      logger->error("Failed to compute T_lidar_imu from URDF: {}", e.what());
    }
  }

  // Setup GPU-based linearization
#ifdef BUILD_GTSAM_POINTS_GPU
  gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  // [P3 FIX 2026-07-14] Hand the operator-configured points_time_offset to the
  // TimeKeeper so it survives the absolute-time stamp overwrite. Previously the
  // offset was added to raw_points->stamp before process(), but the
  // absolute-time branch of replace_points_stamp overwrites the stamp with the
  // raw min point time and silently discarded it for Luminar/absolute clouds.
  time_keeper->set_point_time_offset(points_time_offset);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_odometry(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string odometry_estimation_so_name = config_odometry.param<std::string>("odometry_estimation", "so_name", "libodometry_estimation_cpu.so");
  spdlog::info("load {}", odometry_estimation_so_name);

  std::shared_ptr<glim::OdometryEstimationBase> odom = OdometryEstimationBase::load_module(odometry_estimation_so_name);
  if (!odom) {
    spdlog::critical("failed to load odometry estimation module");
    abort();
  }
  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, odom->requires_imu()));

  // Sub mapping
  if (config_ros.param<bool>("glim_ros", "enable_local_mapping", true)) {
    const std::string sub_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping")).param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
    if (!sub_mapping_so_name.empty()) {
      spdlog::info("load {}", sub_mapping_so_name);
      auto sub = SubMappingBase::load_module(sub_mapping_so_name);
      if (sub) {
        sub_mapping.reset(new AsyncSubMapping(sub));
      }
    }
  }

  // Global mapping
  if (config_ros.param<bool>("glim_ros", "enable_global_mapping", true)) {
    const std::string global_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_global_mapping")).param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
    if (!global_mapping_so_name.empty()) {
      spdlog::info("load {}", global_mapping_so_name);
      auto global = GlobalMappingBase::load_module(global_mapping_so_name);
      if (global) {
        global_mapping.reset(new AsyncGlobalMapping(global));
      }
    }
  }

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      if (extension.find("viewer") == std::string::npos && extension.find("monitor") == std::string::npos) {
        spdlog::warn("Extension modules are enabled!!");
        spdlog::warn("You must carefully check and follow the licenses of ext modules");

        try {
          const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
          spdlog::info("config_ext_path: {}", config_ext_path);
          glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);
        } catch (ament_index_cpp::PackageNotFoundError& e) {
          spdlog::warn("glim_ext package path was not found!!");
        }

        break;
      }
    }

    for (const auto& extension : *extensions) {
      spdlog::info("load {}", extension);
      auto ext_module = ExtensionModule::load_module(extension);
      if (ext_module == nullptr) {
        spdlog::error("failed to load {}", extension);
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions(*this);
          extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
        }
      }
    }
  }

  // ROS-related
  using std::placeholders::_1;

  // Online (live subscription) mapping passway. GLIM builds maps OFFLINE only
  // (glim_rosbag / glim_pcap_rosbag feed the callbacks directly and drive
  // timer_callback() manually), so by default we create NO live subscriptions
  // and NO wall timer. The offline tools do not use any of these. Set
  // glim_ros/enable_online_mapping=true to restore the legacy live path.
  this->online_mapping_enabled_ = config_ros.param<bool>("glim_ros", "enable_online_mapping", false);

  if (this->online_mapping_enabled_) {
    const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
    const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
    const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "");

    // Subscribers
    rclcpp::SensorDataQoS default_imu_qos;
    default_imu_qos.get_rmw_qos_profile().depth = 1000;
    auto qos = get_qos_settings(config_ros, "glim_ros", "imu_qos", default_imu_qos);
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, std::bind(&GlimROS::imu_callback, this, _1));

    qos = get_qos_settings(config_ros, "glim_ros", "points_qos");
    // Route the primary cloud through points_callback_live() so buffered aux
    // clouds are merged in before odometry sees the scan (parity with offline).
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, qos, std::bind(&GlimROS::points_callback_live, this, _1));

    // Subscribe to each auxiliary LiDAR topic and buffer its clouds. They are
    // merged into the primary scan on arrival of a primary cloud.
    if (aux_concat.enabled) {
      // [P2 FIX 2026-07-14] The live path (points_callback_live) merges aux at
      // PRIMARY arrival and has NO future-sweep pending-queue release: the right
      // sweep arriving +66..92 ms after the primary is structurally never
      // buffered yet, so online concat mapping silently degrades to front+left
      // (or throws under require_all_aux). The offline readers (glim_rosbag /
      // glim_pcap_rosbag) have the future-aware release; concat maps must be
      // built there. Refuse this unsupported combination loudly at startup
      // rather than ship a quietly-wrong merge.
      spdlog::critical(
        "glim_ros: online mapping (enable_online_mapping=true) combined with lidar_concat is "
        "unsupported — the live path has no future-sweep release and would drop the late aux "
        "sweep. Build the concatenated map offline (glim_rosbag / glim_pcap_rosbag). Refusing to start.");
      throw std::runtime_error("glim_ros: online mapping + lidar_concat is unsupported (no future-sweep release)");
    }
#ifdef BUILD_WITH_CV_BRIDGE
    qos = get_qos_settings(config_ros, "glim_ros", "image_qos");
    image_sub = image_transport::create_subscription(this, image_topic, std::bind(&GlimROS::image_callback, this, _1), "raw", qos.get_rmw_qos_profile());
#endif

    const std::string external_odom_topic = config_ros.param<std::string>("glim_ros", "external_odom_topic", "");
    if (!external_odom_topic.empty()) {
      rclcpp::QoS default_external_odom_qos(100);
      auto external_odom_qos = get_qos_settings(config_ros, "glim_ros", "external_odom_qos", default_external_odom_qos);
      external_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        external_odom_topic, external_odom_qos, std::bind(&GlimROS::external_odom_callback, this, _1));
      spdlog::info("subscribed to external odometry topic: {}", external_odom_topic);
    }

    for (const auto& sub : this->extension_subscriptions()) {
      spdlog::debug("subscribe to {}", sub->topic);
      sub->create_subscriber(*this);
    }

    // Start timer
    timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });
    spdlog::warn("ONLINE GLIM mapping ENABLED (glim_ros/enable_online_mapping=true) -- live subscriptions created");
  } else {
    spdlog::info(
      "online GLIM mapping DISABLED -- no live subscriptions or wall timer created. "
      "Build maps offline with glim_rosbag / glim_pcap_rosbag.");
  }

  spdlog::debug("initialized");
}

GlimROS::~GlimROS() {
  spdlog::debug("quit");
  extension_modules.clear();

  if (dump_on_unload) {
    std::string dump_path = "/tmp/dump";
    wait(true);
    save(dump_path);
  }
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (!GlobalConfig::instance()->has_param("meta", "imu_frame_id")) {
    spdlog::debug("auto-detecting IMU frame ID: {}", msg->header.frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "imu_frame_id", msg->header.frame_id);
  }

  if (std::abs(acc_scale) < 1e-6) {
    const double norm = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z).norm();
    if (norm > 7.0 && norm < 12.0) {
      acc_scale = 1.0;
      spdlog::debug("assuming [m/s^2] for acceleration unit (acc_scale={}, norm={})", acc_scale, norm);
    } else if (norm > 0.8 && norm < 1.2) {
      acc_scale = 9.80665;
      spdlog::debug("assuming [g] for acceleration unit (acc_scale={}, norm={})", acc_scale, norm);
    } else {
      acc_scale = 1.0;
      spdlog::warn("unexpected acceleration norm {}. assuming [m/s^2] for acceleration unit (acc_scale={})", norm, acc_scale);
    }
  }

  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
  const Eigen::Vector3d linear_acc = imu_input_rotation * (acc_scale * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
  const Eigen::Vector3d angular_vel = imu_input_rotation * Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  if (!time_keeper->validate_imu_stamp(imu_stamp)) {
    spdlog::warn("skip an invalid IMU data (stamp={})", imu_stamp);
    return;
  }

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  if (sub_mapping) {
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
}

#ifdef BUILD_WITH_CV_BRIDGE
void GlimROS::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  spdlog::trace("image: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (!GlobalConfig::instance()->has_param("meta", "image_frame")) {
    spdlog::debug("auto-detecting image frame ID: {}", msg->header.frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "image_frame", msg->header.frame_id);
  }

  cv_bridge::CvImagePtr cv_image;
  try {
    cv_image = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const std::exception& e) {
    // malformed frame (e.g. truncated capture assembly) -- skip, don't abort.
    // (Port of airacingtech glim_ros2@8b454f8.)
    spdlog::warn("dropping malformed image ({}x{}, {} bytes): {}", msg->width, msg->height, msg->data.size(), e.what());
    return;
  }

  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  odometry_estimation->insert_image(stamp, cv_image->image);
  if (sub_mapping) {
    sub_mapping->insert_image(stamp, cv_image->image);
  }
  if (global_mapping) {
    global_mapping->insert_image(stamp, cv_image->image);
  }
}
#endif

void GlimROS::aux_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t aux_index) {
  std::lock_guard<std::mutex> lock(aux_buffers_mutex);
  if (aux_index >= aux_concat.aux_sensors.size()) {
    return;
  }
  auto& aux = aux_concat.aux_sensors[aux_index];
  aux.buffer.push_back(glim_ros::buffer_aux_cloud(msg, aux_concat.float64_time_is_epoch_ns));
  while (aux.buffer.size() > aux.buffer_size) {
    aux.buffer.pop_front();
  }
}

void GlimROS::points_callback_live(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // Merge buffered auxiliary clouds into the primary scan (front + left + right
  // -> luminar_front frame), then hand the result to the estimator. The mutex
  // guards the aux buffers, which merge_clouds() reads via find_nearest().
  if (aux_concat.enabled && !aux_concat.aux_sensors.empty()) {
    // Primary point count BEFORE merge: lidar_concat appends aux bytes after the
    // primary, so these are the first points in the merged cloud. Pass it as the
    // epoch-rebase anchor so a multi-LiDAR sweep is not shifted late when an aux
    // scan started before the primary.
    const int primary_count = static_cast<int>(msg->width * msg->height);
    sensor_msgs::msg::PointCloud2::ConstSharedPtr merged;
    {
      std::lock_guard<std::mutex> lock(aux_buffers_mutex);
      // frame_diag_log wired through (review fix): without it the live node
      // silently used the default `false` even when config_sensors.json
      // enabled the per-frame CONCAT DEBUG evidence.
      merged = glim_ros::merge_clouds(msg, aux_concat.aux_sensors, aux_concat.time_threshold,
                                      aux_concat.require_all_aux, aux_concat.max_consecutive_aux_merge_failures,
                                      &aux_concat.consecutive_merge_failures, aux_concat.abort_on_merge_failure,
                                      aux_concat.frame_diag_log,
                                      aux_concat.luminar_time_threshold,
                                      aux_concat.float64_time_is_epoch_ns);
    }
    // nullptr = strict merge skipped this scan (require_all_aux); drop it.
    if (!merged) {
      return;
    }
    points_callback(merged, primary_count);
  } else {
    points_callback(msg);
  }
}

size_t GlimROS::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, int epoch_anchor_count, bool* ingested) {
  spdlog::trace("points: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (ingested) {
    *ingested = false;
  }
  if (!GlobalConfig::instance()->has_param("meta", "lidar_frame_id")) {
    spdlog::debug("auto-detecting LiDAR frame ID: {}", msg->header.frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "lidar_frame_id", msg->header.frame_id);
  }

  auto raw_points = glim::extract_raw_points(*msg, intensity_field, ring_field, epoch_anchor_count, float64_time_is_epoch_ns);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }

  if (flip_points_y) {
    for (auto& p : raw_points->points) {
      p.y() = -p.y();
    }
  }

  // [P3 FIX 2026-07-14] points_time_offset is now applied inside TimeKeeper
  // (see the constructor), AFTER any absolute-time stamp overwrite, so it is no
  // longer silently discarded for Luminar/absolute clouds.
  if (!time_keeper->process(raw_points)) {
    spdlog::warn("skip an invalid point cloud (stamp={})", raw_points->stamp);
    return 0;
  }
  auto preprocessed = preprocessor->preprocess(raw_points);

  if (keep_raw_points) {
    // note: Raw points are used only in extension modules for visualization purposes.
    //       If you need to reduce the memory footprint, you can safely comment out the following line.
    preprocessed->raw_points = raw_points;
  }

  odometry_estimation->insert_frame(preprocessed);
  if (ingested) {
    *ingested = true;
  }

  // Throttle offline bag playback on the SLOWEST stage, not just odometry.
  // glim_rosbag uses this return value to pace playback; reporting only the
  // odometry workload lets a fast front-end drain its queue while the bag keeps
  // flooding sub/global mapping, whose input queues (frames/submaps WITH points)
  // then grow unbounded and OOM. Take the max across all stages so playback
  // waits for the slowest. (Port of airacingtech glim_ros2@8b454f8.)
  size_t workload = odometry_estimation->workload();
  const size_t sub_wl = sub_mapping ? static_cast<size_t>(sub_mapping->workload()) : 0;
  const size_t global_wl = global_mapping ? static_cast<size_t>(global_mapping->workload()) : 0;
  if (sub_wl > workload) workload = sub_wl;
  if (global_wl > workload) workload = global_wl;
  spdlog::debug("workload={} (odom={} sub={} global={})", workload, odometry_estimation->workload(), sub_wl, global_wl);

  return workload;
}

void GlimROS::external_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!GlobalConfig::instance()->has_param("meta", "ins_frame_id") && !msg->child_frame_id.empty()) {
    spdlog::debug("auto-detecting INS frame ID: {}", msg->child_frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "ins_frame_id", msg->child_frame_id);
  }

  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  Eigen::Isometry3d T_world_ins = Eigen::Isometry3d::Identity();
  T_world_ins.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  const Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  T_world_ins.linear() = q.normalized().toRotationMatrix();

  odometry_estimation->insert_external_pose(stamp, T_world_ins);
}

bool GlimROS::needs_wait() {
  for (const auto& ext_module : extension_modules) {
    if (ext_module->needs_wait()) {
      return true;
    }
  }

  return false;
}

bool GlimROS::ok() const {
  for (const auto& ext_module : extension_modules) {
    if (!ext_module->ok()) {
      return false;
    }
  }
  return true;
}

void GlimROS::timer_callback() {
  if (!ok()) {
    rclcpp::shutdown();
  }

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  if (sub_mapping) {
    for (const auto& frame : marginalized_frames) {
      sub_mapping->insert_frame(frame);
    }

    auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
    }
  }
}

void GlimROS::wait(bool auto_quit) {
  spdlog::info("waiting for odometry estimation");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    spdlog::info("waiting for local mapping");
    sub_mapping->join();

    const auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      spdlog::info("waiting for global mapping");
      global_mapping->join();
    }
  }

  if (!auto_quit) {
    bool terminate = false;
    while (!terminate && rclcpp::ok()) {
      for (const auto& ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

void GlimROS::save(const std::string& path) {
  if (global_mapping) {
    // TODO(follow-up refactor): replace this needs_wait() quiescence-inference
    // flush with an explicit ExtensionModule::flush_at_end_of_sequence() hook
    // (a join()-equivalent for extensions, mirroring the core async stages).
    // An EOS signal lets the GNSS backend drain synchronously to completion and
    // resolves the "waiting for more GNSS" vs "permanently un-bracketable"
    // ambiguity directly -- removing pending_associable_ and the timeout below.
    // Ideally upstreamed to koide3 (its extensions share this latent save-race).
    //
    // Flush extension backends (e.g. gnss_global) that produce factors on their
    // own threads and deliver them only via on_smoother_update(). Wait until no
    // extension reports pending work, so their FINAL position/heading factors
    // are queued before we serialize. global_mapping->save() then runs a final
    // optimize() -- which fires on_smoother_update() and injects those queued
    // factors into the graph -- so they actually reach graph.bin / trajectories.
    // Bounded so a perpetually-busy extension can't hang the save.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (needs_wait() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (needs_wait()) {
      spdlog::warn("save(): extension still reports pending work after flush timeout; some final factors may be missing");
    }

    global_mapping->save(path);
  }
  for (auto& module : extension_modules) {
    module->at_exit(path);
  }
}

size_t GlimROS::num_submaps() {
  return global_mapping ? global_mapping->num_submaps() : 0;
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimROS);
