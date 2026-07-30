#include <cctype>
#include <glob.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <spdlog/spdlog.h>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_compression/sequential_compression_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/glim_ros.hpp>
#include <glim_ros/lidar_concat.hpp>
#include <glim_ros/ros_compatibility.hpp>

class SpeedCounter {
public:
  SpeedCounter() : last_sim_time(0.0), last_real_time(std::chrono::high_resolution_clock::now()) {}

  void update(const double& stamp) {
    const auto now = std::chrono::high_resolution_clock::now();
    if (now - last_real_time < std::chrono::seconds(5)) {
      return;
    }

    if (last_sim_time > 0.0) {
      const auto real = now - last_real_time;
      const auto sim = stamp - last_sim_time;
      const double playback_speed = sim / (std::chrono::duration_cast<std::chrono::nanoseconds>(real).count() / 1e9);
      spdlog::info("playback speed: {:.3f}x", playback_speed);
    }

    last_sim_time = stamp;
    last_real_time = now;
  }

private:
  double last_sim_time;
  std::chrono::high_resolution_clock::time_point last_real_time;
};

class KeyboardHandler {
public:
  KeyboardHandler() : paused_(false), active_(false) {
    if (isatty(STDIN_FILENO)) {
      tcgetattr(STDIN_FILENO, &original_termios_);
      struct termios raw = original_termios_;
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
      active_ = true;
    }
  }

  ~KeyboardHandler() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }
  }

  void update() {
    if (!active_) return;
    char c;
    while (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == ' ') {
        paused_ = !paused_;
        if (paused_) {
          spdlog::info("playback paused (press space to resume)");
        } else {
          spdlog::info("playback resumed");
        }
      }
    }
  }

  bool is_paused() const { return paused_; }

private:
  bool paused_;
  bool active_;
  struct termios original_termios_;
};

// [P3 FIX 2026-07-10] Mid-run hard errors (schema mismatches) previously
// converted into a "successful" partial save with exit code 0 — a pipeline
// could not distinguish "mapped 3 s then hit a schema error" from success.
static bool g_bag_hard_error = false;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);

  // List topics
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "/imu");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "/points");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "/image");
  const std::string external_odom_topic = config_ros.param<std::string>("glim_ros", "external_odom_topic", "");
  std::vector<std::string> topics = {imu_topic, points_topic, image_topic};
  if (!external_odom_topic.empty()) {
    topics.push_back(external_odom_topic);
  }

  // Load multi-LiDAR concatenation config
  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  auto concat_config = glim_ros::load_aux_sensors_from_config(config_sensors);
  const bool concat_enabled = concat_config.enabled;
  const double concat_time_threshold = concat_config.time_threshold;
  auto& aux_sensors = concat_config.aux_sensors;

  // [P3 FIX 2026-07-14] Reject an aux topic that equals the primary points
  // topic. Pass 1 (aux-first classification) and pass 2 (primary-first, see the
  // dispatch below) would classify such a message differently, desynchronizing
  // the ordinal plan so zero scans map. Fail loud at startup.
  for (const auto& sensor : aux_sensors) {
    if (sensor.topic == points_topic) {
      spdlog::error("lidar_concat: aux_topics contains the primary points topic '{}' — "
                    "this desynchronizes the two-pass ordinal plan (zero scans map); "
                    "refusing to start", points_topic);
      return 1;
    }
  }

  for (const auto& sensor : aux_sensors) {
    topics.push_back(sensor.topic);
  }

  rosbag2_storage::StorageFilter filter;
  spdlog::info("topics:");
  for (const auto& topic : topics) {
    spdlog::info("- {}", topic);
    filter.topics.push_back(topic);
  }

  //
  std::unordered_map<std::string, std::vector<glim::GenericTopicSubscription::Ptr>> subscription_map;
  for (const auto& sub : glim->extension_subscriptions()) {
    spdlog::info("- {} (ext)", sub->topic);
    filter.topics.push_back(sub->topic);
    subscription_map[sub->topic].push_back(sub);
  }

  // List input rosbag filenames
  std::vector<std::string> bag_filenames;

  for (int i = 1; i < argc; i++) {
    std::vector<std::string> filenames;
    glob_t globbuf;
    int ret = glob(argv[i], 0, nullptr, &globbuf);
    for (int i = 0; i < globbuf.gl_pathc; i++) {
      filenames.push_back(globbuf.gl_pathv[i]);
    }
    globfree(&globbuf);

    bag_filenames.insert(bag_filenames.end(), filenames.begin(), filenames.end());
  }
  // [P3 FIX 2026-07-09] Natural-numeric sort: plain lexicographic ordering
  // put split "_10" before "_2", dispatching a large backward time jump into
  // the estimator. Compare digit runs numerically, everything else bytewise.
  std::sort(bag_filenames.begin(), bag_filenames.end(), [](const std::string& a, const std::string& b) {
    size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
      if (std::isdigit(static_cast<unsigned char>(a[i])) && std::isdigit(static_cast<unsigned char>(b[j]))) {
        size_t i2 = i, j2 = j;
        while (i2 < a.size() && std::isdigit(static_cast<unsigned char>(a[i2]))) i2++;
        while (j2 < b.size() && std::isdigit(static_cast<unsigned char>(b[j2]))) j2++;
        const long long na = std::stoll(a.substr(i, i2 - i));
        const long long nb = std::stoll(b.substr(j, j2 - j));
        if (na != nb) return na < nb;
        i = i2; j = j2;
      } else {
        if (a[i] != b[j]) return a[i] < b[j];
        i++; j++;
      }
    }
    return a.size() < b.size();
  });

  // [P3 FIX 2026-07-09] An empty glob previously fell through to
  // rclcpp::spin() and sat forever with no data and no error.
  if (bag_filenames.empty()) {
    spdlog::critical("no rosbag files matched the given path(s) — aborting");
    return 1;
  }

  spdlog::info("bag_filenames:");
  for (const auto& bag_filename : bag_filenames) {
    spdlog::info("- {}", bag_filename);
  }

  // Playback range settings
  double delay = 0.0;
  glim->declare_parameter<double>("delay", delay);
  glim->get_parameter<double>("delay", delay);

  double start_offset = 0.0;
  glim->declare_parameter<double>("start_offset", start_offset);
  glim->get_parameter<double>("start_offset", start_offset);
  if (start_offset > 0.0 && bag_filenames.size() > 1) {
    spdlog::critical(
      "start_offset={} is ambiguous across {} input bag paths. Merge the "
      "inputs into one bag or run without start_offset; refusing to seek only "
      "the first path.",
      start_offset, bag_filenames.size());
    return 1;
  }

  double playback_duration = 0.0;
  glim->declare_parameter<double>("playback_duration", playback_duration);
  glim->get_parameter<double>("playback_duration", playback_duration);

  double playback_until = 0.0;
  glim->declare_parameter<double>("playback_until", playback_until);
  glim->get_parameter<double>("playback_until", playback_until);

  // Playback speed settings
  const double playback_speed = config_ros.param<double>("glim_ros", "playback_speed", 100.0);
  std::chrono::system_clock::time_point real_t0;
  rcutils_time_point_value_t bag_t0 = 0;
  SpeedCounter speed_counter;

  double end_time = std::numeric_limits<double>::max();
  glim->declare_parameter<double>("end_time", end_time);
  glim->get_parameter<double>("end_time", end_time);

  if (delay > 0.0) {
    spdlog::info("delaying {} sec", delay);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
  }

  // Keyboard handler for pause/resume
  KeyboardHandler keyboard;

  // Offline future-aware join. The right Iris sweep whose absolute point range
  // overlaps a front sweep can arrive later in bag order because its header has
  // a different acquisition phase.
  //
  // Preferred mode (two_pass_point_time_join): pass 1 below indexes every
  // LiDAR scan's absolute point-time range and bag location, then plans each
  // primary's aux selection by minimum endpoint-range delta within
  // luminar_time_threshold (header time only as tie-break). The streaming pass
  // merges a primary exactly when its planned sweeps have arrived — a known
  // bag time, not a heuristic wait — and maps unmatched primaries front-only
  // immediately with the miss reason recorded at planning time.
  //
  // Fallback mode (streaming wait), used when the primary topic lacks absolute
  // point times: queue primaries until every aux has a point-coherent match or
  // watermark, bounded by future_sweep_wait_timeout of bag time.
  //
  // Release policy — a primary is NEVER dropped by this queue in either mode.
  // It is released when the first of these holds:
  //   1. its planned sweeps have arrived (two-pass) / every aux is ready
  //      (fallback: match or watermark past the gate);
  //   2. the bag stream has advanced future_sweep_wait_timeout past the
  //      planned ready time (two-pass safety) / the primary's enqueue time
  //      (fallback timed release) — merge whichever aux aligned, possibly
  //      none. A dead/gappy aux stream can therefore only degrade coverage,
  //      never park primaries until EOF;
  //   3. end of input (force flush).
  // Header stamps are used for diagnostics (duplicate detection, logs) only.
  const auto header_stamp_ns = [](const builtin_interfaces::msg::Time& t) -> uint64_t {
    return static_cast<uint64_t>(static_cast<int64_t>(t.sec)) * 1000000000ull + t.nanosec;
  };
  // Plan identity is the per-topic BAG-RECORD ORDINAL (0-based count of
  // messages seen on that topic, in stream order), NOT header.stamp: duplicate
  // or zero header stamps would silently collapse map keys. Pass 1 and pass 2
  // read the same bags in the same order, so ordinals align exactly. For a
  // start_offset seek, the pass-1 bag times below initialize the pass-2 ordinal
  // counters to the first record at/after the seek boundary.
  struct PlannedAux {
    bool selected = false;
    uint64_t aux_ordinal = 0;     // identity of the chosen sweep in the stream
    double aux_bag_time_s = 0.0;  // when it arrives in the stream
    const char* miss_reason = "";
  };
  struct PlannedMerge {
    std::vector<PlannedAux> aux;
    double ready_bag_time_s = 0.0;  // max bag time over the selected sweeps
  };
  struct StoredAuxCloud {
    glim_ros::BufferedAuxCloud cloud;
    double bag_time_s = 0.0;  // arrival time, for stranded-entry GC
  };
  bool two_pass_active = false;
  std::unordered_map<uint64_t, PlannedMerge> merge_plan;           // key: primary ordinal
  std::vector<std::unordered_set<uint64_t>> planned_aux_ordinals;  // per-aux planned sweeps
  // Ordered by ordinal (== arrival order) so stranded entries can be
  // garbage-collected from the front by arrival time.
  std::vector<std::map<uint64_t, StoredAuxCloud>> planned_aux_store;
  // Pass-2 per-topic ordinal counters (must count every message on the topic,
  // exactly like pass 1 does).
  uint64_t primary_ordinal_next = 0;
  std::vector<uint64_t> aux_ordinal_next;
  std::vector<double> indexed_primary_bag_times_s;
  std::vector<std::vector<double>> indexed_aux_bag_times_s;
  bool seek_verification_pending = false;
  double seek_expected_first_primary_s = -1.0;
  struct PendingPrimaryScan {
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    double enqueue_bag_time_s = 0.0;
    uint64_t ordinal = 0;  // per-topic bag-record ordinal (two-pass plan key)
    // [P3 FIX 2026-07-14] Cache the primary's point-time range + header time at
    // enqueue so the streaming-fallback readiness poll never re-walks ~10^5
    // points per bag event.
    glim_ros::LuminarTimestampRangeNs range;
    double header_s = 0.0;
  };
  std::deque<PendingPrimaryScan> pending_primary_scans;
  double latest_bag_time_s = 0.0;  // stream time of the newest message read
  // Primary accounting. Invariant checked at EOF:
  //   primary_received == primary_forwarded + primary_strict_skipped +
  //                       primary_imu_skipped + still-pending
  uint64_t primary_received = 0;        // primary bag messages enqueued
  uint64_t primary_forwarded = 0;       // merged clouds ingested by GLIM
  uint64_t primary_strict_skipped = 0;  // merge_clouds nullptr (require_all_aux policy)
  uint64_t primary_imu_skipped = 0;     // released but rejected by GLIM ingestion
                                        // (extract_raw_points / TimeKeeper stamp validation)
  uint64_t primary_timed_release = 0;   // released by the bag-time bound, not readiness
  uint64_t primary_no_plan = 0;         // two-pass: primary absent from the pass-1 index
  uint64_t primary_released_incomplete = 0;  // two-pass: planned sweep never arrived
  const auto drain_pending_primaries = [&](bool force) -> bool {
    while (!pending_primary_scans.empty()) {
      const auto& pending = pending_primary_scans.front();
      const auto& primary = pending.msg;

      const PlannedMerge* plan = nullptr;
      if (two_pass_active) {
        const auto found = merge_plan.find(pending.ordinal);
        if (found != merge_plan.end()) {
          plan = &found->second;
        }
      }

      if (two_pass_active) {
        // Deterministic release: the planned sweeps arrive at bag times known
        // from pass 1. Wait only while a planned sweep is genuinely still
        // ahead of the stream; unmatched primaries release immediately.
        bool waiting = false;
        if (plan && !force) {
          for (size_t i = 0; i < aux_sensors.size() && !waiting; ++i) {
            const auto& pa = plan->aux[i];
            if (pa.selected && !planned_aux_store[i].count(pa.aux_ordinal)) {
              waiting = true;
            }
          }
        }
        if (waiting) {
          if (latest_bag_time_s < plan->ready_bag_time_s +
                                    concat_config.future_sweep_wait_timeout) {
            break;  // planned sweep is still ahead in the stream; keep order
          }
          // Safety: the stream passed the indexed arrival time yet the sweep
          // never showed up (index/stream mismatch or malformed message).
          ++primary_released_incomplete;
          if (primary_released_incomplete <= 10 || primary_released_incomplete % 100 == 0) {
            spdlog::warn(
              "lidar_concat: planned aux sweep(s) for primary (stamp={:.6f}) did not "
              "arrive within {:.3f}s past their indexed bag time; releasing with "
              "whichever arrived ({} incomplete release(s) so far)",
              glim_ros::stamp_to_sec(primary->header.stamp),
              concat_config.future_sweep_wait_timeout, primary_released_incomplete);
          }
        }
        // Stage exactly the planned sweeps for merge_clouds.
        for (size_t i = 0; i < aux_sensors.size(); ++i) {
          auto& aux = aux_sensors[i];
          aux.buffer.clear();
          if (!plan || !plan->aux[i].selected) {
            continue;
          }
          auto it = planned_aux_store[i].find(plan->aux[i].aux_ordinal);
          if (it != planned_aux_store[i].end()) {
            aux.buffer.push_back(std::move(it->second.cloud));
            planned_aux_store[i].erase(it);
          }
        }
        if (!plan) {
          ++primary_no_plan;
          if (primary_no_plan <= 10) {
            spdlog::warn(
              "lidar_concat: primary (stamp={:.6f}) missing from the pass-1 index; "
              "mapping front-only",
              glim_ros::stamp_to_sec(primary->header.stamp));
          }
        }
      } else if (!force && !glim_ros::aux_buffers_ready_for_primary(
                             pending.range, pending.header_s, aux_sensors,
                             concat_config.luminar_time_threshold)) {
        if (latest_bag_time_s - pending.enqueue_bag_time_s <
            concat_config.future_sweep_wait_timeout) {
          break;  // still inside the wait window; keep primary order
        }
        ++primary_timed_release;
        if (primary_timed_release <= 10 || primary_timed_release % 100 == 0) {
          spdlog::warn(
            "lidar_concat: releasing primary (stamp={:.6f}) after {:.3f}s bag-time wait "
            "without a point-coherent match/watermark for every aux; merging with "
            "whichever aux aligned ({} timed release(s) so far)",
            glim_ros::stamp_to_sec(primary->header.stamp),
            concat_config.future_sweep_wait_timeout, primary_timed_release);
        }
      } else if (
        !two_pass_active && force && concat_config.require_all_aux &&
        !glim_ros::aux_buffers_ready_for_primary(pending.range, pending.header_s, aux_sensors, concat_config.luminar_time_threshold)) {
        // At a playback-duration/EOF boundary the future side sweep needed by
        // a relative-time primary may be outside the selected window. Never
        // append an older, merely in-header-threshold sweep just to make the
        // final scan look complete: one deliberately skipped boundary scan is
        // preferable to injecting a 180-200 ms warped cloud into a
        // high-quality map.
        spdlog::warn(
          "lidar_concat: skipping final queued primary (stamp={:.6f}); input "
          "ended before every relative-time aux topic reached its future "
          "header watermark",
          pending.header_s);
        pending_primary_scans.pop_front();
        ++primary_strict_skipped;
        continue;
      }

      const int epoch_anchor_count =
        static_cast<int>(primary->width * primary->height);
      // [P3 FIX 2026-07-14] The strict-merge abort (require_all_aux past budget)
      // throws std::runtime_error from merge_clouds. Uncaught it unwound out of
      // read_bag to std::terminate — no glim->save(), no accounting. Convert it
      // to a controlled stop (partial dump kept, nonzero exit), matching GICP
      // and these readers' own hard-error policy. The un-merged primary stays in
      // pending_primary_scans, so it is counted as still_pending.
      sensor_msgs::msg::PointCloud2::ConstSharedPtr final_points;
      try {
        final_points = glim_ros::merge_clouds(
          primary, aux_sensors, concat_time_threshold,
          concat_config.require_all_aux,
          concat_config.max_consecutive_aux_merge_failures,
          &concat_config.consecutive_merge_failures,
          concat_config.abort_on_merge_failure,
          concat_config.frame_diag_log,
          concat_config.luminar_time_threshold,
          concat_config.float64_time_is_epoch_ns);
      } catch (const std::exception& e) {
        g_bag_hard_error = true;
        spdlog::error("lidar_concat: strict-merge abort: {} — stopping the run "
                      "(partial dump kept, exiting nonzero)", e.what());
        return false;
      }
      const double primary_header_s =
        glim_ros::stamp_to_sec(primary->header.stamp);
      pending_primary_scans.pop_front();

      size_t workload = 0;
      if (final_points) {
        bool ingested = false;
        workload = glim->points_callback(final_points, epoch_anchor_count, &ingested);
        if (ingested) {
          ++primary_forwarded;
        } else {
          ++primary_imu_skipped;
        }
      } else {
        ++primary_strict_skipped;
      }
      if (primary_header_s > end_time) {
        spdlog::info("end_time reached");
        return false;
      }
      if (workload > 5) {
        const size_t sleep_msec = (workload - 4) * 5;
        spdlog::debug("throttling: {} msec (workload={})", sleep_msec, workload);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msec));
      }
    }
    // GC: planned sweeps stranded because their primary released before they
    // arrived (safety-slack incomplete release). Ordinal order == arrival
    // order, so pruning from the front by stored arrival time is sufficient:
    // a sweep 10 s behind the stream can no longer be consumed.
    if (two_pass_active) {
      for (auto& store : planned_aux_store) {
        while (!store.empty() &&
               store.begin()->second.bag_time_s < latest_bag_time_s - 10.0) {
          store.erase(store.begin());
        }
      }
    }
    return true;
  };

  // Bag read function
  // Shared bag-open logic (streaming pass and the two-pass index both use it).
  const auto open_bag_reader =
    [](const std::string& bag_filename) -> std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> {
    rosbag2_storage::StorageOptions options;
    options.uri = bag_filename;

    bool is_mcap = bag_filename.size() > 5 && bag_filename.rfind(".mcap") == (bag_filename.size() - 5);
    if (is_mcap) {
      options.storage_id = "mcap";
    } else if (std::filesystem::is_directory(bag_filename)) {
      try {
        rosbag2_storage::MetadataIo metadata_io;
        const auto metadata = metadata_io.read_metadata(bag_filename);
        options.storage_id = metadata.storage_identifier;

        if (options.storage_id.empty()) {
          spdlog::warn("storage_identifier not found in metadata.yaml (uri={}), fallback to sqlite3", bag_filename);
          options.storage_id = "sqlite3";
        } else {
          spdlog::info("detected storage_id={} from metadata.yaml", options.storage_id);
        }
      } catch (const std::exception& e) {
        spdlog::warn("failed to read metadata.yaml (uri={}): {} (fallback to sqlite3)", bag_filename, e.what());
        options.storage_id = "sqlite3";
      }
    } else {
      options.storage_id = "sqlite3";
    }

    rosbag2_cpp::ConverterOptions converter_options;

    std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_;
    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader_->open(options, converter_options);

    if (reader_->get_metadata().compression_format != "") {
      spdlog::info("compression detected (format={})", reader_->get_metadata().compression_format);
      spdlog::info("opening bag with SequentialCompressionReader");
      reader_ = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
      reader_->open(options, converter_options);
    }
    return reader_;
  };

  const auto read_bag = [&](const std::string& bag_filename) {
    spdlog::info("opening {}", bag_filename);
    auto reader_ = open_bag_reader(bag_filename);
    auto& reader = *reader_;
    reader.set_filter(filter);

    const auto topics_and_types = reader.get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topic_type_map;
    for (const auto& topic : topics_and_types) {
      topic_type_map[topic.name] = topic.type;
    }

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> points_serialization;
    rclcpp::Serialization<nav_msgs::msg::Odometry> odometry_serialization;
#ifdef BUILD_WITH_CV_BRIDGE
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serialization;
#endif

    while (reader.has_next()) {
      if (!rclcpp::ok()) {
        return false;
      }
      rclcpp::spin_some(glim);

      const auto msg = reader.read_next();
      const std::string topic_type = topic_type_map[msg->topic_name];
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      if (real_t0.time_since_epoch().count() == 0) {
        real_t0 = std::chrono::system_clock::now();
      }

      const auto msg_time = get_msg_recv_timestamp(*msg);
      if (bag_t0 == 0) {
        bag_t0 = msg_time;
      }
      latest_bag_time_s = msg_time / 1e9;
      spdlog::debug("msg_time: {} ({} sec)", msg_time / 1e9, (msg_time - bag_t0) / 1e9);

      if (start_offset > 0.0) {
        spdlog::info("skipping msg for start_offset {}", start_offset);
        const rcutils_time_point_value_t seek_time = bag_t0 + start_offset * 1e9;
        reader.seek(seek_time);

        if (two_pass_active) {
          const double seek_time_s = seek_time / 1e9;
          primary_ordinal_next =
            std::distance(indexed_primary_bag_times_s.begin(), std::lower_bound(indexed_primary_bag_times_s.begin(), indexed_primary_bag_times_s.end(), seek_time_s));
          if (primary_ordinal_next < indexed_primary_bag_times_s.size()) {
            seek_expected_first_primary_s =
              indexed_primary_bag_times_s[primary_ordinal_next];
            seek_verification_pending = true;
          } else {
            spdlog::warn(
              "two-pass join seek has no indexed primary at/after {:.6f}; "
              "disabling the plan and using streaming matching",
              seek_time_s);
            two_pass_active = false;
          }
          for (size_t i = 0; i < aux_ordinal_next.size(); ++i) {
            aux_ordinal_next[i] =
              std::distance(indexed_aux_bag_times_s[i].begin(), std::lower_bound(indexed_aux_bag_times_s[i].begin(), indexed_aux_bag_times_s[i].end(), seek_time_s));
          }
          std::ostringstream aux_ordinals;
          for (size_t i = 0; i < aux_ordinal_next.size(); ++i) {
            if (i > 0) aux_ordinals << ",";
            aux_ordinals << aux_ordinal_next[i];
          }
          spdlog::info(
            "two-pass join seek alignment: seek_time={:.6f} "
            "primary_ordinal={} aux_ordinals={}",
            seek_time_s,
            primary_ordinal_next,
            aux_ordinals.str());
        }

        start_offset = 0.0;
        bag_t0 = 0;
        real_t0 = std::chrono::system_clock::from_time_t(0);
        continue;
      }

      if (playback_until > 0.0 && msg_time / 1e9 > playback_until) {
        spdlog::info("reached playback_until ({} < {})", msg_time / 1e9, playback_until);
        return false;
      }

      if (playback_duration > 0.0 && (msg_time - bag_t0) / 1e9 > playback_duration) {
        spdlog::info("reached playback_duration ({} > {})", (msg_time - bag_t0) / 1e9, playback_duration);
        return false;
      }

      // Pause/resume handling
      keyboard.update();
      if (keyboard.is_paused()) {
        auto pause_start = std::chrono::system_clock::now();
        while (keyboard.is_paused() && rclcpp::ok()) {
          rclcpp::spin_some(glim);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          keyboard.update();
        }
        if (!rclcpp::ok()) {
          return false;
        }
        // Adjust real_t0 to account for pause duration to avoid fast-forward
        real_t0 += std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::system_clock::now() - pause_start);
      }

      const auto bag_elapsed = std::chrono::nanoseconds(msg_time - bag_t0);
      while (rclcpp::ok() && playback_speed > 0.0 && (std::chrono::system_clock::now() - real_t0) * playback_speed < bag_elapsed) {
        const double real_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - real_t0).count() / 1e9;
        spdlog::debug("throttling (real_elapsed={} bag_elapsed={} playback_speed={})", real_elapsed, bag_elapsed.count() / 1e9, playback_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (!rclcpp::ok()) {
        return false;
      }

      // Check if this message is for an auxiliary LiDAR sensor
      bool is_aux_sensor = false;
      if (concat_enabled) {
        for (size_t aux_i = 0; aux_i < aux_sensors.size(); ++aux_i) {
          auto& aux = aux_sensors[aux_i];
          if (msg->topic_name == aux.topic) {
            if (topic_type != "sensor_msgs/msg/PointCloud2") {
              g_bag_hard_error = true;
        spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/PointCloud2 (topic={})", topic_type, msg->topic_name);
              return false;
            }
            // [P2 FIX 2026-07-14] Consume the per-topic ordinal BEFORE
            // deserializing so a corrupt CDR message still advances it and the
            // pass-1/pass-2 plan identity stays aligned (pass 1 also consumes an
            // ordinal for malformed messages). Guard the deserialize: an
            // uncaught throw here reached std::terminate — no glim->save(), no
            // accounting. Report it (streaming-pass contract) + hard error + skip.
            const uint64_t akey = two_pass_active ? aux_ordinal_next[aux_i]++ : 0;
            auto aux_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            try {
              points_serialization.deserialize_message(&serialized_msg, aux_msg.get());
            } catch (const std::exception& e) {
              g_bag_hard_error = true;
              spdlog::error("failed to deserialize aux PointCloud2 (topic={} ordinal={}): {} — "
                            "skipping (ordinal consumed to keep the two-pass plan aligned)",
                            msg->topic_name, akey, e.what());
              is_aux_sensor = true;
              break;
            }
            if (two_pass_active) {
              // Keep only sweeps the pass-1 plan selected; everything else is
              // known-unused and discarded immediately. Identity = per-topic
              // ordinal (counted for EVERY message on the topic, matching the
              // pass-1 counting rule exactly).
              if (planned_aux_ordinals[aux_i].count(akey)) {
                planned_aux_store[aux_i].emplace(
                  akey, StoredAuxCloud{glim_ros::buffer_aux_cloud(
                    aux_msg, concat_config.float64_time_is_epoch_ns), latest_bag_time_s});
              }
            } else {
              aux.buffer.push_back(glim_ros::buffer_aux_cloud(
                aux_msg, concat_config.float64_time_is_epoch_ns));
              while (aux.buffer.size() > aux.buffer_size) {
                aux.buffer.pop_front();
              }
            }
            is_aux_sensor = true;
            break;
          }
        }
      }

      if (is_aux_sensor) {
        // Pending primaries are drained once per message below.
      } else if (msg->topic_name == imu_topic) {
        if (topic_type != "sensor_msgs/msg/Imu") {
          g_bag_hard_error = true;
        spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/Imu (topic={})", topic_type, msg->topic_name);
          return false;
        }
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        // [P2 FIX 2026-07-14] Guard deserialize: an uncaught throw reached
        // std::terminate (no glim->save()). Report + hard error + skip the
        // callback (fall through so the per-message drain/timer still run).
        bool imu_deser_ok = true;
        try {
          imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
        } catch (const std::exception& e) {
          imu_deser_ok = false;
          g_bag_hard_error = true;
          spdlog::error("failed to deserialize Imu (topic={}): {} — skipping", msg->topic_name, e.what());
        }
        if (imu_deser_ok) {
          glim->imu_callback(imu_msg);
        }
      } else if (msg->topic_name == points_topic) {
        if (seek_verification_pending) {
          seek_verification_pending = false;
          constexpr double kSeekVerificationToleranceSec = 1e-6;
          const double seek_error_s =
            std::abs(latest_bag_time_s - seek_expected_first_primary_s);
          if (seek_error_s > kSeekVerificationToleranceSec) {
            spdlog::error(
              "two-pass seek verification failed: first streamed primary "
              "bag_time={:.9f}, indexed={:.9f}, error={:.3f} ms. Disabling "
              "the plan and falling back to streaming matching.",
              latest_bag_time_s, seek_expected_first_primary_s,
              seek_error_s * 1e3);
            for (size_t i = 0; i < planned_aux_store.size(); ++i) {
              for (auto& entry : planned_aux_store[i]) {
                aux_sensors[i].buffer.push_back(std::move(entry.second.cloud));
              }
              while (aux_sensors[i].buffer.size() > aux_sensors[i].buffer_size) {
                aux_sensors[i].buffer.pop_front();
              }
              planned_aux_store[i].clear();
            }
            merge_plan.clear();
            planned_aux_ordinals.clear();
            two_pass_active = false;
          } else {
            spdlog::info(
              "two-pass seek verification passed: first primary {:.9f} "
              "matches the indexed stream",
              latest_bag_time_s);
          }
        }
        if (topic_type != "sensor_msgs/msg/PointCloud2") {
          g_bag_hard_error = true;
        spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/PointCloud2 (topic={})", topic_type, msg->topic_name);
          return false;
        }
        auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        // [P2 FIX 2026-07-14] Guard deserialize (an uncaught throw reached
        // std::terminate, no glim->save()). On failure in two-pass concat mode
        // the primary ordinal MUST still be consumed so the pass-1/pass-2 plan
        // stays aligned (pass 1 consumes an ordinal for a malformed primary
        // too); the corrupt scan is simply never received/forwarded, which keeps
        // the primary-accounting invariant intact.
        bool points_deser_ok = true;
        try {
          points_serialization.deserialize_message(&serialized_msg, points_msg.get());
        } catch (const std::exception& e) {
          points_deser_ok = false;
          g_bag_hard_error = true;
          spdlog::error("failed to deserialize primary PointCloud2 (topic={}): {} — skipping",
                        msg->topic_name, e.what());
        }

        if (!points_deser_ok) {
          if (concat_enabled && !aux_sensors.empty()) {
            ++primary_ordinal_next;  // stay aligned with the pass-1 ordinal plan
          }
        } else if (concat_enabled && !aux_sensors.empty()) {
          PendingPrimaryScan scan;
          scan.msg = points_msg;
          scan.enqueue_bag_time_s = latest_bag_time_s;
          scan.ordinal = primary_ordinal_next++;
          scan.range = glim_ros::luminar_timestamp_range(
            *points_msg, concat_config.float64_time_is_epoch_ns);  // decode once
          scan.header_s = glim_ros::stamp_to_sec(points_msg->header.stamp);
          pending_primary_scans.push_back(std::move(scan));
          ++primary_received;
          // Drained once per message below.
        } else {
          const size_t workload = glim->points_callback(points_msg);
          if (glim_ros::stamp_to_sec(points_msg->header.stamp) > end_time) {
            spdlog::info("end_time reached");
            return false;
          }
          if (workload > 5) {
            const size_t sleep_msec = (workload - 4) * 5;
            spdlog::debug("throttling: {} msec (workload={})", sleep_msec, workload);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msec));
          }
        }
      } else if (!external_odom_topic.empty() && msg->topic_name == external_odom_topic) {
        if (topic_type != "nav_msgs/msg/Odometry") {
          g_bag_hard_error = true;
        spdlog::error("topic_type mismatch: {} != nav_msgs/msg/Odometry (topic={})", topic_type, msg->topic_name);
          return false;
        }
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        // [P2 FIX 2026-07-14] Guard deserialize (uncaught throw -> std::terminate).
        bool odom_deser_ok = true;
        try {
          odometry_serialization.deserialize_message(&serialized_msg, odom_msg.get());
        } catch (const std::exception& e) {
          odom_deser_ok = false;
          g_bag_hard_error = true;
          spdlog::error("failed to deserialize Odometry (topic={}): {} — skipping", msg->topic_name, e.what());
        }
        if (odom_deser_ok) {
          glim->external_odom_callback(odom_msg);
        }
      }
#ifdef BUILD_WITH_CV_BRIDGE
      else if (msg->topic_name == image_topic) {
        if (topic_type == "sensor_msgs/msg/Image") {
          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          image_serialization.deserialize_message(&serialized_msg, image_msg.get());
          glim->image_callback(image_msg);
        } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
          auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
          compressed_image_serialization.deserialize_message(&serialized_msg, compressed_image_msg.get());

          // [P2 FIX 2026-07-09] Guarded decode: a corrupt compressed frame
          // previously threw out of read_bag and killed the run before save().
          try {
            auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
            cv_bridge::toCvCopy(*compressed_image_msg, "bgr8")->toImageMsg(*image_msg);
            glim->image_callback(image_msg);
          } catch (const std::exception& e) {
            spdlog::warn("skipping malformed CompressedImage: {}", e.what());
          }
        } else {
          g_bag_hard_error = true;
        spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/(Image|CompressedImage) (topic={})", topic_type, msg->topic_name);
          return false;
        }
      }
#endif

      auto found = subscription_map.find(msg->topic_name);
      if (found != subscription_map.end()) {
        for (const auto& sub : found->second) {
          sub->insert_message_instance(serialized_msg, topic_type);
        }
      }

      // Drain once per message, not only on aux/primary arrivals: any message
      // (IMU at 125 Hz in particular) advances bag time, so the timed release
      // fires promptly even when an aux stream has died completely.
      if (concat_enabled && !aux_sensors.empty() && !pending_primary_scans.empty()) {
        if (!drain_pending_primaries(false)) {
          return false;
        }
      }

      glim->timer_callback();
      speed_counter.update(msg_time / 1e9);

      const auto t0 = std::chrono::high_resolution_clock::now();
      while (rclcpp::ok() && glim->needs_wait()) {
        rclcpp::spin_some(glim);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        spdlog::debug("throttling (waiting for odometry estimation)");
        if (std::chrono::high_resolution_clock::now() - t0 > std::chrono::seconds(1)) {
          spdlog::warn("throttling timeout (an extension module may be hanged)");
          break;
        }
      }
      if (!rclcpp::ok()) {
        return false;
      }
    }

    return true;
  };

  // ---------- Pass 1: point-time index + deterministic merge plan ----------
  if (concat_enabled && !aux_sensors.empty() && concat_config.two_pass_point_time_join) {
    // A scan's identity is its per-topic bag-record ORDINAL. Every message on
    // the topic gets an ordinal — including malformed ones (indexed with an
    // invalid range) — so the pass-2 counters, which see every message, stay
    // aligned. Vector position IS the ordinal for primary_index/aux_index.
    struct IndexedScan {
      double bag_time_s = 0.0;
      uint64_t header_ns = 0;  // diagnostic only (duplicate detection), not identity
      glim_ros::LuminarTimestampRangeNs range;
    };
    std::vector<IndexedScan> primary_index;
    std::vector<std::vector<IndexedScan>> aux_index(aux_sensors.size());

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_ser;
    rosbag2_storage::StorageFilter lidar_filter;
    lidar_filter.topics.push_back(points_topic);
    for (const auto& aux : aux_sensors) {
      lidar_filter.topics.push_back(aux.topic);
    }

    // A bounded replay should not deserialize every point of a multi-thousand
    // second bag merely to plan a few-minute mapping window. Probe the exact
    // first timestamp seen by the streaming filter (the same value pass 2 uses
    // as bag_t0), then index only a small pre-roll plus the requested range.
    // Ordinals are local to that indexed slice; the seek-alignment code above
    // initializes pass-2 counters from the indexed bag times.
    double index_seek_time_s = -1.0;
    double index_stop_time_s = std::numeric_limits<double>::infinity();
    if (start_offset > 0.0 && !bag_filenames.empty()) {
      try {
        auto probe = open_bag_reader(bag_filenames.front());
        probe->set_filter(filter);
        if (probe->has_next()) {
          const double stream_t0_s = get_msg_recv_timestamp(*probe->read_next()) / 1e9;
          const double requested_start_s = stream_t0_s + start_offset;
          const double safety_s = std::max(1.0, concat_config.future_sweep_wait_timeout + 0.5);
          index_seek_time_s = requested_start_s - safety_s;
          if (playback_duration > 0.0) {
            index_stop_time_s = requested_start_s + playback_duration + safety_s;
          } else if (playback_until > 0.0) {
            index_stop_time_s = playback_until + safety_s;
          }
          spdlog::info(
            "two-pass join: bounded index window [{:.6f}, {:.6f}] "
            "(requested start {:.6f}, safety {:.3f}s)",
            index_seek_time_s,
            index_stop_time_s,
            requested_start_s,
            safety_s);
        }
      } catch (const std::exception& e) {
        spdlog::warn(
          "two-pass join: failed to probe bounded index window: {}; "
          "falling back to full-bag indexing",
          e.what());
        index_seek_time_s = -1.0;
        index_stop_time_s = std::numeric_limits<double>::infinity();
      }
    }

    spdlog::info("two-pass join: indexing LiDAR point-time ranges ({} bag(s))", bag_filenames.size());
    bool index_ok = true;
    bool index_window_complete = false;
    for (const auto& bag_filename : bag_filenames) {
      try {
        auto reader_ = open_bag_reader(bag_filename);
        reader_->set_filter(lidar_filter);
        if (index_seek_time_s > 0.0) {
          reader_->seek(static_cast<rcutils_time_point_value_t>(index_seek_time_s * 1e9));
          // The requested bounded window can only live in the first input
          // stream reached after this seek; do not re-seek a later split bag.
          index_seek_time_s = -1.0;
        }
        while (reader_->has_next()) {
          if (!rclcpp::ok()) {
            index_ok = false;
            break;
          }
          const auto msg = reader_->read_next();
          IndexedScan s;
          s.bag_time_s = get_msg_recv_timestamp(*msg) / 1e9;
          if (s.bag_time_s > index_stop_time_s) {
            index_window_complete = true;
            break;
          }
          auto pc = std::make_shared<sensor_msgs::msg::PointCloud2>();
          try {
            const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            pc2_ser.deserialize_message(&serialized_msg, pc.get());
            s.header_ns = header_stamp_ns(pc->header.stamp);
            s.range = glim_ros::luminar_timestamp_range(
              *pc, concat_config.float64_time_is_epoch_ns);
          } catch (const std::exception&) {
            // Malformed message: still consumes an ordinal (invalid range);
            // the streaming pass reports the deserialization failure itself.
          }
          if (msg->topic_name == points_topic) {
            primary_index.push_back(s);
          } else {
            for (size_t i = 0; i < aux_sensors.size(); ++i) {
              if (msg->topic_name == aux_sensors[i].topic) {
                aux_index[i].push_back(s);
                break;
              }
            }
          }
        }
      } catch (const std::exception& e) {
        spdlog::error("two-pass join: failed to index {}: {}", bag_filename, e.what());
        index_ok = false;
      }
      if (!index_ok) {
        break;
      }
      if (index_window_complete) {
        break;
      }
    }

    size_t primary_valid = 0;
    for (const auto& s : primary_index) {
      if (s.range.valid) ++primary_valid;
    }
    if (!index_ok || primary_index.empty() || primary_valid * 10 < primary_index.size() * 9) {
      spdlog::warn(
        "two-pass join disabled: {}/{} primary scans carry absolute point times; "
        "falling back to the streaming future-sweep wait",
        primary_valid, primary_index.size());
    } else {
      indexed_primary_bag_times_s.reserve(primary_index.size());
      for (const auto& scan : primary_index) {
        indexed_primary_bag_times_s.push_back(scan.bag_time_s);
      }
      indexed_aux_bag_times_s.resize(aux_index.size());
      for (size_t i = 0; i < aux_index.size(); ++i) {
        indexed_aux_bag_times_s[i].reserve(aux_index[i].size());
        for (const auto& scan : aux_index[i]) {
          indexed_aux_bag_times_s[i].push_back(scan.bag_time_s);
        }
      }
      planned_aux_ordinals.assign(aux_sensors.size(), {});
      planned_aux_store.assign(aux_sensors.size(), {});
      aux_ordinal_next.assign(aux_sensors.size(), 0);
      // Loud duplicate-header diagnostic: ordinals make duplicates harmless
      // for identity, but duplicated/zero primary header stamps usually mean a
      // recorder or driver fault worth surfacing.
      {
        std::unordered_set<uint64_t> seen;
        uint64_t dup = 0, zero = 0;
        for (const auto& s : primary_index) {
          if (s.header_ns == 0) ++zero;
          else if (!seen.insert(s.header_ns).second) ++dup;
        }
        if (dup > 0 || zero > 0) {
          spdlog::warn(
            "two-pass join: primary topic has {} duplicate and {} zero header "
            "stamp(s); plan identity uses bag-record ordinals so matching is "
            "unaffected, but the recording should be investigated",
            dup, zero);
        }
      }
      // Per-aux candidates sorted by point-clock-corrected range start for
      // binary search. Only sweeps with a valid absolute range participate;
      // each carries its per-topic ordinal (= position in aux_index).
      struct AuxCandidate {
        uint64_t shifted_min_ns = 0;
        glim_ros::LuminarTimestampRangeNs shifted;
        uint64_t ordinal = 0;
        double header_s = 0.0;
        double bag_time_s = 0.0;
      };
      std::vector<std::vector<AuxCandidate>> aux_candidates(aux_sensors.size());
      for (size_t i = 0; i < aux_sensors.size(); ++i) {
        aux_candidates[i].reserve(aux_index[i].size());
        for (size_t k = 0; k < aux_index[i].size(); ++k) {
          const auto& s = aux_index[i][k];
          if (!s.range.valid) continue;
          AuxCandidate c;
          c.shifted = glim_ros::shifted_range(s.range, aux_sensors[i].point_time_offset);
          c.shifted_min_ns = c.shifted.min_ns;
          c.ordinal = k;
          c.header_s = static_cast<double>(s.header_ns) * 1e-9;
          c.bag_time_s = s.bag_time_s;
          aux_candidates[i].push_back(c);
        }
        std::sort(aux_candidates[i].begin(), aux_candidates[i].end(),
                  [](const AuxCandidate& a, const AuxCandidate& b) {
                    return a.shifted_min_ns < b.shifted_min_ns;
                  });
      }

      std::vector<uint64_t> planned_matched(aux_sensors.size(), 0);
      std::vector<uint64_t> planned_no_candidate(aux_sensors.size(), 0);
      std::vector<uint64_t> planned_exceeds_gate(aux_sensors.size(), 0);
      std::vector<uint64_t> planned_reserved(aux_sensors.size(), 0);
      std::vector<uint64_t> planned_no_absolute_time(aux_sensors.size(), 0);
      uint64_t planned_primary_invalid = 0;
      uint64_t planned_full = 0;
      // An aux topic with messages but ZERO valid absolute point-time ranges
      // (non-UINT8[8] layout or big-endian payload) cannot participate in the
      // point-time plan at all; say so once instead of per-primary noise.
      for (size_t i = 0; i < aux_sensors.size(); ++i) {
        if (aux_candidates[i].empty() && !aux_index[i].empty()) {
          spdlog::warn(
            "two-pass join: aux topic {} has {} message(s) but none carry a "
            "decodable absolute UINT8[8] point-time range — it cannot merge "
            "under a Luminar primary (the byte-append merge also requires an "
            "identical schema, so header matching is not a usable fallback); "
            "normalize the sensor layout upstream",
            aux_sensors[i].topic, aux_index[i].size());
        }
      }

      merge_plan.reserve(primary_index.size());
      for (size_t p_ord = 0; p_ord < primary_index.size(); ++p_ord) {
        const auto& p = primary_index[p_ord];
        PlannedMerge plan;
        plan.aux.resize(aux_sensors.size());
        if (!p.range.valid) {
          ++planned_primary_invalid;
          for (auto& pa : plan.aux) pa.miss_reason = "primary_range_invalid";
          merge_plan.emplace(p_ord, std::move(plan));
          continue;
        }
        const double primary_header_s = static_cast<double>(p.header_ns) * 1e-9;
        size_t selected_count = 0;
        for (size_t i = 0; i < aux_sensors.size(); ++i) {
          auto& pa = plan.aux[i];
          const auto& cands = aux_candidates[i];
          if (cands.empty()) {
            pa.miss_reason = aux_index[i].empty() ? "no_candidate" : "no_absolute_time";
            if (aux_index[i].empty()) {
              ++planned_no_candidate[i];
            } else {
              ++planned_no_absolute_time[i];
            }
            continue;
          }
          // Endpoint-range error is minimized in a small neighborhood of the
          // range-start lower bound; header time is only the tie-break.
          // An aux sweep is reserved by AT MOST ONE primary: without the
          // reservation, a duplicated primary could plan the same sweep twice,
          // the streaming pass would consume/erase it once, and the second
          // primary would stall to its safety slack and merge incomplete
          // despite a "matched" plan. Select the best in-gate UNRESERVED
          // candidate; sweeps are ~50 ms apart with a 10 ms gate, so at most
          // one candidate is in-gate and a reserved hit means front-only.
          auto lb = std::lower_bound(
            cands.begin(), cands.end(), p.range.min_ns,
            [](const AuxCandidate& c, uint64_t v) { return c.shifted_min_ns < v; });
          size_t lo = (lb - cands.begin() >= 3) ? (lb - cands.begin() - 3) : 0;
          size_t hi = std::min(cands.size(), static_cast<size_t>(lb - cands.begin()) + 3);
          const AuxCandidate* best = nullptr;
          double best_delta = std::numeric_limits<double>::infinity();
          double best_header = std::numeric_limits<double>::infinity();
          bool in_gate_reserved = false;
          for (size_t k = lo; k < hi; ++k) {
            const double delta = glim_ros::endpoint_delta_seconds(p.range, cands[k].shifted);
            if (planned_aux_ordinals[i].count(cands[k].ordinal)) {
              if (delta <= concat_config.luminar_time_threshold) {
                in_gate_reserved = true;
              }
              continue;  // reserved by an earlier primary
            }
            const double header_abs = std::abs(
              cands[k].header_s + aux_sensors[i].match_time_offset - primary_header_s);
            if (delta < best_delta ||
                (delta == best_delta && header_abs < best_header)) {
              best = &cands[k];
              best_delta = delta;
              best_header = header_abs;
            }
          }
          if (best && best_delta <= concat_config.luminar_time_threshold) {
            pa.selected = true;
            pa.aux_ordinal = best->ordinal;
            pa.aux_bag_time_s = best->bag_time_s;
            plan.ready_bag_time_s = std::max(plan.ready_bag_time_s, best->bag_time_s);
            planned_aux_ordinals[i].insert(best->ordinal);
            ++planned_matched[i];
            ++selected_count;
          } else if (in_gate_reserved) {
            pa.miss_reason = "candidate_reserved";
            ++planned_reserved[i];
          } else {
            pa.miss_reason = "exceeds_gate";
            ++planned_exceeds_gate[i];
          }
        }
        if (selected_count == aux_sensors.size()) ++planned_full;
        merge_plan.emplace(p_ord, std::move(plan));
      }

      two_pass_active = true;
      spdlog::info(
        "two-pass join plan: {} primaries ({} without absolute point times), "
        "full {}-aux merges planned for {} ({:.1f}%)",
        primary_index.size(), planned_primary_invalid, aux_sensors.size(), planned_full,
        primary_index.empty() ? 0.0 : 100.0 * planned_full / primary_index.size());
      for (size_t i = 0; i < aux_sensors.size(); ++i) {
        spdlog::info(
          "two-pass join plan [{}]: matched={} no_candidate={} exceeds_gate={} "
          "candidate_reserved={} no_absolute_time={} "
          "(gate {:.3f}s, header only tie-break, one primary per sweep)",
          aux_sensors[i].topic, planned_matched[i], planned_no_candidate[i],
          planned_exceeds_gate[i], planned_reserved[i], planned_no_absolute_time[i],
          concat_config.luminar_time_threshold);
        if (planned_reserved[i] > 0) {
          spdlog::warn(
            "two-pass join: {} primary scan(s) lost the in-gate {} sweep to an "
            "earlier primary (candidate_reserved) — usually duplicated primary "
            "messages in the recording; those primaries merge without this aux "
            "immediately instead of stalling",
            planned_reserved[i], aux_sensors[i].topic);
        }
      }
    }
  }

  // Read all rosbags
  // glim_rosbag is an offline batch runner: reaching the true end of every
  // input bag must drain, optimize, save, and return without an interactive
  // Ctrl-C.  A bounded playback already forced this path through read_bag()'s
  // stop condition, which hid the full-run-only hang.  Interactive inspection
  // remains available by explicitly passing -p auto_quit:=false.
  bool auto_quit = true;
  glim->declare_parameter<bool>("auto_quit", auto_quit);
  glim->get_parameter<bool>("auto_quit", auto_quit);

  std::string dump_path = "/tmp/dump";
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  for (const auto& bag_filename : bag_filenames) {
    if (!read_bag(bag_filename)) {
      auto_quit = true;
      break;
    }
  }

  if (!pending_primary_scans.empty()) {
    spdlog::info(
      "lidar_concat: flushing {} queued primary scan(s) at end of input",
      pending_primary_scans.size());
    if (!drain_pending_primaries(true)) {
      auto_quit = true;
    }
  }

  if (concat_enabled && !aux_sensors.empty()) {
    spdlog::info(
      "lidar_concat primary accounting: received={} forwarded={} strict_skipped={} "
      "imu_skipped={} timed_release={} no_plan={} released_incomplete={} still_pending={}",
      primary_received, primary_forwarded, primary_strict_skipped,
      primary_imu_skipped, primary_timed_release, primary_no_plan,
      primary_released_incomplete, pending_primary_scans.size());
    if (primary_received != primary_forwarded + primary_strict_skipped +
                              primary_imu_skipped + pending_primary_scans.size()) {
      spdlog::error(
        "lidar_concat primary accounting MISMATCH: a primary scan was lost on an "
        "unaccounted path — this violates the never-drop-front contract");
      g_bag_hard_error = true;
    }
  }

  if (!auto_quit) {
    rclcpp::spin(glim);
  }

  glim->wait(auto_quit);
  try {
    glim->save(dump_path);
  } catch (const std::exception& e) {
    g_bag_hard_error = true;
    spdlog::critical(
      "GLIM dump save failed after retaining all recoverable submaps: {}", e.what());
  }

  const size_t num_submaps = glim->num_submaps();
  if (num_submaps == 0) {
    spdlog::critical(
      "mapping produced zero submaps — input was empty/filtered or odometry "
      "never initialized; partial dump kept, exiting nonzero");
    return 1;
  }

  if (!glim->ok()) {
    spdlog::error("run rejected by a mapping quality/safety extension — partial dump saved, exiting nonzero");
    return 1;
  }

  // [P3 FIX 2026-07-10] partial dump is kept, but exit nonzero on hard errors.
  if (g_bag_hard_error) {
    spdlog::error("run completed WITH hard errors (see log) — partial dump saved, exiting nonzero");
    return 1;
  }
  return 0;
}
