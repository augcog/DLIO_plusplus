#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_compression/sequential_compression_reader.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/glim_ros.hpp>
#include <glim_ros/iris_pcap_reader.hpp>
#include <glim_ros/lidar_concat.hpp>
#include <glim_ros/ros_compatibility.hpp>

namespace {

using glim_ros::AssembledScan;
using glim_ros::AuxLidarSensor;
using glim_ros::IrisLidarConfig;
using glim_ros::IrisPcapConfig;
using glim_ros::IrisPcapReader;

void shift_luminar_point_timestamps(sensor_msgs::msg::PointCloud2& cloud, int64_t shift_ns) {
  if (shift_ns == 0) {
    return;
  }

  const auto field_it = std::find_if(cloud.fields.begin(), cloud.fields.end(), [](const auto& f) {
    return (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") &&
           f.datatype == sensor_msgs::msg::PointField::UINT8 && f.count == 8;
  });
  if (field_it == cloud.fields.end()) {
    return;
  }

  const size_t n = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  const size_t off = field_it->offset;
  const size_t step = cloud.point_step;
  for (size_t i = 0; i < n; i++) {
    uint8_t* ptr = cloud.data.data() + i * step + off;
    uint64_t raw = 0;
    std::memcpy(&raw, ptr, sizeof(raw));
    const int64_t shifted = std::max<int64_t>(0, static_cast<int64_t>(raw) + shift_ns);
    const uint64_t out = static_cast<uint64_t>(shifted);
    std::memcpy(ptr, &out, sizeof(out));
  }
}

IrisPcapConfig load_pcap_config_from_json(const glim::Config& cfg) {
  IrisPcapConfig out;
  out.iris_data_src_port = static_cast<uint16_t>(cfg.param<int>("pcap", "iris_data_src_port", 4371));
  out.inactivity_sec = cfg.param<double>("pcap", "inactivity_sec", 0.05);
  // glim::Config supports int/double/bool/string and vector<double>/vector<string>
  // robustly. Read scan_reorder_lookahead_ns as a double (in ns) for safety,
  // and dst_ports as a vector<double> for the same reason.
  out.scan_reorder_lookahead_ns =
    static_cast<uint64_t>(cfg.param<double>("pcap", "scan_reorder_lookahead_ns", 200'000'000.0));

  auto lidar_ips = cfg.param<std::vector<std::string>>("pcap", "lidar_ips", {});
  auto lidar_topics = cfg.param<std::vector<std::string>>("pcap", "lidar_topics", {});
  auto lidar_frames = cfg.param<std::vector<std::string>>("pcap", "lidar_frame_ids", {});
  auto lidar_ports = cfg.param<std::vector<double>>("pcap", "lidar_dst_ports", {});
  if (lidar_ips.size() != lidar_topics.size() ||
      lidar_ips.size() != lidar_frames.size() ||
      lidar_ips.size() != lidar_ports.size()) {
    spdlog::error("config_pcap.json: lidar_ips/lidar_topics/lidar_frame_ids/lidar_dst_ports must all be the same length");
    return out;
  }
  for (size_t i = 0; i < lidar_ips.size(); i++) {
    IrisLidarConfig l;
    l.ip = lidar_ips[i];
    l.topic = lidar_topics[i];
    l.frame_id = lidar_frames[i];
    l.dst_port = static_cast<uint16_t>(lidar_ports[i]);
    out.lidars.push_back(l);
  }
  return out;
}

// Unified-clock event queued for dispatch.
struct Event {
  uint64_t t_ns;
  enum class Source { PCAP_SCAN, BAG_MSG } source;
  // PCAP_SCAN payload
  AssembledScan scan;
  // BAG_MSG payload
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
  std::string topic_type;
  // Tie-breaker for ordering: bag before pcap on identical t_ns (favors IMU
  // history feeding the estimator before a same-timestamp scan).
  uint64_t order;
};

struct EventGreater {
  bool operator()(const Event& a, const Event& b) const {
    if (a.t_ns != b.t_ns) return a.t_ns > b.t_ns;
    return a.order > b.order;
  }
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: glim_pcap_rosbag <pcap_path> <mcap_path>" << std::endl;
    return 0;
  }
  const std::string pcap_path = argv[1];
  const std::string mcap_path = argv[2];

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // Early file-sink setup so all subsequent spdlog output is captured. The
  // dump_path is fetched off the just-created node below, but we already
  // know argc/argv. We re-attach to per-module loggers after GlimROS is
  // constructed (it creates [odom], [submap], [global], etc.).
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> early_file_sink;

  auto glim = std::make_shared<glim::GlimROS>(options);

  // Topic config
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));
  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "/imu");
  const std::string primary_points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "/points");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "/image");

  // Multi-LiDAR concat config (URDF transforms etc.)
  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  auto concat_config = glim_ros::load_aux_sensors_from_config(config_sensors);
  const bool concat_enabled = concat_config.enabled;
  const double concat_time_threshold = concat_config.time_threshold;
  auto& aux_sensors = concat_config.aux_sensors;

  std::unordered_set<std::string> aux_topic_set;
  for (const auto& s : aux_sensors) aux_topic_set.insert(s.topic);

  // PCAP source config — single source of truth for inactivity_sec.
  glim::Config config_pcap(glim::GlobalConfig::get_config_path("config_pcap"));
  IrisPcapConfig pcap_cfg = load_pcap_config_from_json(config_pcap);

  IrisPcapReader pcap;
  try {
    pcap.open(pcap_path, pcap_cfg);
  } catch (const std::exception& e) {
    spdlog::error("failed to open pcap: {}", e.what());
    return 1;
  }
  spdlog::info("pcap: opened '{}'", pcap_path);
  spdlog::info("pcap: first packet epoch ns = {}", pcap.peek_first_pcap_epoch_ns());
  spdlog::info("pcap: last  packet epoch ns = {}", pcap.peek_last_pcap_epoch_ns());

  // Build the bag filter union (additive — extension subs may also re-add
  // primary_points_topic, but bag-native primary points are dropped at
  // dispatch by an explicit guard).
  rosbag2_storage::StorageFilter filter;
  std::unordered_set<std::string> filter_topic_set;
  auto add_filter = [&](const std::string& t) {
    if (t.empty()) return;
    if (filter_topic_set.insert(t).second) filter.topics.push_back(t);
  };
  add_filter(imu_topic);
  add_filter(image_topic);
  for (const auto& s : aux_sensors) add_filter(s.topic);

  std::unordered_map<std::string, std::vector<glim::GenericTopicSubscription::Ptr>> subscription_map;
  for (const auto& sub : glim->extension_subscriptions()) {
    spdlog::info("- {} (ext)", sub->topic);
    add_filter(sub->topic);
    subscription_map[sub->topic].push_back(sub);
  }

  spdlog::info("topics:");
  for (const auto& t : filter.topics) spdlog::info("- {}", t);

  // Open the rosbag. If a directory was passed but its metadata.yaml is bad
  // or absent, fall back to the first *.mcap file inside.
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = mcap_path;
  const bool is_mcap_file = mcap_path.size() > 5 && mcap_path.rfind(".mcap") == (mcap_path.size() - 5);
  if (is_mcap_file) {
    storage_opts.storage_id = "mcap";
  } else if (std::filesystem::is_directory(mcap_path)) {
    bool metadata_ok = false;
    try {
      rosbag2_storage::MetadataIo metadata_io;
      const auto md = metadata_io.read_metadata(mcap_path);
      if (!md.storage_identifier.empty()) {
        storage_opts.storage_id = md.storage_identifier;
        spdlog::info("detected storage_id={} from metadata.yaml", storage_opts.storage_id);
        metadata_ok = true;
      } else {
        spdlog::warn("storage_identifier missing in metadata.yaml");
      }
    } catch (const std::exception& e) {
      spdlog::warn("metadata.yaml read failed ({})", e.what());
    }
    if (!metadata_ok) {
      // Find a .mcap file inside the directory and open it directly.
      std::string fallback;
      for (const auto& entry : std::filesystem::directory_iterator(mcap_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
          fallback = entry.path().string();
          break;
        }
      }
      if (fallback.empty()) {
        spdlog::error("could not find a .mcap file inside directory '{}'", mcap_path);
        return 1;
      }
      spdlog::warn("falling back to direct mcap file '{}'", fallback);
      storage_opts.uri = fallback;
      storage_opts.storage_id = "mcap";
    }
  } else {
    storage_opts.storage_id = "mcap";
  }

  rosbag2_cpp::ConverterOptions conv_opts;
  std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_;
  reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  reader_->open(storage_opts, conv_opts);
  if (reader_->get_metadata().compression_format != "") {
    spdlog::info("compression detected (format={}); using SequentialCompressionReader",
                 reader_->get_metadata().compression_format);
    reader_ = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
    reader_->open(storage_opts, conv_opts);
  }
  auto& reader = *reader_;
  reader.set_filter(filter);

  std::unordered_map<std::string, std::string> topic_type_map;
  for (const auto& t : reader.get_all_topics_and_types()) topic_type_map[t.name] = t.type;

  // Time alignment (one-shot at startup).
  const auto bag_meta = reader.get_metadata();
  const uint64_t bag_start_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(bag_meta.starting_time.time_since_epoch()).count());
  const uint64_t bag_end_ns = bag_start_ns + static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(bag_meta.duration).count());
  const uint64_t pcap_first_ns = pcap.peek_first_pcap_epoch_ns();
  const uint64_t pcap_last_ns = pcap.peek_last_pcap_epoch_ns();  // UINT64_MAX (unbounded)
  const uint64_t tail_ns = static_cast<uint64_t>(pcap_cfg.inactivity_sec * 1e9);

  enum class AnchorMode { NATURAL, FORCE_TO_BAG_START };
  AnchorMode anchor_mode = AnchorMode::NATURAL;
  uint64_t pcap_window_lo = 0, pcap_window_hi = std::numeric_limits<uint64_t>::max();
  uint64_t bag_trim_lo = 0, bag_trim_hi = std::numeric_limits<uint64_t>::max();
  bool bag_trim_active = false;

  const bool overlap = (std::max(bag_start_ns, pcap_first_ns) <= std::min(bag_end_ns + tail_ns, pcap_last_ns));
  if (overlap) {
    anchor_mode = AnchorMode::NATURAL;
    pcap_window_lo = std::max(bag_start_ns, pcap_first_ns);
    pcap_window_hi = std::min(bag_end_ns + tail_ns, pcap_last_ns);
    bag_trim_lo = std::max(bag_start_ns, pcap_first_ns);
    bag_trim_hi = std::min(bag_end_ns, pcap_last_ns);
    bag_trim_active = true;
    spdlog::info("alignment: NATURAL (overlap window ns [{}, {}])", pcap_window_lo, pcap_window_hi);
  } else {
    anchor_mode = AnchorMode::FORCE_TO_BAG_START;
    pcap_window_lo = pcap_first_ns;
    const uint64_t bag_duration_ns = (bag_end_ns - bag_start_ns) + tail_ns;
    pcap_window_hi = std::min(pcap_last_ns, pcap_first_ns + bag_duration_ns);
    bag_trim_active = false;
    spdlog::info("alignment: FORCE_TO_BAG_START (pcap window ns [{}, {}], will anchor first scan to bag_start={})",
                 pcap_window_lo, pcap_window_hi, bag_start_ns);
  }
  pcap.set_window(pcap_window_lo, pcap_window_hi);

  // PTP -> ROS shift, applied to all assembled scans before they enter the heap.
  // Set on the first scan emitted from the assembler.
  bool ptp_shift_set = false;
  int64_t ptp_to_ros_shift_ns = 0;

  auto apply_ptp_shift = [&](AssembledScan& s) {
    if (!ptp_shift_set) {
      const uint64_t scan_ptp_ns = s.t_ns;  // first ray's PTP timestamp
      if (anchor_mode == AnchorMode::FORCE_TO_BAG_START) {
        // Mirrors scripts/merge_luminar_pcap.py force-anchor mode:
        // force_anchor_ns - scan_ptp_ns.
        ptp_to_ros_shift_ns = static_cast<int64_t>(bag_start_ns) - static_cast<int64_t>(scan_ptp_ns);
      } else {
        // NATURAL: shift = first_packet_wall_ns - first_ray_ptp_ns. We use
        // the FIRST packet's wall-clock time (not the last packet's) so the
        // shift represents the constant offset between wall-clock and PTP
        // master clocks, with no scan-duration bias. Using last-packet time
        // (as the Python reference's natural-anchor path does) bakes in ~50ms of scan
        // accumulation latency, which manifests downstream as IMU/lidar
        // sync error proportional to vehicle velocity (~0.15m at 3m/s).
        ptp_to_ros_shift_ns = static_cast<int64_t>(s.wall_clock_first_packet_ns) -
                              static_cast<int64_t>(scan_ptp_ns);
      }
      ptp_shift_set = true;
      spdlog::info("ptp_to_ros_shift_ns = {} (first_pkt_wall={}, last_pkt_wall={}, first_ray_ptp={})",
                   ptp_to_ros_shift_ns, s.wall_clock_first_packet_ns,
                   s.wall_clock_last_packet_ns, scan_ptp_ns);
    }
    shift_luminar_point_timestamps(*s.cloud, ptp_to_ros_shift_ns);
    int64_t out_ns = static_cast<int64_t>(s.t_ns) + ptp_to_ros_shift_ns;
    if (out_ns < 0) out_ns = 0;
    s.t_ns = static_cast<uint64_t>(out_ns);
    s.cloud->header.stamp.sec = static_cast<int32_t>(s.t_ns / 1'000'000'000ULL);
    s.cloud->header.stamp.nanosec = static_cast<uint32_t>(s.t_ns % 1'000'000'000ULL);
  };

  // Runtime parameters (same names as glim_rosbag).
  double delay = 0.0;
  glim->declare_parameter<double>("delay", delay);
  glim->get_parameter<double>("delay", delay);
  double start_offset = 0.0;
  glim->declare_parameter<double>("start_offset", start_offset);
  glim->get_parameter<double>("start_offset", start_offset);
  double playback_until = 0.0;
  glim->declare_parameter<double>("playback_until", playback_until);
  glim->get_parameter<double>("playback_until", playback_until);
  double playback_duration = 0.0;
  glim->declare_parameter<double>("playback_duration", playback_duration);
  glim->get_parameter<double>("playback_duration", playback_duration);
  double end_time = std::numeric_limits<double>::max();
  glim->declare_parameter<double>("end_time", end_time);
  glim->get_parameter<double>("end_time", end_time);
  bool auto_quit = false;
  glim->declare_parameter<bool>("auto_quit", auto_quit);
  glim->get_parameter<bool>("auto_quit", auto_quit);
  std::string dump_path = "/tmp/dump";
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  // /clock publisher — drives sim-time for any downstream consumer (rviz,
  // tf2 listeners, offline_viewer) so they don't see TF stamps as "old data"
  // relative to wall-clock. Default ON; pass -p publish_clock:=false to skip.
  bool publish_clock = true;
  glim->declare_parameter<bool>("publish_clock", publish_clock);
  glim->get_parameter<bool>("publish_clock", publish_clock);
  std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock>> clock_pub;
  if (publish_clock) {
    clock_pub = glim->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
    spdlog::info("publishing /clock from merged dispatch time");
  }

  // log_path: if empty, defaults to <dump_path>/run.log. Pass a literal "-"
  // to disable file logging entirely.
  std::string log_path;
  glim->declare_parameter<std::string>("log_path", log_path);
  glim->get_parameter<std::string>("log_path", log_path);
  if (log_path.empty()) log_path = dump_path + "/run.log";
  if (log_path != "-") {
    try {
      std::filesystem::create_directories(std::filesystem::path(log_path).parent_path());
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path, true);
      file_sink->set_level(spdlog::level::trace);
      spdlog::apply_all([&](std::shared_ptr<spdlog::logger> l) {
        l->sinks().push_back(file_sink);
      });
      spdlog::info("logging to {}", log_path);
      // Re-emit topic list now that the sink is attached so the log file
      // captures it.
      spdlog::info("imu_topic={} primary_points_topic={}", imu_topic, primary_points_topic);
      for (const auto& t : filter.topics) spdlog::info("filter topic: {}", t);
    } catch (const std::exception& e) {
      spdlog::warn("failed to open log file '{}': {}", log_path, e.what());
    }
  }

  if (delay > 0.0) {
    spdlog::info("delay {} sec", delay);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
  }

  // Serializers
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_ser;
#ifdef BUILD_WITH_CV_BRIDGE
  rclcpp::Serialization<sensor_msgs::msg::Image> image_ser;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_ser;
#endif

  // Heap of dispatchable events.
  std::priority_queue<Event, std::vector<Event>, EventGreater> heap;
  uint64_t order_counter = 0;
  bool pcap_eof_flushed = false;
  bool bag_eof = false;

  // Dispatch-side state
  uint64_t merged_t0 = 0;
  bool merged_t0_set = false;
  bool dispatch_t0_set = false;
  uint64_t dispatch_t0_ns = 0;
  bool primary_bag_warned = false;

  auto pull_pcap = [&]() {
    while (true) {
      auto s = pcap.next();
      if (!s) {
        if (!pcap_eof_flushed) {
          pcap_eof_flushed = true;
          for (auto& f : pcap.flush_all()) {
            apply_ptp_shift(f);
            Event e;
            e.t_ns = f.t_ns;
            e.source = Event::Source::PCAP_SCAN;
            e.scan = std::move(f);
            e.order = order_counter++;
            heap.push(std::move(e));
          }
        }
        return false;
      }
      apply_ptp_shift(*s);
      Event e;
      e.t_ns = s->t_ns;
      e.source = Event::Source::PCAP_SCAN;
      e.scan = std::move(*s);
      e.order = order_counter++;
      heap.push(std::move(e));
      return true;
    }
  };

  auto pull_bag = [&]() {
    while (reader.has_next()) {
      auto msg = reader.read_next();
      const uint64_t recv_ns = static_cast<uint64_t>(get_msg_recv_timestamp(*msg));
      if (bag_trim_active && (recv_ns < bag_trim_lo || recv_ns > bag_trim_hi)) continue;
      auto it = topic_type_map.find(msg->topic_name);
      const std::string topic_type = (it != topic_type_map.end()) ? it->second : "";

      Event e;
      e.t_ns = recv_ns;
      e.source = Event::Source::BAG_MSG;
      e.msg = msg;
      e.topic_type = topic_type;
      e.order = order_counter++;
      heap.push(std::move(e));
      return true;
    }
    bag_eof = true;
    return false;
  };

  // Prime: one event from each source.
  // Both sources produce events in non-decreasing t_ns (the pcap reader's
  // internal 200ms reorder heap handles cross-lidar finalization order;
  // the bag reader yields messages in stored order). With one in-flight
  // event from each, popping the heap top yields strict time-merged
  // dispatch — no extra lookahead needed at this layer.
  pull_pcap();
  pull_bag();

  // Dispatch counters (per-second window). Declared above the lambdas that
  // capture them.
  uint64_t cnt_pcap_primary = 0, cnt_pcap_aux = 0, cnt_imu = 0, cnt_image = 0, cnt_ext = 0, cnt_dropped_primary = 0;
  uint64_t window_t0_ns = 0;

  auto bag_dispatch_fanout = [&](const Event& e) {
    const std::string& topic_name = e.msg->topic_name;
    const std::string& topic_type = e.topic_type;
    const rclcpp::SerializedMessage serialized_msg(*e.msg->serialized_data);

    // Hard guard: pcap is the only source of truth for primary points.
    if (topic_name == primary_points_topic) {
      if (!primary_bag_warned) {
        primary_bag_warned = true;
        spdlog::warn("dropping bag primary points '{}'; pcap is authoritative (one-shot)", topic_name);
      }
      cnt_dropped_primary++;
      return;
    }

    // Type-based handler.
    bool is_aux = false;
    if (concat_enabled) {
      for (auto& aux : aux_sensors) {
        if (topic_name == aux.topic) {
          if (topic_type == "sensor_msgs/msg/PointCloud2") {
            auto aux_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pc2_ser.deserialize_message(&serialized_msg, aux_msg.get());
            aux.buffer.push_back(aux_msg);
            while (aux.buffer.size() > aux.buffer_size) aux.buffer.pop_front();
          } else {
            spdlog::error("topic_type mismatch on aux topic {}: {} (expected PointCloud2)", topic_name, topic_type);
          }
          is_aux = true;
          break;
        }
      }
    }
    if (!is_aux) {
      if (topic_name == imu_topic && topic_type == "sensor_msgs/msg/Imu") {
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_ser.deserialize_message(&serialized_msg, imu_msg.get());
        glim->imu_callback(imu_msg);
        cnt_imu++;
      }
#ifdef BUILD_WITH_CV_BRIDGE
      else if (topic_name == image_topic) {
        if (topic_type == "sensor_msgs/msg/Image") {
          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          image_ser.deserialize_message(&serialized_msg, image_msg.get());
          glim->image_callback(image_msg);
        } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
          auto cm = std::make_shared<sensor_msgs::msg::CompressedImage>();
          compressed_image_ser.deserialize_message(&serialized_msg, cm.get());
          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          cv_bridge::toCvCopy(*cm, "bgr8")->toImageMsg(*image_msg);
          glim->image_callback(image_msg);
        }
      }
#endif
    }

    // Additive extension subscriptions.
    auto found = subscription_map.find(topic_name);
    if (found != subscription_map.end()) {
      for (const auto& sub : found->second) {
        sub->insert_message_instance(serialized_msg, topic_type);
      }
      cnt_ext++;
    }
  };

  auto maybe_log_counters = [&](uint64_t t_ns) {
    if (window_t0_ns == 0) { window_t0_ns = t_ns; return; }
    if (t_ns - window_t0_ns >= 1'000'000'000ULL) {
      spdlog::info("dispatch[Hz]: pcap_primary={} pcap_aux={} imu={} image={} ext={} bag_primary_dropped={}",
                   cnt_pcap_primary, cnt_pcap_aux, cnt_imu, cnt_image, cnt_ext, cnt_dropped_primary);
      cnt_pcap_primary = cnt_pcap_aux = cnt_imu = cnt_image = cnt_ext = cnt_dropped_primary = 0;
      window_t0_ns = t_ns;
    }
  };

  // Do NOT call rclcpp::spin_some(glim) in the main loop. GlimROS subscribes
  // to imu_topic / points_topic / image_topic in its constructor, and if a
  // live publisher exists on the network (e.g. the same dataset replaying or
  // the actual vehicle stack) those messages would pre-empt the bag-sourced
  // samples and corrupt TimeKeeper state with future timestamps.

  bool stop = false;
  while (!stop && rclcpp::ok()) {
    if (heap.empty()) break;
    Event ev = heap.top();
    heap.pop();
    // Replenish from the source the popped event came from so the heap
    // always has the next candidate from each non-EOF source.
    if (ev.source == Event::Source::PCAP_SCAN) pull_pcap();
    else pull_bag();

    if (!merged_t0_set) {
      merged_t0 = ev.t_ns;
      merged_t0_set = true;
    }

    // Unified-clock gates: start_offset, playback_until, dispatch_t0+playback_duration.
    if (start_offset > 0.0) {
      const double elapsed = (static_cast<double>(ev.t_ns) - static_cast<double>(merged_t0)) / 1e9;
      if (elapsed < start_offset) continue;
    }
    if (!dispatch_t0_set) {
      dispatch_t0_set = true;
      dispatch_t0_ns = ev.t_ns;
      spdlog::info("dispatch_t0 set at merged-t={:.3f} (after start_offset={})",
                   static_cast<double>(ev.t_ns) / 1e9, start_offset);
    }

    if (clock_pub) {
      rosgraph_msgs::msg::Clock clock_msg;
      clock_msg.clock.sec = static_cast<int32_t>(ev.t_ns / 1'000'000'000ULL);
      clock_msg.clock.nanosec = static_cast<uint32_t>(ev.t_ns % 1'000'000'000ULL);
      clock_pub->publish(clock_msg);
    }
    if (playback_until > 0.0 && ev.t_ns / 1e9 > playback_until) {
      spdlog::info("reached playback_until ({:.3f} > {:.3f})", ev.t_ns / 1e9, playback_until);
      stop = true;
      break;
    }
    if (playback_duration > 0.0) {
      const double dt = (static_cast<double>(ev.t_ns) - static_cast<double>(dispatch_t0_ns)) / 1e9;
      if (dt > playback_duration) {
        spdlog::info("reached playback_duration ({:.3f} > {:.3f})", dt, playback_duration);
        stop = true;
        break;
      }
    }

    // Dispatch.
    if (ev.source == Event::Source::PCAP_SCAN) {
      AssembledScan& s = ev.scan;
      if (s.topic == primary_points_topic) {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr final_points = s.cloud;
        if (concat_enabled && !aux_sensors.empty()) {
          final_points = glim_ros::merge_clouds(s.cloud, aux_sensors, concat_time_threshold);
        }
        const size_t workload = glim->points_callback(final_points);
        cnt_pcap_primary++;
        if (s.cloud->header.stamp.sec + s.cloud->header.stamp.nanosec * 1e-9 > end_time) {
          spdlog::info("end_time reached");
          stop = true;
          break;
        }
        if (workload > 5) {
          const size_t sleep_ms = (workload - 4) * 5;
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
      } else if (aux_topic_set.count(s.topic)) {
        for (auto& aux : aux_sensors) {
          if (aux.topic == s.topic) {
            aux.buffer.push_back(s.cloud);
            while (aux.buffer.size() > aux.buffer_size) aux.buffer.pop_front();
            break;
          }
        }
        cnt_pcap_aux++;
      }
    } else {
      bag_dispatch_fanout(ev);
    }

    maybe_log_counters(ev.t_ns);
    glim->timer_callback();

    const auto t0 = std::chrono::high_resolution_clock::now();
    while (glim->needs_wait()) {
      // Do NOT spin_some — see comment at top of main loop. We accept the
      // risk that extension modules requiring spin_some won't get pumped here.
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (std::chrono::high_resolution_clock::now() - t0 > std::chrono::seconds(1)) {
        spdlog::warn("throttling timeout (an extension module may be hanged)");
        break;
      }
    }
  }

  // Drain any remaining heap items (no more pulls).
  while (!stop && rclcpp::ok() && !heap.empty()) {
    Event ev = heap.top();
    heap.pop();
    if (ev.source == Event::Source::PCAP_SCAN) {
      AssembledScan& s = ev.scan;
      if (s.topic == primary_points_topic) {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr final_points = s.cloud;
        if (concat_enabled && !aux_sensors.empty()) {
          final_points = glim_ros::merge_clouds(s.cloud, aux_sensors, concat_time_threshold);
        }
        glim->points_callback(final_points);
      }
    } else {
      bag_dispatch_fanout(ev);
    }
    glim->timer_callback();
  }

  if (!auto_quit) {
    rclcpp::spin(glim);
  }

  glim->wait(auto_quit);
  glim->save(dump_path);
  return 0;
}
