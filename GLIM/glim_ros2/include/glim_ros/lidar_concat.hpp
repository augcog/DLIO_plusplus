#pragma once

#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <glim/util/config.hpp>
#include <glim/util/urdf_transforms.hpp>

namespace glim_ros {

struct AuxLidarSensor {
  std::string topic;
  Eigen::Isometry3d T_primary_sensor;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> buffer;
  size_t buffer_size;
};

inline double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}

inline bool find_xyz_offsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off, int& y_off, int& z_off) {
  x_off = y_off = z_off = -1;
  for (const auto& f : msg.fields) {
    if (f.name == "x") x_off = f.offset;
    else if (f.name == "y") y_off = f.offset;
    else if (f.name == "z") z_off = f.offset;
  }
  return x_off >= 0 && y_off >= 0 && z_off >= 0;
}

// Full point-field schema comparison: decides whether an auxiliary cloud can be
// byte-appended onto the primary. merge_clouds() transforms aux points using the
// AUX cloud's own field offsets, but the MERGED cloud keeps the PRIMARY's
// `fields`, so every downstream reader interprets the appended aux bytes with the
// primary layout. Appending is only safe when the aux layout is byte-identical to
// the primary: same point_step, same endianness, and the same ordered set of
// field {name, offset, datatype, count}. A same-point_step cloud with different
// offsets/datatypes would otherwise be silently misread. O(#fields) (~10-20 per
// Luminar scan) -- negligible next to deskew / voxelisation / registration.
// Returns true on match; on mismatch returns false and sets `reason` for logging.
inline bool schema_matches_primary(const sensor_msgs::msg::PointCloud2& aux,
                                   const sensor_msgs::msg::PointCloud2& primary,
                                   std::string& reason) {
  if (aux.point_step != primary.point_step) {
    reason = "point_step " + std::to_string(aux.point_step) + " vs primary " + std::to_string(primary.point_step);
    return false;
  }
  if (aux.is_bigendian != primary.is_bigendian) {
    reason = "endianness differs (aux is_bigendian=" + std::to_string(aux.is_bigendian) + ")";
    return false;
  }
  if (aux.fields.size() != primary.fields.size()) {
    reason = "field count " + std::to_string(aux.fields.size()) + " vs primary " + std::to_string(primary.fields.size());
    return false;
  }
  for (size_t i = 0; i < primary.fields.size(); i++) {
    const auto& a = aux.fields[i];
    const auto& p = primary.fields[i];
    if (a.name != p.name || a.offset != p.offset || a.datatype != p.datatype || a.count != p.count) {
      reason = "field[" + std::to_string(i) + "] '" + a.name + "' (offset=" + std::to_string(a.offset) +
               ",datatype=" + std::to_string(a.datatype) + ",count=" + std::to_string(a.count) +
               ") differs from primary '" + p.name + "' (offset=" + std::to_string(p.offset) +
               ",datatype=" + std::to_string(p.datatype) + ",count=" + std::to_string(p.count) + ")";
      return false;
    }
  }
  return true;
}

inline void transform_cloud_data(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int x_off,
  int y_off,
  int z_off,
  const Eigen::Isometry3d& T) {
  const Eigen::Matrix3f R = T.linear().cast<float>();
  const Eigen::Vector3f t = T.translation().cast<float>();
  const size_t num_points = data.size() / point_step;

  for (size_t i = 0; i < num_points; i++) {
    const size_t base = i * point_step;
    float x, y, z;
    std::memcpy(&x, &data[base + x_off], sizeof(float));
    std::memcpy(&y, &data[base + y_off], sizeof(float));
    std::memcpy(&z, &data[base + z_off], sizeof(float));

    Eigen::Vector3f p = R * Eigen::Vector3f(x, y, z) + t;
    std::memcpy(&data[base + x_off], &p.x(), sizeof(float));
    std::memcpy(&data[base + y_off], &p.y(), sizeof(float));
    std::memcpy(&data[base + z_off], &p.z(), sizeof(float));
  }
}

inline sensor_msgs::msg::PointCloud2::SharedPtr find_nearest(
  const std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>& buffer,
  double target_sec,
  double threshold) {
  sensor_msgs::msg::PointCloud2::SharedPtr best;
  double best_dt = std::numeric_limits<double>::max();
  for (const auto& msg : buffer) {
    double dt = std::abs(stamp_to_sec(msg->header.stamp) - target_sec);
    if (dt < best_dt) {
      best_dt = dt;
      best = msg;
    }
  }
  return (best && best_dt <= threshold) ? best : nullptr;
}

inline bool find_time_field(const sensor_msgs::msg::PointCloud2& msg, int& time_off, uint8_t& time_datatype, int& time_count) {
  time_off = -1;
  time_datatype = 0;
  time_count = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") {
      time_off = f.offset;
      time_datatype = f.datatype;
      time_count = f.count;
      return true;
    }
  }
  return false;
}

// Shift per-point timestamps by `dt` seconds to rebase an aux scan from its
// own header.stamp onto the merged cloud's primary header.stamp.
//
// SCAN-RELATIVE encodings (FLOAT32/FLOAT64 seconds-since-scan-start, UINT32
// nanoseconds-since-scan-start): add dt so the value reads as "offset since
// primary scan start" and deskew works.
//
// CAVEAT: the FLOAT64 branch ALWAYS adds dt, i.e. it assumes scan-relative
// seconds. This is correct for every aux LiDAR wired up today, but FLOAT64
// is also a valid carrier for ABSOLUTE epoch seconds (and GLIM's converter
// + TimeKeeper interpret large FLOAT64 values as absolute). A future aux
// sensor emitting FLOAT64 epoch seconds would therefore be double-shifted
// here, exactly like an unguarded UINT8[8] sensor would be. If such a
// sensor is added, gate the FLOAT64 shift the same way UINT8[8] is left
// untouched below (e.g. skip the shift when values look epoch-scaled).
//
// ABSOLUTE-EPOCH encodings (Luminar Iris UINT8[8] = uint64 PTP epoch ns):
// must NOT be shifted. Each point already carries its absolute capture
// time; the deskewer computes (t_i - merged_header.stamp) and naturally
// produces the correct (T_aux - T_primary + intra-aux-offset). Adding dt
// here would double-count the inter-scan offset.
//
// Luminar timestamp format (Luminar Iris Data Output Specification v1.3.0):
// the sensor does NOT emit a single uint64 epoch-ns field -- it carries
// 48-bit integer epoch SECONDS once per packet header (§2.1, UQ48.0) and a
// 32-bit SUB-SECOND NANOSECOND count per ray (§2.2/§2.6.3, UQ32.0) that
// wraps every 1 s; all fields little-endian (§2). The uint64 epoch-ns used
// here is the upstream ROS driver's reconstruction (seconds*1e9 + ns), so
// this depends on the driver, not the datasheet -- verify against the
// actual Luminar driver. (The "epoch time" guidance lives in the PTP
// sections of the Product Information Guide, not the data layout.)
inline void shift_cloud_timestamps(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int time_off,
  uint8_t time_datatype,
  int time_count,
  double dt) {
  if (time_off < 0) return;

  const size_t num_points = data.size() / point_step;
  for (size_t i = 0; i < num_points; i++) {
    uint8_t* time_ptr = &data[i * point_step + time_off];
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t val;
        std::memcpy(&val, time_ptr, sizeof(uint32_t));
        int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
        val = static_cast<uint32_t>(std::max<int64_t>(0, shifted));
        std::memcpy(time_ptr, &val, sizeof(uint32_t));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT32: {
        float val;
        std::memcpy(&val, time_ptr, sizeof(float));
        val += static_cast<float>(dt);
        std::memcpy(time_ptr, &val, sizeof(float));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double val;
        std::memcpy(&val, time_ptr, sizeof(double));
        val += dt;
        std::memcpy(time_ptr, &val, sizeof(double));
        break;
      }
      case sensor_msgs::msg::PointField::UINT8: {
        // UINT8 count=8 == Luminar Iris uint64 PTP epoch nanoseconds
        // (driver reconstruction of header seconds + per-ray nanoseconds;
        // see header comment for the format and citation). Absolute
        // timestamps -- leave untouched.
        // Any other count is not a recognised timestamp encoding.
        (void)dt;
        (void)time_count;
        break;
      }
      default:
        break;
    }
  }
}

// `primary` is taken as a ConstSharedPtr so both the offline tools (which hold
// a mutable SharedPtr) and the live GlimROS points_callback (which receives a
// ConstSharedPtr) can call this directly. The primary cloud is only read here;
// the merged output is a fresh copy.
inline sensor_msgs::msg::PointCloud2::ConstSharedPtr merge_clouds(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary,
  std::vector<AuxLidarSensor>& aux_sensors,
  double time_threshold) {
  const double t_primary = stamp_to_sec(primary->header.stamp);
  const uint32_t point_step = primary->point_step;

  int x_off, y_off, z_off;
  if (!find_xyz_offsets(*primary, x_off, y_off, z_off)) {
    spdlog::warn("lidar_concat: cannot find xyz fields in primary cloud");
    return primary;
  }

  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  size_t total_points = primary->width * primary->height;

  for (auto& aux : aux_sensors) {
    auto match = find_nearest(aux.buffer, t_primary, time_threshold);
    if (!match) {
      spdlog::debug("lidar_concat: no match for {} (t={:.3f})", aux.topic, t_primary);
      continue;
    }
    // Validate the FULL field schema, not just point_step: the merged cloud
    // keeps the primary's `fields`, so an aux scan with the same point_step but
    // different field offsets/datatypes would be silently misread downstream.
    std::string schema_reason;
    if (!schema_matches_primary(*match, *primary, schema_reason)) {
      spdlog::warn(
        "lidar_concat: skipping {} — PointCloud2 schema mismatch vs primary: {} "
        "(merged cloud uses the primary field layout; appending mismatched aux bytes "
        "would misread them — normalize the aux layout upstream to enable concatenation)",
        aux.topic, schema_reason);
      continue;
    }

    std::vector<uint8_t> data(match->data.begin(), match->data.end());
    int ax, ay, az;
    if (find_xyz_offsets(*match, ax, ay, az)) {
      transform_cloud_data(data, point_step, ax, ay, az, aux.T_primary_sensor);
    }

    int time_off;
    uint8_t time_datatype;
    int time_count;
    if (find_time_field(*match, time_off, time_datatype, time_count)) {
      if (time_datatype == sensor_msgs::msg::PointField::UINT8 && time_count == 8) {
        spdlog::debug("lidar_concat: keeping absolute UINT8[8] timestamps for {}", aux.topic);
      } else {
        double dt = stamp_to_sec(match->header.stamp) - t_primary;
        shift_cloud_timestamps(data, point_step, time_off, time_datatype, time_count, dt);
        spdlog::debug("lidar_concat: shifted timestamps for {} by {:.6f}s", aux.topic, dt);
      }
    }

    merged->data.insert(merged->data.end(), data.begin(), data.end());
    total_points += match->width * match->height;

    double dt = std::abs(stamp_to_sec(match->header.stamp) - t_primary);
    spdlog::debug("lidar_concat: merged {} (dt={:.4f}s, {} pts)", aux.topic, dt, match->width * match->height);
  }

  merged->width = total_points;
  merged->height = 1;
  merged->row_step = point_step * total_points;
  return merged;
}

struct AuxConcatConfig {
  bool enabled = false;
  double time_threshold = 0.05;
  int buffer_size = 200;
  std::vector<AuxLidarSensor> aux_sensors;
};

inline AuxConcatConfig load_aux_sensors_from_config(const glim::Config& config_sensors) {
  AuxConcatConfig out;
  out.enabled = config_sensors.param<bool>("lidar_concat", "enabled", false);
  out.time_threshold = config_sensors.param<double>("lidar_concat", "time_threshold", 0.05);
  out.buffer_size = config_sensors.param<int>("lidar_concat", "buffer_size", 200);

  if (!out.enabled) {
    return out;
  }

  const auto aux_topics = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_topics", {});

  const std::string urdf_path = config_sensors.param<std::string>("lidar_concat", "urdf_path", "");
  const std::string primary_frame = config_sensors.param<std::string>("lidar_concat", "primary_frame", "");
  std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> urdf_transforms;
  bool use_urdf = !urdf_path.empty() && !primary_frame.empty();

  if (use_urdf) {
    try {
      urdf_transforms = glim::parse_urdf_transforms(urdf_path);
      spdlog::info("lidar_concat: loaded URDF from {} (primary_frame={})", urdf_path, primary_frame);
    } catch (const std::exception& e) {
      spdlog::error("lidar_concat: failed to parse URDF: {}", e.what());
      use_urdf = false;
    }
  }

  const auto aux_frames = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_frames", {});
  if (use_urdf && aux_frames.size() != aux_topics.size()) {
    spdlog::error("lidar_concat: aux_frames size ({}) must match aux_topics size ({})", aux_frames.size(), aux_topics.size());
    use_urdf = false;
  }

  for (size_t i = 0; i < aux_topics.size(); i++) {
    const auto& topic = aux_topics[i];
    AuxLidarSensor sensor;
    sensor.topic = topic;
    sensor.buffer_size = out.buffer_size;

    if (use_urdf) {
      const std::string& aux_frame = aux_frames[i];
      try {
        sensor.T_primary_sensor = glim::compute_transform(urdf_transforms, primary_frame, aux_frame);
        std::stringstream ss;
        ss << sensor.T_primary_sensor.matrix();
        spdlog::info("lidar_concat: T_{}_{}:\n{}", primary_frame, aux_frame, ss.str());
      } catch (const std::exception& e) {
        spdlog::error("lidar_concat: failed to compute transform {} -> {}: {}", primary_frame, aux_frame, e.what());
        continue;
      }
    } else {
      std::string key = topic;
      for (auto& c : key) {
        if (c == '/') c = '_';
      }
      if (!key.empty() && key[0] == '_') key = key.substr(1);
      key = "T_primary_" + key;

      auto flat = config_sensors.param<std::vector<double>>("lidar_concat", key);
      if (!flat || flat->size() != 16) {
        spdlog::error("lidar_concat: missing or invalid transform '{}' for topic '{}'", key, topic);
        continue;
      }

      Eigen::Matrix4d mat;
      for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
          mat(r, c) = (*flat)[r * 4 + c];
      sensor.T_primary_sensor = Eigen::Isometry3d(mat);
    }

    spdlog::info("lidar_concat: auxiliary sensor {} enabled", sensor.topic);
    out.aux_sensors.push_back(std::move(sensor));
  }
  spdlog::info("lidar_concat: {} auxiliary sensors, threshold={:.3f}s", out.aux_sensors.size(), out.time_threshold);
  return out;
}

}  // namespace glim_ros
