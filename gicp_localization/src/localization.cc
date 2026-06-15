/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "gicp_localization/localization.h"
#include "dlio/utils.h"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

namespace {

constexpr double kRadToDeg = 57.29577951308232;

bool matrixFinite(const Eigen::Matrix4f& pose) {
  return pose.array().isFinite().all();
}

geometry_msgs::msg::Pose poseMsgFromMatrix(const Eigen::Matrix4f& pose) {
  geometry_msgs::msg::Pose msg;
  const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  q.normalize();

  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

geometry_msgs::msg::PoseStamped poseStampedFromMatrix(const Eigen::Matrix4f& pose,
                                                      const rclcpp::Time& stamp,
                                                      const std::string& frame_id) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose = poseMsgFromMatrix(pose);
  return msg;
}

double rotationDistanceDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  Eigen::Quaternionf qa(a.block<3, 3>(0, 0));
  Eigen::Quaternionf qb(b.block<3, 3>(0, 0));
  qa.normalize();
  qb.normalize();

  Eigen::Quaternionf dq = qa.conjugate() * qb;
  dq.normalize();
  const double w = std::clamp(std::abs(static_cast<double>(dq.w())), 0.0, 1.0);
  return 2.0 * std::acos(w) * kRadToDeg;
}

double deltaTranslationNorm(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to) {
  return (to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3)).norm();
}

std::string poseSummary(const Eigen::Matrix4f& pose) {
  if (!matrixFinite(pose)) {
    return "invalid";
  }

  const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  const Eigen::Vector3f rpy_deg = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2) * static_cast<float>(kRadToDeg);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "xyz=[" << t.x() << "," << t.y() << "," << t.z() << "]"
      << " rpy_deg=[" << rpy_deg.x() << "," << rpy_deg.y() << "," << rpy_deg.z() << "]";
  return oss.str();
}

std::string scalarSummary(double value, int precision = 3) {
  if (!std::isfinite(value)) {
    return "nan";
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

double hessianConditionProxy(const Eigen::Matrix<double, 6, 6>& hessian) {
  if (!hessian.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Matrix<double, 6, 6> sym_hessian = 0.5 * (hessian + hessian.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(sym_hessian);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }

  const auto abs_eigenvalues = solver.eigenvalues().cwiseAbs();
  const double max_eigenvalue = abs_eigenvalues.maxCoeff();

  double min_nonzero_eigenvalue = std::numeric_limits<double>::infinity();
  for (int i = 0; i < abs_eigenvalues.size(); ++i) {
    const double value = abs_eigenvalues[i];
    if (value > 1e-12 && value < min_nonzero_eigenvalue) {
      min_nonzero_eigenvalue = value;
    }
  }

  if (!std::isfinite(max_eigenvalue) || !std::isfinite(min_nonzero_eigenvalue)) {
    return std::numeric_limits<double>::infinity();
  }

  return max_eigenvalue / min_nonzero_eigenvalue;
}

// Find x/y/z field offsets in a PointCloud2 message. Returns false if any are missing.
bool findXYZOffsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off, int& y_off, int& z_off) {
  x_off = y_off = z_off = -1;
  for (const auto& f : msg.fields) {
    if (f.name == "x") x_off = static_cast<int>(f.offset);
    else if (f.name == "y") y_off = static_cast<int>(f.offset);
    else if (f.name == "z") z_off = static_cast<int>(f.offset);
  }
  return x_off >= 0 && y_off >= 0 && z_off >= 0;
}

// Apply rigid transform to xyz of `num_points` points starting at `data` (in place).
// Pointer-based variant — operates on a span within a larger buffer so callers
// can write aux scans directly into the merged cloud's data without an
// intermediate copy.
void transformCloudData(uint8_t* data, size_t num_points, uint32_t point_step,
                        int x_off, int y_off, int z_off,
                        const Eigen::Matrix4f& T) {
  const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  const Eigen::Vector3f t = T.block<3, 1>(0, 3);
  for (size_t i = 0; i < num_points; ++i) {
    uint8_t* base = data + i * point_step;
    float x, y, z;
    std::memcpy(&x, base + x_off, sizeof(float));
    std::memcpy(&y, base + y_off, sizeof(float));
    std::memcpy(&z, base + z_off, sizeof(float));
    Eigen::Vector3f p = R * Eigen::Vector3f(x, y, z) + t;
    std::memcpy(base + x_off, &p.x(), sizeof(float));
    std::memcpy(base + y_off, &p.y(), sizeof(float));
    std::memcpy(base + z_off, &p.z(), sizeof(float));
  }
}

const char* pointFieldDatatypeName(uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:    return "INT8";
    case sensor_msgs::msg::PointField::UINT8:   return "UINT8";
    case sensor_msgs::msg::PointField::INT16:   return "INT16";
    case sensor_msgs::msg::PointField::UINT16:  return "UINT16";
    case sensor_msgs::msg::PointField::INT32:   return "INT32";
    case sensor_msgs::msg::PointField::UINT32:  return "UINT32";
    case sensor_msgs::msg::PointField::FLOAT32: return "FLOAT32";
    case sensor_msgs::msg::PointField::FLOAT64: return "FLOAT64";
    default:                                    return "UNKNOWN";
  }
}

// One-shot diagnostic: print everything we can extract about the incoming
// PointCloud2's timestamp field so a developer can decide which bit-level
// interpretation the live driver actually uses. See
// docs/luminar_timestamp_diagnostic_guide.pdf for how to read this output.
//
// Fires only on the first cloud (guarded by std::call_once at the caller),
// always emits the lines regardless of localization/verbose so a single
// test run produces the answer.
//
// Output format (per line):
//   [LUMINAR_TS_DIAG] <key>: <value>
// The block is bracketed by [LUMINAR_TS_DIAG] BEGIN / END markers so it's
// easy to grep out of a noisy log.
void logTimestampDiagnostic(const sensor_msgs::msg::PointCloud2& pc,
                            int ts_off, uint8_t ts_datatype, int ts_count,
                            const char* sensor_name) {
  std::fprintf(stderr, "[LUMINAR_TS_DIAG] BEGIN\n");
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] sensor_type=%s  point_step=%u  "
               "num_points=%u  width=%u  height=%u  is_bigendian=%d  "
               "header.stamp=%u.%09u  frame_id='%s'\n",
               sensor_name, pc.point_step,
               static_cast<unsigned>(pc.data.size() / std::max<uint32_t>(pc.point_step, 1u)),
               pc.width, pc.height, pc.is_bigendian ? 1 : 0,
               pc.header.stamp.sec, pc.header.stamp.nanosec,
               pc.header.frame_id.c_str());
  std::fprintf(stderr, "[LUMINAR_TS_DIAG] fields (offset, datatype, count, name):\n");
  for (const auto& f : pc.fields) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   off=%-4u  type=%-7s  count=%-3u  name='%s'\n",
                 f.offset, pointFieldDatatypeName(f.datatype), f.count,
                 f.name.c_str());
  }
  if (ts_off < 0) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG] no timestamp field detected (no field named "
                 "t/time/time_stamp/timestamp). Deskew cannot use per-point times.\n");
    std::fprintf(stderr, "[LUMINAR_TS_DIAG] END\n");
    std::fflush(stderr);
    return;
  }
  // Implied byte size per datatype, times count.
  // (count is normally 1 except for UINT8 where count carries the length of
  // the byte run, e.g. UINT8 count=8 = 8 raw bytes.)
  int bytes_per_elem = 1;
  switch (ts_datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:    bytes_per_elem = 1; break;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:   bytes_per_elem = 2; break;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:  bytes_per_elem = 4; break;
    case sensor_msgs::msg::PointField::FLOAT64:  bytes_per_elem = 8; break;
    default:                                      bytes_per_elem = 0; break;
  }
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] timestamp_field: off=%d  type=%s  count=%d  "
               "implied_byte_size=%d\n",
               ts_off, pointFieldDatatypeName(ts_datatype), ts_count,
               bytes_per_elem * ts_count);

  // Walk up to 3 sample points (first, midpoint, last) and dump their 8
  // timestamp bytes interpreted four different ways.  This lets the developer
  // pattern-match what the driver actually emits without instrumenting the
  // driver itself.
  const uint32_t step = pc.point_step;
  const size_t num_points = pc.data.size() / std::max<uint32_t>(step, 1u);
  if (num_points == 0 || ts_off + 8 > static_cast<int>(step)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG] (no samples to dump -- empty cloud or "
                 "field extends past point_step)\n");
    std::fprintf(stderr, "[LUMINAR_TS_DIAG] END\n");
    std::fflush(stderr);
    return;
  }
  const size_t sample_indices[3] = {
      0u, num_points / 2u,
      num_points > 0u ? num_points - 1u : 0u};
  const char* sample_labels[3] = {"point[0]    ", "point[mid]  ", "point[N-1]  "};

  uint64_t ts_uint64[3] = {0, 0, 0};
  double   ts_double[3] = {0.0, 0.0, 0.0};

  for (int s = 0; s < 3; ++s) {
    const size_t idx = sample_indices[s];
    const uint8_t* ptr = pc.data.data() + idx * step + ts_off;

    // Raw 8 bytes (little-endian dump as hex).
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG] %s idx=%zu  raw=%02x %02x %02x %02x "
                 "%02x %02x %02x %02x\n",
                 sample_labels[s], idx, ptr[0], ptr[1], ptr[2], ptr[3],
                 ptr[4], ptr[5], ptr[6], ptr[7]);

    // Interpretation A: bytes are a uint64 (e.g. PTP ns since epoch, or ns
    // since boot, or ns since scan start).
    uint64_t u64 = 0;
    std::memcpy(&u64, ptr, sizeof(uint64_t));
    ts_uint64[s] = u64;

    // Interpretation B: bytes are an IEEE-754 double encoded as seconds.
    double d_sec = 0.0;
    std::memcpy(&d_sec, ptr, sizeof(double));
    ts_double[s] = d_sec;

    // Interpretation C: bytes are an IEEE-754 double encoded as nanoseconds
    // (i.e. d_sec interpreted as ns directly).
    const double d_ns = d_sec;  // same memory, just rename for clarity.

    // Interpretation D: two uint32s (PTP layout: secs in low half, ns offset
    // in high half, or vice versa).
    uint32_t u32_lo = 0, u32_hi = 0;
    std::memcpy(&u32_lo, ptr, sizeof(uint32_t));
    std::memcpy(&u32_hi, ptr + 4, sizeof(uint32_t));

    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]              as_uint64=%-20lu  "
                 "as_double_sec=%.9f  as_double_ns=%.3e  "
                 "as_uint32_pair=(lo=%-10u hi=%-10u)\n",
                 static_cast<unsigned long>(u64), d_sec, d_ns,
                 u32_lo, u32_hi);
  }

  bool have_uint64_stats =
      ts_datatype == sensor_msgs::msg::PointField::UINT8 && ts_count == 8;
  uint64_t min_u64 = std::numeric_limits<uint64_t>::max();
  uint64_t max_u64 = 0;
  if (have_uint64_stats) {
    for (size_t i = 0; i < num_points; ++i) {
      uint64_t u64 = 0;
      const uint8_t* ptr = pc.data.data() + i * step + ts_off;
      std::memcpy(&u64, ptr, sizeof(uint64_t));
      min_u64 = std::min(min_u64, u64);
      max_u64 = std::max(max_u64, u64);
    }
    const uint64_t span_u64 = max_u64 - min_u64;
    const int64_t header_ns =
        static_cast<int64_t>(pc.header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(pc.header.stamp.nanosec);
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG] aggregate uint64_ns: min=%lu  max=%lu  "
                 "span=%lu ns ~= %.3f ms  header_minus_min=%ld ns\n",
                 static_cast<unsigned long>(min_u64),
                 static_cast<unsigned long>(max_u64),
                 static_cast<unsigned long>(span_u64),
                 static_cast<double>(span_u64) * 1e-6,
                 static_cast<long>(header_ns - static_cast<int64_t>(min_u64)));
  }

  // Deltas between adjacent samples in each interpretation, to make collapse
  // obvious at a glance.
  const int64_t d_u64_01 =
      static_cast<int64_t>(ts_uint64[1]) - static_cast<int64_t>(ts_uint64[0]);
  const int64_t d_u64_0N =
      static_cast<int64_t>(ts_uint64[2]) - static_cast<int64_t>(ts_uint64[0]);
  const double  d_dbl_01 = ts_double[1] - ts_double[0];
  const double  d_dbl_0N = ts_double[2] - ts_double[0];

  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] deltas (mid - first / last - first):\n");
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG]   as_uint64_ns:    mid-first=%-15ld  "
               "last-first=%-15ld\n",
               static_cast<long>(d_u64_01), static_cast<long>(d_u64_0N));
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG]   as_double_sec:   mid-first=%.9f  "
               "last-first=%.9f\n",
               d_dbl_01, d_dbl_0N);

  // Heuristic verdict: order-of-magnitude check on each interpretation,
  // with the assumption that a healthy 10 Hz LiDAR scan should span ~0.1 s.
  // This is just a hint; the developer reads the raw lines above to confirm.
  auto plausible_seconds = [](double x) {
    return x > 1e-4 && x < 1.0;  // within 0.1 ms to 1 s
  };
  auto plausible_ns_as_uint64 = [](int64_t x) {
    return x > 100000 && x < 1000000000;  // 0.1 ms to 1 s, in ns
  };
  auto plausible_ns_span_u64 = [](uint64_t x) {
    return x > 100000ULL && x < 1000000000ULL;  // 0.1 ms to 1 s, in ns
  };
  auto plausible_epoch_ns = [](uint64_t x) {
    return x > 1600000000000000000ULL && x < 2100000000000000000ULL;
  };
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] verdict (heuristic; check raw lines to confirm):\n");
  if (have_uint64_stats && plausible_ns_span_u64(max_u64 - min_u64) &&
      plausible_epoch_ns(min_u64) && plausible_epoch_ns(max_u64)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   uint64 epoch-ns interpretation looks plausible "
                 "(aggregate span %lu ns ~= %.3f ms)\n",
                 static_cast<unsigned long>(max_u64 - min_u64),
                 static_cast<double>(max_u64 - min_u64) * 1e-6);
  } else if (have_uint64_stats && plausible_ns_span_u64(max_u64 - min_u64) &&
             (!plausible_epoch_ns(min_u64) || !plausible_epoch_ns(max_u64))) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   uint64 ns span is plausible, but magnitude "
                 "is not Unix epoch ns (min=%lu). Relative deskew may work, "
                 "but this FAILS the full epoch-ns validation contract.\n",
                 static_cast<unsigned long>(min_u64));
  } else if (plausible_ns_as_uint64(d_u64_0N) && plausible_epoch_ns(ts_uint64[0]) &&
      !plausible_seconds(d_dbl_0N)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   uint64 epoch-ns interpretation looks plausible "
                 "(sample span %ld ns ~= %.3f ms)\n",
                 static_cast<long>(d_u64_0N), d_u64_0N * 1e-6);
  } else if (plausible_ns_as_uint64(d_u64_0N) && !plausible_epoch_ns(ts_uint64[0])) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   uint64 ns span is plausible, but magnitude "
                 "is not Unix epoch ns (first=%lu). Relative deskew may work, "
                 "but this FAILS the full epoch-ns validation contract.\n",
                 static_cast<unsigned long>(ts_uint64[0]));
  } else if (plausible_seconds(d_dbl_0N) && !plausible_ns_as_uint64(d_u64_0N)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   FLOAT64 seconds interpretation looks "
                 "plausible (span %.6f s ~= %.3f ms). Current code memcpys as "
                 "uint64, which scrambles this.  Read as double instead.\n",
                 d_dbl_0N, d_dbl_0N * 1000.0);
  } else if (d_u64_0N == 0 && d_dbl_0N == 0.0) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   timestamps appear COLLAPSED (zero span "
                 "in both interpretations). Driver likely fills every point "
                 "with the same scan-level stamp.\n");
  } else {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   verdict unclear -- see raw lines above. "
                 "Possible: per-scan timestamps with random jitter, or a "
                 "format we don't recognise.\n");
  }
  std::fprintf(stderr, "[LUMINAR_TS_DIAG] END\n");
  std::fflush(stderr);
}

bool findTimeField(const sensor_msgs::msg::PointCloud2& msg, int& time_off,
                   uint8_t& time_datatype, int& time_count) {
  time_off = -1;
  time_datatype = 0;
  time_count = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") {
      time_off = static_cast<int>(f.offset);
      time_datatype = f.datatype;
      time_count = static_cast<int>(f.count);
      return true;
    }
  }
  return false;
}

bool luminarCloudTimeMidpointSec(const sensor_msgs::msg::PointCloud2& msg, double& midpoint_sec) {
  int time_off;
  uint8_t time_datatype;
  int time_count;
  if (!findTimeField(msg, time_off, time_datatype, time_count) ||
      time_datatype != sensor_msgs::msg::PointField::UINT8 ||
      time_count != 8 ||
      time_off + 8 > static_cast<int>(msg.point_step)) {
    return false;
  }

  const size_t num_points = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (num_points == 0) {
    return false;
  }

  uint64_t min_ts = std::numeric_limits<uint64_t>::max();
  uint64_t max_ts = 0;
  const uint32_t step = msg.point_step;
  for (size_t i = 0; i < num_points; ++i) {
    uint64_t ts = 0;
    std::memcpy(&ts, msg.data.data() + i * step + time_off, sizeof(uint64_t));
    min_ts = std::min(min_ts, ts);
    max_ts = std::max(max_ts, ts);
  }

  midpoint_sec = (static_cast<double>(min_ts) + static_cast<double>(max_ts)) * 0.5e-9;
  return true;
}

// Luminar stores hardware-clock ns in the timestamp union slot (8 raw bytes, not IEEE double).
inline uint64_t luminarPointTimestampNs(const PointType& pt) {
  uint64_t ts = 0;
  std::memcpy(&ts, &pt.timestamp, sizeof(uint64_t));
  return ts;
}

inline void clearPointTimeUnion(PointType& pt) {
  const uint64_t zero = 0;
  std::memcpy(&pt.timestamp, &zero, sizeof(uint64_t));
}

// Copy per-point time from PointCloud2 into the dlio::Point union for the configured sensor.
void copyPointTimeFromCloud(const uint8_t* src, int time_off, uint8_t time_datatype, int time_count,
                           dlio::SensorType sensor, PointType& dst) {
  if (time_off < 0) {
    return;
  }
  const uint8_t* tp = src + time_off;

  switch (sensor) {
    case dlio::SensorType::LUMINAR: {
      uint64_t ts_raw = 0;
      if (time_datatype == sensor_msgs::msg::PointField::FLOAT64 ||
          (time_datatype == sensor_msgs::msg::PointField::UINT8 && time_count == 8)) {
        std::memcpy(&ts_raw, tp, sizeof(uint64_t));
      } else if (time_datatype == sensor_msgs::msg::PointField::UINT32) {
        uint32_t ts_ns32 = 0;
        std::memcpy(&ts_ns32, tp, sizeof(uint32_t));
        ts_raw = static_cast<uint64_t>(ts_ns32);
      } else {
        return;
      }
      std::memcpy(&dst.timestamp, &ts_raw, sizeof(uint64_t));
      return;
    }
    case dlio::SensorType::OUSTER: {
      uint32_t t_ns = 0;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::UINT32:
          std::memcpy(&t_ns, tp, sizeof(uint32_t));
          break;
        case sensor_msgs::msg::PointField::FLOAT32: {
          float t_s = 0.f;
          std::memcpy(&t_s, tp, sizeof(float));
          t_ns = static_cast<uint32_t>(t_s * 1e9f);
          break;
        }
        case sensor_msgs::msg::PointField::FLOAT64: {
          double t_s = 0.;
          std::memcpy(&t_s, tp, sizeof(double));
          t_ns = static_cast<uint32_t>(t_s * 1e9);
          break;
        }
        default:
          break;
      }
      dst.t = t_ns;
      return;
    }
    case dlio::SensorType::VELODYNE: {
      float t_s = 0.f;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::FLOAT32:
          std::memcpy(&t_s, tp, sizeof(float));
          break;
        case sensor_msgs::msg::PointField::UINT32: {
          uint32_t t_ns = 0;
          std::memcpy(&t_ns, tp, sizeof(uint32_t));
          t_s = static_cast<float>(t_ns * 1e-9);
          break;
        }
        default:
          break;
      }
      dst.time = t_s;
      return;
    }
    case dlio::SensorType::HESAI: {
      double t_s = 0.;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::FLOAT64:
          std::memcpy(&t_s, tp, sizeof(double));
          break;
        case sensor_msgs::msg::PointField::FLOAT32: {
          float t_f = 0.f;
          std::memcpy(&t_f, tp, sizeof(float));
          t_s = static_cast<double>(t_f);
          break;
        }
        default:
          break;
      }
      dst.timestamp = t_s;
      return;
    }
    case dlio::SensorType::LIVOX: {
      if (time_datatype == sensor_msgs::msg::PointField::UINT8 && time_count == 8) {
        uint64_t t_ns = 0;
        std::memcpy(&t_ns, tp, sizeof(uint64_t));
        dst.timestamp = static_cast<double>(t_ns);
      } else if (time_datatype == sensor_msgs::msg::PointField::UINT32) {
        uint32_t t_ns = 0;
        std::memcpy(&t_ns, tp, sizeof(uint32_t));
        dst.timestamp = static_cast<double>(t_ns);
      } else if (time_datatype == sensor_msgs::msg::PointField::FLOAT64) {
        std::memcpy(&dst.timestamp, tp, sizeof(double));
      } else if (time_datatype == sensor_msgs::msg::PointField::FLOAT32) {
        float t_f = 0.f;
        std::memcpy(&t_f, tp, sizeof(float));
        dst.timestamp = static_cast<double>(t_f);
      }
      return;
    }
    default:
      return;
  }
}

void logLuminarTimestampStats(size_t num_points, const pcl::PointCloud<PointType>& cloud,
                              size_t unique_ros_times) {
  if (cloud.points.empty()) {
    return;
  }
  uint64_t tmin = std::numeric_limits<uint64_t>::max();
  uint64_t tmax = 0;
  for (const auto& pt : cloud.points) {
    const uint64_t ts = luminarPointTimestampNs(pt);
    tmin = std::min(tmin, ts);
    tmax = std::max(tmax, ts);
  }
  const size_t mid = cloud.points.size() / 2;
  const uint64_t ts0 = luminarPointTimestampNs(cloud.points.front());
  const uint64_t ts_mid = luminarPointTimestampNs(cloud.points[mid]);
  const uint64_t tsN = luminarPointTimestampNs(cloud.points.back());
  std::fprintf(stderr,
               "[LUMINAR_DBG] %zu pts, %zu unique_ros_times, ts0=%lu tsMid=%lu tsN=%lu "
               "span_ns=%ld (minmax_span=%ld)\n",
               num_points, unique_ros_times, static_cast<unsigned long>(ts0),
               static_cast<unsigned long>(ts_mid), static_cast<unsigned long>(tsN),
               static_cast<long>(static_cast<int64_t>(tsN) - static_cast<int64_t>(ts0)),
               static_cast<long>(static_cast<int64_t>(tmax) - static_cast<int64_t>(tmin)));
  std::fflush(stderr);
}

// Shift per-point timestamps by `dt` seconds. Relative-time LiDAR formats need
// this when aux scans are appended to a primary cloud. Luminar UINT8[8] fields
// are absolute epoch nanoseconds and are intentionally not shifted by callers.
void shiftCloudTimestamps(uint8_t* data, size_t num_points, uint32_t point_step,
                          int time_off, uint8_t time_datatype, int time_count,
                          double dt, bool luminar_uint64) {
  if (time_off < 0) return;

  if (luminar_uint64) {
    return;
  }

  for (size_t i = 0; i < num_points; ++i) {
    uint8_t* time_ptr = data + i * point_step + time_off;
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t val;
        std::memcpy(&val, time_ptr, sizeof(uint32_t));
        const int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
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
        if (time_count == 8) {
          uint64_t val;
          std::memcpy(&val, time_ptr, sizeof(uint64_t));
          const int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
          val = static_cast<uint64_t>(std::max<int64_t>(0, shifted));
          std::memcpy(time_ptr, &val, sizeof(uint64_t));
        }
        break;
      }
      default:
        break;
    }
  }
}

visualization_msgs::msg::Marker makeArrowMarker(const Eigen::Matrix4f& pose,
                                                const std::string& frame_id,
                                                const rclcpp::Time& stamp,
                                                int id,
                                                const std::string& ns,
                                                float r,
                                                float g,
                                                float b) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = poseMsgFromMatrix(pose);
  marker.scale.x = 2.0;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.a = 1.0f;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  return marker;
}

}  // namespace

gicp_localization::LocalizationNode::LocalizationNode() : Node("gicp_localization_node") {

  // Default before parameter loading; localization/sensor_type may override it.
  this->sensor = dlio::SensorType::OUSTER;
  this->getParams();

  // Initialize flags
  this->initialized = false;
  this->first_imu_received = false;

  // Initialize pose
  this->current_pose = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->last_gicp_pose_ = Eigen::Matrix4f::Identity();
  this->last_gicp_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->last_gicp_valid_ = false;

  // Initialize IMU buffer
  this->imu_buffer.set_capacity(this->imu_buffer_size_);

  // Initialize IMU calibration state
  this->imu_calibrated_ = false;
  this->imu_calib_start_stamp_ = -1.0;
  this->imu_calib_count_ = 0;
  this->imu_calib_gyro_sum_ = Eigen::Vector3f::Zero();
  this->imu_calib_accel_sum_ = Eigen::Vector3f::Zero();

  // Initialize RTK-driven calibration state
  this->init_phase_ = InitPhase::WAITING;
  this->first_imu_stamp_ = -1.0;
  this->rtk_calib_start_stamp_ = -1.0;
  this->rtk_calib_count_ = 0;
  this->rtk_gyro_bias_sum_ = Eigen::Vector3f::Zero();
  this->rtk_accel_bias_sum_ = Eigen::Vector3f::Zero();
  this->rtk_gyro_bias_sq_sum_ = Eigen::Vector3f::Zero();
  this->rtk_accel_bias_sq_sum_ = Eigen::Vector3f::Zero();
  this->has_prev_gt_for_accel_ = false;
  this->prev_gt_stamp_ = 0.0;
  this->prev_v_world_ = Eigen::Vector3f::Zero();
  this->has_latest_rtk_seed_ = false;

  // Initialize previous scan stamp
  this->prev_scan_stamp = 0.0;
  this->last_pointcloud_stamp_.store(-1.0);
  this->observer_dt_ = 0.0;
  this->last_scan_input_frame_.clear();
  this->last_raw_point_count_ = 0;
  this->last_preprocessed_point_count_ = 0;

  // Initialize lidar pose
  this->basePose.p = Eigen::Vector3f::Zero();
  this->basePose.q = Eigen::Quaternionf::Identity();

  // Initialize previous velocity
  this->prev_vel = Eigen::Vector3f::Zero();

  // Initialize geometric observer state
  this->state.p = Eigen::Vector3f::Zero();
  this->state.q = Eigen::Quaternionf::Identity();
  this->state.v.lin.b = Eigen::Vector3f::Zero();
  this->state.v.lin.w = Eigen::Vector3f::Zero();
  this->state.v.ang.b = Eigen::Vector3f::Zero();
  this->state.v.ang.w = Eigen::Vector3f::Zero();
  this->state.b.gyro = Eigen::Vector3f::Zero();
  this->state.b.accel = Eigen::Vector3f::Zero();

  this->geo.first_opt_done = false;
  this->geo.update_seq = 0;
  this->consecutive_failures_ = 0;
  this->gt_extrinsics_cached_ = false;
  this->T_base_gtbody_.setIdentity();
  this->geo.dp = 0.0;
  this->geo.dq_deg = 0.0;
  this->geo.prev_p = Eigen::Vector3f::Zero();
  this->geo.prev_q = Eigen::Quaternionf::Identity();
  this->geo.prev_vel = Eigen::Vector3f::Zero();

  // Initialize extrinsics to identity (should be configured from parameters)
  this->extrinsics.baselink2imu.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2imu.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2lidar.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2lidar.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
  this->extrinsics_cached_ = false;
  this->imu_extrinsics_cached_ = false;
  this->pending_initial_pose_ = false;

  // Initialize point clouds
  this->map_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  this->map_cloud_ds = std::make_shared<pcl::PointCloud<PointType>>();
  this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
  this->original_scan = std::make_shared<pcl::PointCloud<PointType>>();

  // Load map
  if (!this->loadMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map! Exiting...");
    throw std::runtime_error("Failed to load map");
  }

  // Setup GICP
  this->gicp.setNumThreads(omp_get_max_threads());
  this->gicp.setCorrespondenceRandomness(this->gicp_corr_randomness_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_epsilon_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_epsilon_);
  this->gicp.setDebugPrint(this->debug_lm_print_);

  // Set target (map)
  this->gicp.setInputTarget(this->map_cloud);
  if (!this->gicp.calculateTargetCovariances()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to calculate map covariances! GICP will not work correctly.");
    throw std::runtime_error("Failed to calculate target covariances");
  }

  // Setup subscribers
  this->pointcloud_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pointcloud_sub_opt = rclcpp::SubscriptionOptions();
  pointcloud_sub_opt.callback_group = this->pointcloud_cb_group;
  // Use sensor-data QoS so rosbag/sensor publishers with BEST_EFFORT are compatible.
  this->pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&gicp_localization::LocalizationNode::callbackPointCloud, this, std::placeholders::_1),
      pointcloud_sub_opt);

  this->initial_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = this->initial_pose_cb_group;
  this->initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&gicp_localization::LocalizationNode::callbackInitialPose, this, std::placeholders::_1),
      initial_pose_sub_opt);

  // Aux LiDAR subscribers (multi-LiDAR concatenation). Use a Reentrant group so
  // aux scans can land in parallel with the primary callback and with each
  // other; each aux only writes to its own buffer (mutex-protected), so no
  // shared mutable state is touched here.
  if (this->concat_enabled_ && !this->aux_lidars_.empty()) {
    this->aux_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto aux_sub_opt = rclcpp::SubscriptionOptions();
    aux_sub_opt.callback_group = this->aux_cb_group_;
    for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
      const std::string topic = this->aux_lidars_[i]->topic;
      const int idx = static_cast<int>(i);
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::SensorDataQoS(),
          [this, idx](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->callbackAuxPointCloud(idx, std::move(msg));
          },
          aux_sub_opt);
      this->aux_subs_.push_back(sub);
      RCLCPP_INFO(this->get_logger(), "Subscribed to aux LiDAR topic: %s", topic.c_str());
    }
  }

  // Use Reentrant callback group so IMU can process in parallel with pointcloud processing
  this->imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;

  // Set QoS for IMU subscriber (BEST_EFFORT to match sensor publishers)
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
  imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_qos,
      std::bind(&gicp_localization::LocalizationNode::callbackImu, this, std::placeholders::_1),
      imu_sub_opt);
  const std::string resolved_imu_topic = this->imu_sub->get_topic_name();
  if (this->imu_require_topic_allowlist_) {
    const bool allowed = std::find(this->imu_topic_allowlist_.begin(),
                                   this->imu_topic_allowlist_.end(),
                                   resolved_imu_topic) != this->imu_topic_allowlist_.end();
    if (!allowed) {
      std::ostringstream oss;
      for (size_t i = 0; i < this->imu_topic_allowlist_.size(); ++i) {
        if (i) oss << ", ";
        oss << this->imu_topic_allowlist_[i];
      }
      RCLCPP_FATAL(this->get_logger(),
                   "IMU topic hard guard triggered: resolved topic '%s' is not in allowlist [%s]. "
                   "Expected fused Point One Atlas IMU path.",
                   resolved_imu_topic.c_str(), oss.str().c_str());
      throw std::runtime_error("IMU topic hard guard mismatch");
    }
  }
  RCLCPP_INFO(this->get_logger(),
              "IMU input topic: %s (expect Point One Atlas INS IMU, frame='%s', strict_frame_match=%s)",
              resolved_imu_topic.c_str(),
              this->imu_frame.c_str(),
              this->imu_require_frame_match_ ? "true" : "false");

  // IMU input health check. The most common silent failure is launching with an
  // `imu_topic:=` arg that doesn't match any publisher — the subscription is
  // created but callbacks never fire and there's nothing in the log to tell
  // the user why. Fire a periodic timer that warns when no IMU has been
  // received AND no publisher exists on the resolved topic name. The timer
  // cancels itself once the first IMU arrives.
  this->input_health_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      [this]() {
        if (this->first_imu_received.load()) {
          this->input_health_timer_->cancel();
          return;
        }
        const std::string imu_topic = this->imu_sub->get_topic_name();
        const size_t pub_count = this->count_publishers(imu_topic);
        if (pub_count == 0) {
          RCLCPP_WARN(this->get_logger(),
                      "No IMU received on '%s' (0 publishers). Check the imu_topic launch arg. "
                      "Run `ros2 topic list | grep -i imu` to see available IMU topics. "
                      "Default expects /gps_p1/imu (Atlas FusionEngine imu_calibrated).",
                      imu_topic.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "No IMU received on '%s' yet, but %zu publisher(s) exist. "
                      "QoS mismatch or sim-time/clock issue is possible.",
                      imu_topic.c_str(), pub_count);
        }
      });

  // Optional ground-truth odom subscriber for divergence cross-check.
  // Topic is remappable as "gt_odom"; default points to /gps_p1/filtered_odom
  // in launch (Atlas FusionEngine INS at gps_antenna_top, matching base_frame).
  if (this->gt_odom_enabled_) {
    this->gt_odom_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto gt_sub_opt = rclcpp::SubscriptionOptions();
    gt_sub_opt.callback_group = this->gt_odom_cb_group;
    auto gt_qos = rclcpp::QoS(rclcpp::KeepLast(this->gt_odom_buffer_size_));
    gt_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    gt_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    this->gt_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "gt_odom", gt_qos,
        std::bind(&gicp_localization::LocalizationNode::callbackGtOdom, this, std::placeholders::_1),
        gt_sub_opt);
    RCLCPP_INFO(this->get_logger(),
                "Ground-truth odom cross-check ENABLED (topic remap 'gt_odom', buffer=%zu, max_dt=%.3fs)",
                this->gt_odom_buffer_size_, this->gt_odom_max_dt_);
  }

  // RTK quality gate is now self-contained in callbackGtOdom: it inspects
  // msg->pose.covariance on each gt_odom sample (no separate subscription).
  // Log the current configuration so the operator can see what's active.
  if (this->gt_odom_enabled_) {
    if (this->rtk_gate_enabled_) {
      RCLCPP_INFO(this->get_logger(),
                  "RTK quality gate ENABLED (P1-native): drop gt_odom when "
                  "pose covariance exceeds max_pose_var_xy=%.3f m^2 or "
                  "max_pose_var_z=%.3f m^2.",
                  this->rtk_gate_max_pose_var_xy_,
                  this->rtk_gate_max_pose_var_z_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "RTK quality gate DISABLED. gt_odom samples will be "
                  "accepted regardless of Atlas-reported pose covariance "
                  "-- snap and init may seed state from degraded GNSS. "
                  "Enable localization/rtk_gate/enable to reject samples "
                  "with poor covariance.");
    }
  }

  // Setup publishers
  this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("localized_pose", 10);

  // Note: Pose is published at IMU rate from propagateState() for perfect time synchronization
  RCLCPP_INFO(this->get_logger(), "Pose will be published at IMU rate (~100 Hz) from propagateState()");

  // High-frequency odometry publisher (100Hz from IMU propagation)
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1000));  // Large queue for 100Hz
  odom_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  odom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  this->localized_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("localized_odom", odom_qos);

  if (this->utm_enabled_) {
    this->utm_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/pose_utm", 10);
    this->utm_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("gicp/localization/odom_utm", odom_qos);
    this->utm_path_pub = this->create_publisher<nav_msgs::msg::Path>("gicp/localization/path_utm", 10);
  }

  if (this->debug_pub_enabled_) {
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("localized_path", 10);
    this->gt_snap_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/gt_snap", 10);
    this->dbg_initial_guess_pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/debug/initial_guess_pose", 10);
    this->dbg_final_pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/debug/final_pose", 10);
    this->dbg_pose_markers_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("gicp/localization/debug/pose_markers", 10);
    this->dbg_fitness_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/fitness", 10);
    this->dbg_gicp_elapsed_ms_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/gicp_elapsed_ms", 10);
    this->dbg_corr_norm_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/corr_norm", 10);
    this->dbg_scan_dt_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/scan_dt", 10);
    this->dbg_imu_age_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/imu_age", 10);
    this->dbg_num_correspondences_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/num_correspondences", 10);
    this->dbg_correspondence_ratio_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/correspondence_ratio", 10);
    this->dbg_final_error_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/final_error", 10);
    this->dbg_guess_to_solution_trans_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_to_solution_trans_m", 10);
    this->dbg_guess_to_solution_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_to_solution_rot_deg", 10);
    this->dbg_guess_from_last_trans_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_from_last_m", 10);
    this->dbg_guess_from_last_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_from_last_deg", 10);
    this->dbg_raw_points_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/raw_points", 10);
    this->dbg_preprocessed_points_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/preprocessed_points", 10);
    this->dbg_imu_buffer_span_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/imu_buffer_span_s", 10);
    this->dbg_scan_to_latest_imu_lag_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/scan_to_latest_imu_lag_s", 10);
    this->dbg_hessian_condition_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/hessian_condition_proxy", 10);
    this->dbg_jump_trans_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/jump_trans", 10);
    this->dbg_jump_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/jump_rot_deg", 10);
    this->dbg_converged_pub = this->create_publisher<std_msgs::msg::Bool>("gicp/localization/debug/converged", 10);
    this->dbg_gt_pos_err_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/gt_pos_err_m", 10);
    this->dbg_gt_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/gt_rot_err_deg", 10);
  }

  if (this->visualize_map_) {
    this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
  }

  // TF broadcaster
  if (this->publish_tf_) {
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  // TF buffer and listener for transforming incoming point clouds
  this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

  RCLCPP_INFO(this->get_logger(), "DLIO Localization Node Initialized");
  RCLCPP_INFO(this->get_logger(), "Map loaded with %lu points", this->map_cloud->points.size());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  this->applyInitialPoseFromParams();
}

gicp_localization::LocalizationNode::~LocalizationNode() {}

bool gicp_localization::LocalizationNode::loadUTMTransform(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open UTM transform file: %s", path.c_str());
    return false;
  }
  Eigen::Matrix4f T_world_utm = Eigen::Matrix4f::Identity();
  std::string line;
  int row = 0;
  while (std::getline(f, line) && row < 4) {
    if (line.empty() || line[0] == '#' || line.find("T_world_utm") != std::string::npos) continue;
    std::istringstream ss(line);
    for (int col = 0; col < 4; ++col) ss >> T_world_utm(row, col);
    ++row;
  }
  if (row < 4) {
    RCLCPP_ERROR(this->get_logger(), "UTM transform file malformed (only %d rows parsed): %s", row, path.c_str());
    return false;
  }
  this->T_utm_map_ = T_world_utm.inverse();
  RCLCPP_INFO(this->get_logger(), "Loaded UTM transform from %s (T_utm_map origin: [%.2f, %.2f, %.2f])",
    path.c_str(),
    this->T_utm_map_(0, 3), this->T_utm_map_(1, 3), this->T_utm_map_(2, 3));
  return true;
}

void gicp_localization::LocalizationNode::getParams() {

  // Frame IDs
  this->declare_parameter<std::string>("localization/map_frame", "map");
  this->declare_parameter<std::string>("localization/base_frame", "base_link");
  this->declare_parameter<std::string>("odom/odom_frame", "odom");
  this->declare_parameter<std::string>("localization/imu_frame", "imu");
  this->declare_parameter<std::string>("localization/lidar_frame", "lidar");

  this->get_parameter("localization/map_frame", this->map_frame);
  this->get_parameter("localization/base_frame", this->base_frame);
  this->get_parameter("odom/odom_frame", this->odom_frame);
  this->get_parameter("localization/imu_frame", this->imu_frame);
  this->get_parameter("localization/lidar_frame", this->lidar_frame);

  // Map parameters
  this->declare_parameter<std::string>("localization/map_path", "");
  this->declare_parameter<std::string>("localization/utm_transform_path", "");
  this->declare_parameter<std::string>("localization/utm_frame", "utm");
  this->declare_parameter<double>("localization/voxel_leaf_size", 0.25);
  this->declare_parameter<bool>("localization/visualize_map", true);
  this->declare_parameter<double>("localization/map_voxel_size_vis", 0.5);
  this->declare_parameter<double>("localization/map_rotation/roll_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/pitch_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/yaw_deg", 0.0);

  this->get_parameter("localization/map_path", this->map_path_);

  std::string utm_transform_path;
  this->get_parameter("localization/utm_transform_path", utm_transform_path);
  this->get_parameter("localization/utm_frame", this->utm_frame);
  this->utm_enabled_ = false;
  this->T_utm_map_ = Eigen::Matrix4f::Identity();
  if (!utm_transform_path.empty()) {
    this->utm_enabled_ = loadUTMTransform(utm_transform_path);
  }
  this->get_parameter("localization/voxel_leaf_size", this->voxel_leaf_size_);
  this->get_parameter("localization/visualize_map", this->visualize_map_);
  this->get_parameter("localization/map_voxel_size_vis", this->map_voxel_size_vis_);
  this->get_parameter("localization/map_rotation/roll_deg", this->map_roll_deg_);
  this->get_parameter("localization/map_rotation/pitch_deg", this->map_pitch_deg_);
  this->get_parameter("localization/map_rotation/yaw_deg", this->map_yaw_deg_);

  // Localization parameters
  this->declare_parameter<bool>("localization/publish_tf", true);
  this->declare_parameter<bool>("localization/imu_only", false);
  this->declare_parameter<bool>("localization/use_odom_init", true);
  this->declare_parameter<bool>("localization/initial_pose/use", false);
  this->declare_parameter<std::string>("localization/initial_pose/frame", "lidar");
  this->declare_parameter<double>("localization/initial_pose/x", 0.0);
  this->declare_parameter<double>("localization/initial_pose/y", 0.0);
  this->declare_parameter<double>("localization/initial_pose/z", 0.0);
  this->declare_parameter<double>("localization/initial_pose/roll", 0.0);
  this->declare_parameter<double>("localization/initial_pose/pitch", 0.0);
  this->declare_parameter<double>("localization/initial_pose/yaw", 0.0);

  // Ground-truth odom cross-check (optional). Uses topic remap "gt_odom".
  this->declare_parameter<bool>("localization/gt_odom/enable", false);
  this->declare_parameter<int>("localization/gt_odom/buffer_size", 200);
  this->declare_parameter<double>("localization/gt_odom/max_dt", 0.1);
  bool gt_enable = false; int gt_buf = 200; double gt_max_dt = 0.1;
  this->get_parameter("localization/gt_odom/enable", gt_enable);
  this->get_parameter("localization/gt_odom/buffer_size", gt_buf);
  this->get_parameter("localization/gt_odom/max_dt", gt_max_dt);
  this->gt_odom_enabled_ = gt_enable;
  this->gt_odom_buffer_size_ = static_cast<size_t>(std::max(gt_buf, 1));
  this->gt_odom_max_dt_ = gt_max_dt;

  // RTK quality gate (P1-native): drop gt_odom samples whose Atlas-reported
  // position covariance exceeds the configured thresholds. Conservative
  // defaults: gate ON; xy threshold 0.25 m^2 (~0.5 m std, comfortably above
  // RTK-fixed and float-mode covariances measured on AV-24 ~ 5e-5 m^2);
  // z threshold 1.0 m^2 (~1 m std, since GPS Z is naturally worse).
  this->declare_parameter<bool>("localization/rtk_gate/enable", true);
  this->declare_parameter<double>("localization/rtk_gate/max_pose_var_xy", 0.25);
  this->declare_parameter<double>("localization/rtk_gate/max_pose_var_z", 1.0);
  this->get_parameter("localization/rtk_gate/enable",
                      this->rtk_gate_enabled_);
  this->get_parameter("localization/rtk_gate/max_pose_var_xy",
                      this->rtk_gate_max_pose_var_xy_);
  this->get_parameter("localization/rtk_gate/max_pose_var_z",
                      this->rtk_gate_max_pose_var_z_);

  // GT-driven pose recovery (optional). Independent of gt_odom/enable; recovery
  // requires the same subscriber to be active, so it implies gt_odom/enable.
  this->declare_parameter<bool>("localization/gt_recovery/enable", false);
  this->declare_parameter<int>("localization/gt_recovery/min_consecutive_failures", 3);
  this->declare_parameter<bool>("localization/gt_recovery/hold_on_lidar_timeout", false);
  this->declare_parameter<double>("localization/gt_recovery/lidar_timeout_sec", 0.5);
  this->get_parameter("localization/gt_recovery/enable", this->gt_recovery_enabled_);
  this->get_parameter("localization/gt_recovery/min_consecutive_failures",
                      this->gt_recovery_min_consecutive_failures_);
  this->get_parameter("localization/gt_recovery/hold_on_lidar_timeout",
                      this->gt_recovery_hold_on_lidar_timeout_);
  this->get_parameter("localization/gt_recovery/lidar_timeout_sec",
                      this->gt_recovery_lidar_timeout_sec_);
  this->declare_parameter<bool>("localization/gt_rejection/enable", false);
  this->declare_parameter<double>("localization/gt_rejection/max_pos_err_m", 0.0);
  this->declare_parameter<double>("localization/gt_rejection/max_rot_err_deg", 0.0);
  this->get_parameter("localization/gt_rejection/enable", this->gt_rejection_enabled_);
  this->get_parameter("localization/gt_rejection/max_pos_err_m", this->gt_rejection_max_pos_err_m_);
  this->get_parameter("localization/gt_rejection/max_rot_err_deg", this->gt_rejection_max_rot_err_deg_);
  if (this->gt_recovery_enabled_ && !this->gt_odom_enabled_) {
    RCLCPP_WARN(this->get_logger(),
                "localization/gt_recovery/enable=true but gt_odom/enable=false — forcing gt_odom on so the buffer fills.");
    this->gt_odom_enabled_ = true;
  }
  if (this->gt_recovery_min_consecutive_failures_ < 1) {
    this->gt_recovery_min_consecutive_failures_ = 1;
  }
  if (this->gt_recovery_lidar_timeout_sec_ <= 0.0) {
    this->gt_recovery_lidar_timeout_sec_ = 0.5;
  }

  this->get_parameter("localization/publish_tf", this->publish_tf_);
  this->get_parameter("localization/imu_only", this->imu_only_mode_);
  this->get_parameter("localization/use_odom_init", this->use_odom_init_);
  this->get_parameter("localization/initial_pose/use", this->use_param_initial_pose_);
  this->get_parameter("localization/initial_pose/frame", this->initial_pose_frame_);
  this->get_parameter("localization/initial_pose/x", this->initial_pose_x_);
  this->get_parameter("localization/initial_pose/y", this->initial_pose_y_);
  this->get_parameter("localization/initial_pose/z", this->initial_pose_z_);
  this->get_parameter("localization/initial_pose/roll", this->initial_pose_roll_);
  this->get_parameter("localization/initial_pose/pitch", this->initial_pose_pitch_);
  this->get_parameter("localization/initial_pose/yaw", this->initial_pose_yaw_);

  // GICP parameters
  this->declare_parameter<int>("gicp/maxIterations", 32);
  this->declare_parameter<int>("gicp/correspondenceRandomness", 20);
  this->declare_parameter<double>("gicp/maxCorrespondenceDistance", 1.0);
  this->declare_parameter<double>("gicp/transformationEpsilon", 0.0001);
  this->declare_parameter<double>("gicp/rotationEpsilon", 0.0001);
  this->declare_parameter<double>("gicp/fitnessRejectThreshold", 1.0);
  this->declare_parameter<bool>("gicp/rejectLargeJumps", true);
  // Reject scans whose Hessian condition number proxy exceeds this threshold.
  // Straights typically run ~1e4; feature-poor corners spike to 1e8-1e9 and the
  // optimizer slides along the unconstrained axis. Set <=0 to disable the gate.
  this->declare_parameter<double>("gicp/hessianCondMax", 1.0e6);
  // Hessian rejection fires when condition number is high AND any of:
  //   - fitness exceeds the warn floor below
  //   - GICP applied a translation correction larger than transWarn
  //   - GICP applied a rotation correction larger than rotWarn
  // The latter two catch the optimizer sliding along an unconstrained axis: in
  // degenerate geometry IMU already provides a good prior, so a healthy GICP
  // correction is small. A large correction in degenerate geometry is the
  // slide signature even when fitness looks fine. Set any threshold <= 0 to
  // disable that specific OR branch.
  this->declare_parameter<double>("gicp/hessianFitnessWarnThreshold", 0.15);
  this->declare_parameter<double>("gicp/hessianTransWarnM", 1.0);
  this->declare_parameter<double>("gicp/hessianRotWarnDeg", 1.5);

  this->get_parameter("gicp/maxIterations", this->gicp_max_iter_);
  this->get_parameter("gicp/correspondenceRandomness", this->gicp_corr_randomness_);
  this->get_parameter("gicp/maxCorrespondenceDistance", this->gicp_max_corr_dist_);
  this->get_parameter("gicp/transformationEpsilon", this->gicp_transformation_epsilon_);
  this->get_parameter("gicp/rotationEpsilon", this->gicp_rotation_epsilon_);
  this->get_parameter("gicp/fitnessRejectThreshold", this->gicp_fitness_reject_threshold_);
  this->get_parameter("gicp/rejectLargeJumps", this->gicp_reject_large_jumps_);
  this->get_parameter("gicp/hessianCondMax", this->gicp_hessian_cond_max_);
  this->get_parameter("gicp/hessianFitnessWarnThreshold", this->gicp_hessian_fitness_warn_);
  this->get_parameter("gicp/hessianTransWarnM", this->gicp_hessian_trans_warn_m_);
  this->get_parameter("gicp/hessianRotWarnDeg", this->gicp_hessian_rot_warn_deg_);

  // Preprocessing parameters
  this->declare_parameter<double>("dlio/preprocessing/cropBoxFilter/size", 80.0);
  this->declare_parameter<bool>("dlio/preprocessing/voxelFilter/use", true);
  this->declare_parameter<double>("dlio/preprocessing/voxelFilter/res", 0.3);

  this->get_parameter("dlio/preprocessing/cropBoxFilter/size", this->crop_size_);
  this->get_parameter("dlio/preprocessing/voxelFilter/use", this->vf_use_);
  this->get_parameter("dlio/preprocessing/voxelFilter/res", this->vf_res_);

  // IMU and deskewing parameters
  this->declare_parameter<bool>("dlio/deskew", true);
  this->declare_parameter<double>("dlio/gravity", 9.81);
  this->declare_parameter<int>("dlio/imu/bufferSize", 2000);

  this->get_parameter("dlio/deskew", this->deskew_);
  this->get_parameter("dlio/gravity", this->gravity_);
  this->get_parameter("dlio/imu/bufferSize", this->imu_buffer_size_);

  this->declare_parameter<bool>("localization/flip_y", false);
  this->get_parameter("localization/flip_y", this->flip_y_);

  // Multi-LiDAR concatenation: merge nearest-in-time aux scans into the primary
  // PointCloud2 before the existing pipeline runs. Aux XYZ are transformed into
  // the primary sensor frame via TF (URDF). Luminar UINT8[8] per-point
  // timestamps are absolute epoch ns and remain unchanged; relative-time
  // formats are shifted by the inter-header dt.
  this->declare_parameter<bool>("localization/lidar_concat/enabled", false);
  this->declare_parameter<std::vector<std::string>>("localization/lidar_concat/aux_topics", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("localization/lidar_concat/aux_frames", std::vector<std::string>{});
  this->declare_parameter<double>("localization/lidar_concat/time_threshold", 0.05);
  this->declare_parameter<int>("localization/lidar_concat/buffer_size", 20);
  this->declare_parameter<bool>("localization/lidar_concat/delay_primary_until_aux", false);

  this->get_parameter("localization/lidar_concat/enabled", this->concat_enabled_);
  std::vector<std::string> aux_topics_param, aux_frames_param;
  this->get_parameter("localization/lidar_concat/aux_topics", aux_topics_param);
  this->get_parameter("localization/lidar_concat/aux_frames", aux_frames_param);
  this->get_parameter("localization/lidar_concat/time_threshold", this->concat_time_threshold_);
  this->get_parameter("localization/lidar_concat/delay_primary_until_aux", this->concat_delay_primary_until_aux_);
  int concat_buffer_size_int = 20;
  this->get_parameter("localization/lidar_concat/buffer_size", concat_buffer_size_int);
  this->concat_buffer_size_ = static_cast<size_t>(std::max(1, concat_buffer_size_int));

  if (this->concat_enabled_) {
    if (aux_topics_param.size() != aux_frames_param.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "lidar_concat: aux_topics size (%zu) != aux_frames size (%zu); disabling concat",
                   aux_topics_param.size(), aux_frames_param.size());
      this->concat_enabled_ = false;
    } else if (aux_topics_param.empty()) {
      RCLCPP_WARN(this->get_logger(), "lidar_concat enabled but no aux_topics configured; disabling concat");
      this->concat_enabled_ = false;
    } else {
      for (size_t i = 0; i < aux_topics_param.size(); ++i) {
        auto aux = std::make_unique<AuxLidar>();
        aux->topic = aux_topics_param[i];
        aux->frame = aux_frames_param[i];
        aux->T_primary_aux = Eigen::Matrix4f::Identity();
        aux->extrinsic_cached = false;
        this->aux_lidars_.push_back(std::move(aux));
      }
      RCLCPP_INFO(this->get_logger(),
                  "lidar_concat enabled: %zu aux lidars, time_threshold=%.3fs, buffer_size=%zu, delay_primary_until_aux=%s",
                  this->aux_lidars_.size(), this->concat_time_threshold_, this->concat_buffer_size_,
                  this->concat_delay_primary_until_aux_ ? "true" : "false");
      for (const auto& a : this->aux_lidars_) {
        RCLCPP_INFO(this->get_logger(), "  aux lidar: topic='%s' frame='%s'",
                    a->topic.c_str(), a->frame.c_str());
      }
    }
  }

  // IMU calibration time (seconds of stationary data to average for bias/gravity)
  this->declare_parameter<double>("dlio/imu/calibTime", 3.0);
  this->get_parameter("dlio/imu/calibTime", this->imu_calib_time_);
  // Hard guard for single-source P1 deployments: by default the localizer only
  // accepts an IMU subscription resolved to /gps_p1/imu.
  this->declare_parameter<bool>("localization/imu/require_topic_allowlist", true);
  this->declare_parameter<std::vector<std::string>>(
      "localization/imu/topic_allowlist", std::vector<std::string>{"/gps_p1/imu"});
  this->get_parameter("localization/imu/require_topic_allowlist", this->imu_require_topic_allowlist_);
  this->get_parameter("localization/imu/topic_allowlist", this->imu_topic_allowlist_);
  if (this->imu_topic_allowlist_.empty()) {
    this->imu_topic_allowlist_.push_back("/gps_p1/imu");
  }
  // Safety guard: reject IMU samples whose header.frame_id does not match
  // localization/imu_frame. Keep enabled by default for P1-only operation.
  this->declare_parameter<bool>("localization/imu/require_frame_match", true);
  this->get_parameter("localization/imu/require_frame_match", this->imu_require_frame_match_);

  // RTK-driven IMU calibration. When enabled, the first accepted GT odom
  // sample triggers a calibration window in which IMU residuals are computed
  // against the GT pose/twist (no stationary assumption). With the RTK gate
  // enabled, "accepted" means /gps_p1/filtered_odom reported pose covariance
  // within the configured thresholds; disabling the gate for bag replay
  // removes that guarantee. Falls back to stationary calibration if no
  // accepted GT arrives within fallback_timeout.
  this->declare_parameter<bool>("localization/rtk_init/enable", true);
  this->declare_parameter<double>("localization/rtk_init/calib_window", 2.0);
  this->declare_parameter<double>("localization/rtk_init/fallback_timeout", 5.0);
  this->get_parameter("localization/rtk_init/enable", this->rtk_init_enabled_);
  this->get_parameter("localization/rtk_init/calib_window", this->rtk_calib_window_sec_);
  this->get_parameter("localization/rtk_init/fallback_timeout", this->rtk_fallback_timeout_sec_);
  RCLCPP_INFO(this->get_logger(),
              "RTK-driven IMU calibration: %s (window=%.1fs, fallback_timeout=%.1fs)",
              this->rtk_init_enabled_ ? "ENABLED" : "disabled",
              this->rtk_calib_window_sec_, this->rtk_fallback_timeout_sec_);

  // Sensor type for per-point timestamp handling during deskewing
  this->declare_parameter<std::string>("localization/sensor_type", "ouster");
  std::string sensor_type_str;
  this->get_parameter("localization/sensor_type", sensor_type_str);
  if (sensor_type_str == "luminar") {
    this->sensor = dlio::SensorType::LUMINAR;
  } else if (sensor_type_str == "velodyne") {
    this->sensor = dlio::SensorType::VELODYNE;
  } else if (sensor_type_str == "hesai") {
    this->sensor = dlio::SensorType::HESAI;
  } else if (sensor_type_str == "livox") {
    this->sensor = dlio::SensorType::LIVOX;
  } else if (sensor_type_str == "ouster") {
    this->sensor = dlio::SensorType::OUSTER;
  } else {
    this->sensor = dlio::SensorType::UNKNOWN;
    RCLCPP_WARN(this->get_logger(),
                "Unknown localization/sensor_type '%s'; per-point deskew needs ouster, velodyne, "
                "hesai, livox, or luminar",
                sensor_type_str.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Sensor type: %s", sensor_type_str.c_str());

  // Geometric Observer parameters. Position/orientation gains stay active, but
  // online IMU bias adaptation defaults off for the fused Point One Atlas INS path; the
  // initial RTK/stationary calibration still seeds state.b once before
  // propagation.
  this->declare_parameter<double>("odom/geo/Kp", 4.5);
  this->declare_parameter<double>("odom/geo/Kv", 11.25);
  this->declare_parameter<double>("odom/geo/Kq", 4.0);
  this->declare_parameter<double>("odom/geo/Kab", 0.0);
  this->declare_parameter<double>("odom/geo/Kgb", 0.0);
  this->declare_parameter<double>("odom/geo/Kz_damping", 5.0);
  this->declare_parameter<double>("odom/geo/abias_max", 5.0);
  this->declare_parameter<double>("odom/geo/gbias_max", 0.5);

  this->get_parameter("odom/geo/Kp", this->geo_Kp_);
  this->get_parameter("odom/geo/Kv", this->geo_Kv_);
  this->get_parameter("odom/geo/Kq", this->geo_Kq_);
  this->get_parameter("odom/geo/Kab", this->geo_Kab_);
  this->get_parameter("odom/geo/Kgb", this->geo_Kgb_);
  this->get_parameter("odom/geo/Kz_damping", this->geo_Kz_damping_);
  this->get_parameter("odom/geo/abias_max", this->geo_abias_max_);
  this->get_parameter("odom/geo/gbias_max", this->geo_gbias_max_);

  // Debug parameters
  this->declare_parameter<bool>("localization/debug/enable_pub", true);
  this->declare_parameter<bool>("localization/debug/enable_jump_log", true);
  this->declare_parameter<bool>("localization/debug/verbose_scan_log", false);
  this->declare_parameter<bool>("localization/debug/nano_gicp_lm_debug", false);
  this->declare_parameter<double>("localization/debug/jump_trans_m", 1.0);
  this->declare_parameter<double>("localization/debug/jump_rot_deg", 10.0);
  this->declare_parameter<bool>("localization/verbose", true);

  this->get_parameter("localization/debug/enable_pub", this->debug_pub_enabled_);
  this->get_parameter("localization/debug/enable_jump_log", this->debug_jump_log_enabled_);
  this->get_parameter("localization/debug/verbose_scan_log", this->debug_verbose_scan_log_);
  this->get_parameter("localization/debug/nano_gicp_lm_debug", this->debug_lm_print_);
  this->get_parameter("localization/debug/jump_trans_m", this->debug_jump_trans_m_);
  this->get_parameter("localization/debug/jump_rot_deg", this->debug_jump_rot_deg_);
  this->get_parameter("localization/verbose", this->verbose_);

  // Suppress INFO/DEBUG logs when verbose is off; WARN/ERROR still pass through.
  if (!this->verbose_) {
    rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_WARN);
  }

  RCLCPP_INFO(this->get_logger(), "Preprocessing config: crop_size=%.2f, voxel_filter=%s, voxel_res=%.2f",
              this->crop_size_, this->vf_use_ ? "ENABLED" : "DISABLED", this->vf_res_);
  RCLCPP_INFO(this->get_logger(), "IMU config: deskew=%s, gravity=%.2f, buffer_size=%d",
              this->deskew_ ? "ENABLED" : "DISABLED", this->gravity_, this->imu_buffer_size_);
  RCLCPP_INFO(this->get_logger(), "Geometric Observer: Kp=%.2f, Kv=%.2f, Kq=%.2f, Kab=%.2f, Kgb=%.2f",
              this->geo_Kp_, this->geo_Kv_, this->geo_Kq_, this->geo_Kab_, this->geo_Kgb_);
  RCLCPP_INFO(this->get_logger(), "Localization mode: %s",
              this->imu_only_mode_ ? "IMU-only (GICP disabled)" : "GICP + IMU");
  RCLCPP_INFO(this->get_logger(),
              "GICP rejection: fitness>%.3f, large_jump=%s, hessian_cond>%.2e AND (fitness>%.3f OR trans>%.2fm OR rot>%.2fdeg) (%s)",
              this->gicp_fitness_reject_threshold_,
              this->gicp_reject_large_jumps_ ? "on" : "off",
              this->gicp_hessian_cond_max_,
              this->gicp_hessian_fitness_warn_,
              this->gicp_hessian_trans_warn_m_,
              this->gicp_hessian_rot_warn_deg_,
              this->gicp_hessian_cond_max_ > 0.0 ? "on" : "disabled");
  RCLCPP_INFO(this->get_logger(),
              "GT recovery: %s (min consecutive failures=%d, lidar-timeout hold=%s after %.2fs)",
              this->gt_recovery_enabled_ ? "ENABLED" : "DISABLED",
              this->gt_recovery_min_consecutive_failures_,
              this->gt_recovery_hold_on_lidar_timeout_ ? "on" : "off",
              this->gt_recovery_lidar_timeout_sec_);
  RCLCPP_INFO(this->get_logger(),
              "GT rejection gate: %s (pos>%.2fm or rot>%.1fdeg)",
              this->gt_rejection_enabled_ ? "ENABLED" : "DISABLED",
              this->gt_rejection_max_pos_err_m_,
              this->gt_rejection_max_rot_err_deg_);
  RCLCPP_INFO(this->get_logger(), "Debug: publish=%s jump_log=%s thresholds=[%.2fm, %.1fdeg]",
              this->debug_pub_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_log_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_trans_m_, this->debug_jump_rot_deg_);
  RCLCPP_INFO(this->get_logger(), "Debug detail: verbose_scan_log=%s nano_gicp_lm_debug=%s",
              this->debug_verbose_scan_log_ ? "ENABLED" : "DISABLED",
              this->debug_lm_print_ ? "ENABLED" : "DISABLED");
}

bool gicp_localization::LocalizationNode::loadMap() {

  if (this->map_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Map path is empty! Please set localization/map_path parameter.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", this->map_path_.c_str());

  // Load PCD file
  if (pcl::io::loadPCDFile<PointType>(this->map_path_, *this->map_cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", this->map_path_.c_str());
    return false;
  }

  if (this->map_cloud->points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Loaded map is empty!");
    return false;
  }

  // Optional static map rotation to correct coordinate-frame differences from source map files.
  if (std::abs(this->map_roll_deg_) > 1e-6 ||
      std::abs(this->map_pitch_deg_) > 1e-6 ||
      std::abs(this->map_yaw_deg_) > 1e-6) {
    constexpr float kDeg2Rad = 0.017453292519943295f;
    const float roll = static_cast<float>(this->map_roll_deg_ * kDeg2Rad);
    const float pitch = static_cast<float>(this->map_pitch_deg_ * kDeg2Rad);
    const float yaw = static_cast<float>(this->map_yaw_deg_ * kDeg2Rad);

    Eigen::Affine3f map_tf = Eigen::Affine3f::Identity();
    map_tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*this->map_cloud, *this->map_cloud, map_tf.matrix());

    RCLCPP_INFO(this->get_logger(),
                "Applied map rotation [roll=%.2f, pitch=%.2f, yaw=%.2f] deg",
                this->map_roll_deg_, this->map_pitch_deg_, this->map_yaw_deg_);
  }

  RCLCPP_INFO(this->get_logger(), "Map loaded successfully with %lu points", this->map_cloud->points.size());

  // Downsample map for visualization if needed
  if (this->visualize_map_) {
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(this->map_voxel_size_vis_, this->map_voxel_size_vis_, this->map_voxel_size_vis_);
    vg.setInputCloud(this->map_cloud);
    vg.filter(*this->map_cloud_ds);
    RCLCPP_INFO(this->get_logger(), "Downsampled map for visualization: %lu points", this->map_cloud_ds->points.size());
  }

  return true;
}

void gicp_localization::LocalizationNode::start() {
  // Publish map periodically
  if (this->visualize_map_) {
    auto timer_callback = [this]() {
      sensor_msgs::msg::PointCloud2 map_msg;
      pcl::toROSMsg(*this->map_cloud_ds, map_msg);
      map_msg.header.stamp = this->now();
      map_msg.header.frame_id = this->map_frame;
      this->map_pub->publish(map_msg);
    };
    this->map_pub_timer_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback);
  }

  // Republish the configured initial pose (PoseStamped only — NOT TF) until
  // GICP produces a real result, so RViz has something to show before scans
  // arrive. We deliberately do NOT publish a map->base_link TF here because
  // under use_sim_time, this->now() returns 0 until /clock is active, and
  // a TF stamped at time 0 poisons the TF buffer with OLD_DATA warnings.
  if (this->initialized && this->debug_pub_enabled_) {
    auto initial_pose_cb = [this]() {
      if (this->last_gicp_valid_) {
        this->initial_pose_pub_timer_->cancel();
        return;
      }
      const rclcpp::Time stamp = this->now();
      // Skip while sim time is still 0 (clock not yet flowing)
      if (stamp.nanoseconds() == 0) {
        return;
      }
      Eigen::Matrix4f pose;
      {
        std::lock_guard<std::mutex> lock(this->pose_mutex);
        pose = this->current_pose;
      }
      this->dbg_initial_guess_pose_pub->publish(
          poseStampedFromMatrix(pose, stamp, this->map_frame));
    };
    this->initial_pose_pub_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200), initial_pose_cb);
  }
}

void gicp_localization::LocalizationNode::applyInitialPoseFromParams() {

  if (!this->use_param_initial_pose_) {
    return;
  }

  Eigen::Vector3f position(static_cast<float>(this->initial_pose_x_),
                           static_cast<float>(this->initial_pose_y_),
                           static_cast<float>(this->initial_pose_z_));

  Eigen::AngleAxisf roll_angle(static_cast<float>(this->initial_pose_roll_), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(static_cast<float>(this->initial_pose_pitch_), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(static_cast<float>(this->initial_pose_yaw_), Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf orientation = yaw_angle * pitch_angle * roll_angle;
  orientation.normalize();

  // Pose values may be supplied in the LiDAR frame (matching GLIM's
  // traj_lidar.txt), but internally this node tracks the base_link pose in
  // world. When frame=="lidar", convert T_world_lidar -> T_world_base by
  // post-multiplying inv(baselink2lidar_T). This requires the URDF TF from
  // robot_state_publisher; defer until the first pointcloud if not yet cached.
  if (this->initial_pose_frame_ == "lidar") {
    if (!this->extrinsics_cached_) {
      this->pending_initial_pose_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Initial pose in lidar frame; deferring apply until baselink->lidar TF is cached");
      return;
    }
    const Eigen::Matrix3f& R_bl = this->extrinsics.baselink2lidar.R;
    const Eigen::Vector3f& t_bl = this->extrinsics.baselink2lidar.t;
    // T_world_base.t = R_world_lidar * (-R_base_lidar^T * t_base_lidar) + t_world_lidar
    // With lidar == luminar_front (no rotation in URDF) this simplifies to
    // t_world_base = t_world_lidar - R_world_lidar * R_base_lidar * (R_base_lidar^T * t_base_lidar).
    // We compute the general form: T_world_base = T_world_lidar * inv(T_base_lidar).
    Eigen::Matrix3f R_lb = R_bl.transpose();
    Eigen::Vector3f t_lb = -R_lb * t_bl;
    Eigen::Matrix3f R_world_lidar = orientation.toRotationMatrix();
    Eigen::Vector3f position_base = R_world_lidar * t_lb + position;
    Eigen::Quaternionf orientation_base(R_world_lidar * R_lb);
    orientation_base.normalize();
    RCLCPP_INFO(this->get_logger(),
                "Converted initial lidar pose -> base_link: [%.3f, %.3f, %.3f] (lidar) -> [%.3f, %.3f, %.3f] (base)",
                position.x(), position.y(), position.z(),
                position_base.x(), position_base.y(), position_base.z());
    position = position_base;
    orientation = orientation_base;
  }
  this->pending_initial_pose_ = false;

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = position;
    this->initialized = true;
  }

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = position;
    this->state.q = orientation;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
    this->geo.prev_p = position;
    this->geo.prev_q = orientation;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  this->basePose.p = position;
  this->basePose.q = orientation;

  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = this->now();

  RCLCPP_INFO(this->get_logger(),
              "Initial pose loaded from parameters at [%.2f, %.2f, %.2f] m with RPY [%.2f, %.2f, %.2f] rad",
              this->initial_pose_x_, this->initial_pose_y_, this->initial_pose_z_,
              this->initial_pose_roll_, this->initial_pose_pitch_, this->initial_pose_yaw_);
}

void gicp_localization::LocalizationNode::applyInitialPose(const Eigen::Vector3f& p,
                                                          const Eigen::Quaternionf& q_in,
                                                          const rclcpp::Time& stamp,
                                                          const std::string& source) {

  Eigen::Quaternionf q = q_in;
  if (q.squaredNorm() < 1e-10f) {
    RCLCPP_WARN(this->get_logger(), "Received near-zero quaternion in initial pose, ignoring");
    return;
  }
  q.normalize();

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = p;
    this->initialized = true;
    if (stamp.nanoseconds() > 0) {
      this->scan_stamp = stamp;
    }
  }

  // Reset geometric observer state to prevent drift from previous estimates
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = p;
    this->state.q = q;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
    this->geo.prev_p = p;
    this->geo.prev_q = q;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  this->basePose.p = p;
  this->basePose.q = q;

  // Clear trajectory path on reinitialization
  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = stamp.nanoseconds() > 0 ? stamp : this->now();

  RCLCPP_INFO(this->get_logger(), "Received initial pose (%s) at [%.2f, %.2f, %.2f]",
              source.c_str(), p.x(), p.y(), p.z());
}

void gicp_localization::LocalizationNode::callbackInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose) {

  Eigen::Quaternionf q(
      pose->pose.pose.orientation.w,
      pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y,
      pose->pose.pose.orientation.z);

  Eigen::Vector3f p(
      pose->pose.pose.position.x,
      pose->pose.pose.position.y,
      pose->pose.pose.position.z);

  this->applyInitialPose(p, q, pose->header.stamp, "PoseWithCovarianceStamped");
}

void gicp_localization::LocalizationNode::callbackPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_in) {

  if (this->imu_only_mode_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "IMU-only mode enabled: skipping pointcloud/GICP updates.");
    return;
  }

  const bool delay_primary =
      this->concat_enabled_ &&
      this->concat_delay_primary_until_aux_ &&
      this->sensor == dlio::SensorType::LUMINAR &&
      !this->aux_lidars_.empty();
  if (delay_primary) {
    {
      std::lock_guard<std::mutex> lk(this->delayed_primary_mtx_);
      this->delayed_primary_buffer_.push_back(pc_in);
    }
    this->drainDelayedPrimaryClouds();
    return;
  }

  std::lock_guard<std::mutex> process_lock(this->pointcloud_process_mtx_);
  this->processPointCloud(pc_in);
}

void gicp_localization::LocalizationNode::processPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_in) {

  // Multi-LiDAR concatenation: merge nearest aux scans into the primary cloud
  // before any other processing. Downstream steps (TF cache, manual field
  // extraction, Luminar timestamp read, Y-flip, deskew, GICP) all run on the
  // merged cloud unchanged — primary frame_id, point_step, and field layout
  // are preserved.
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc =
      this->concat_enabled_ ? this->mergeAuxClouds(pc_in) : pc_in;

  // Cache base_link -> lidar extrinsic from TF once. With
  // robot_state_publisher providing the URDF TF tree, this is the true
  // lever arm from the vehicle chassis (base_link) to the LiDAR sensor.
  // Deskewing below chains this as `frames[i] * baselink2lidar_T` so the
  // incoming points stay in their native LiDAR frame until then.
  if (!this->extrinsics_cached_) {
    try {
      auto tf_bl = this->tf_buffer->lookupTransform(
          this->base_frame, pc->header.frame_id, tf2::TimePointZero);
      Eigen::Quaternionf q_bl(
          tf_bl.transform.rotation.w, tf_bl.transform.rotation.x,
          tf_bl.transform.rotation.y, tf_bl.transform.rotation.z);
      Eigen::Vector3f t_bl(
          tf_bl.transform.translation.x, tf_bl.transform.translation.y,
          tf_bl.transform.translation.z);
      this->extrinsics.baselink2lidar.R = q_bl.toRotationMatrix();
      this->extrinsics.baselink2lidar.t = t_bl;
      this->extrinsics.baselink2lidar_T.setIdentity();
      this->extrinsics.baselink2lidar_T.block<3, 3>(0, 0) = q_bl.toRotationMatrix();
      this->extrinsics.baselink2lidar_T.block<3, 1>(0, 3) = t_bl;
      this->extrinsics_cached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Cached baselink->lidar extrinsic from '%s': t=[%.3f,%.3f,%.3f]",
                  pc->header.frame_id.c_str(), t_bl.x(), t_bl.y(), t_bl.z());
      if (this->pending_initial_pose_) {
        this->applyInitialPoseFromParams();
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for baselink->lidar TF ('%s' -> '%s'): %s",
                           this->base_frame.c_str(), pc->header.frame_id.c_str(), ex.what());
      return;
    }
  }

  this->last_pointcloud_stamp_.store(pc->header.stamp.sec + pc->header.stamp.nanosec * 1e-9);

  if (!this->initialized) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for initialization (odom or initial pose)...");
    return;
  }

  this->scan_stamp = pc->header.stamp;
  this->last_scan_input_frame_ = pc->header.frame_id;

  // Convert to PCL format using manual field extraction for robustness
  pcl::PointCloud<PointType>::Ptr raw_scan = std::make_shared<pcl::PointCloud<PointType>>();

  // Calculate number of points
  size_t num_points = static_cast<size_t>(pc->width) * pc->height;

  RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2: width=%d, height=%d, num_points=%lu, data_size=%lu",
               pc->width, pc->height, num_points, pc->data.size());

  if (num_points == 0) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud (width=%d, height=%d)", pc->width, pc->height);
    return;
  }

  // Single-pass conversion: resolve field offsets once, then walk pc->data
  // exactly once doing xyz + intensity + per-point time + flip_y in the same
  // iteration.
  int x_off = -1, y_off = -1, z_off = -1, i_off = -1;
  uint8_t i_type = 0;
  for (const auto& field : pc->fields) {
    if (field.name == "x") x_off = static_cast<int>(field.offset);
    else if (field.name == "y") y_off = static_cast<int>(field.offset);
    else if (field.name == "z") z_off = static_cast<int>(field.offset);
    else if (field.name == "intensity") {
      i_off = static_cast<int>(field.offset);
      i_type = field.datatype;
    }
  }

  int time_off = -1;
  uint8_t time_datatype = 0;
  int time_count = 0;
  const bool has_time_field = findTimeField(*pc, time_off, time_datatype, time_count);

  // One-shot timestamp-field diagnostic. Fires exactly once across the whole
  // node lifetime (std::call_once) and dumps every PointField + the first few
  // points' timestamp bytes interpreted four ways. The developer reads the
  // [LUMINAR_TS_DIAG] block in stderr to decide which bit-level interpretation
  // the live driver actually uses. See
  // docs/luminar_timestamp_diagnostic_guide.pdf for how to interpret the
  // output and the corresponding fix in copyPointTimeFromCloud.
  static std::once_flag ts_diag_once;
  std::call_once(ts_diag_once, [&]() {
    const char* sensor_name =
        this->sensor == dlio::SensorType::LUMINAR  ? "luminar"
        : this->sensor == dlio::SensorType::OUSTER ? "ouster"
        : this->sensor == dlio::SensorType::VELODYNE ? "velodyne"
        : this->sensor == dlio::SensorType::HESAI   ? "hesai"
        : this->sensor == dlio::SensorType::LIVOX   ? "livox"
                                                    : "unknown";
    logTimestampDiagnostic(*pc, time_off, time_datatype, time_count,
                           sensor_name);
  });

  if (x_off < 0 || y_off < 0 || z_off < 0) {
    RCLCPP_ERROR(this->get_logger(), "Point cloud missing x/y/z fields");
    return;
  }

  if (this->deskew_ && !has_time_field) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "dlio/deskew enabled but point cloud has no per-point time field "
                         "(t/time/timestamp)");
  } else if (this->sensor != dlio::SensorType::UNKNOWN && !has_time_field) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "No per-point time field (t/time/timestamp) in cloud; deskew will not "
                     "work until one is present");
  }

  raw_scan->points.resize(num_points);
  raw_scan->width = pc->width;
  raw_scan->height = pc->height;
  raw_scan->is_dense = pc->is_dense;

  const bool flip_y = this->flip_y_;
  const uint32_t point_step = pc->point_step;
  const uint8_t* base = pc->data.data();

  // Per-point body templated on the intensity reader, so the dispatch happens
  // once outside the loop and the compiler can inline + auto-vectorize.
  auto run = [&](auto&& read_intensity) {
    for (size_t i = 0; i < num_points; ++i) {
      const uint8_t* src = base + i * point_step;
      auto& dst = raw_scan->points[i];

      float x, y, z;
      std::memcpy(&x, src + x_off, sizeof(float));
      std::memcpy(&y, src + y_off, sizeof(float));
      std::memcpy(&z, src + z_off, sizeof(float));
      dst.x = x;
      dst.y = flip_y ? -y : y;
      dst.z = z;
      dst.intensity = read_intensity(src);
      clearPointTimeUnion(dst);
      if (has_time_field) {
        copyPointTimeFromCloud(src, time_off, time_datatype, time_count, this->sensor, dst);
      }
    }
  };

  try {
    if (i_off < 0) {
      run([](const uint8_t*) { return 0.0f; });
    } else {
      const uint8_t i_type_local = i_type;
      const int i_off_local = i_off;
      switch (i_type_local) {
        case sensor_msgs::msg::PointField::FLOAT32:
          run([i_off_local](const uint8_t* src) {
            float v;
            std::memcpy(&v, src + i_off_local, sizeof(float));
            return v;
          });
          break;
        case sensor_msgs::msg::PointField::UINT16:
          run([i_off_local](const uint8_t* src) {
            uint16_t v;
            std::memcpy(&v, src + i_off_local, sizeof(uint16_t));
            return static_cast<float>(v);
          });
          break;
        case sensor_msgs::msg::PointField::UINT8:
          run([i_off_local](const uint8_t* src) {
            return static_cast<float>(*(src + i_off_local));
          });
          break;
        case sensor_msgs::msg::PointField::FLOAT64:
          run([i_off_local](const uint8_t* src) {
            double v;
            std::memcpy(&v, src + i_off_local, sizeof(double));
            return static_cast<float>(v);
          });
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unknown intensity type %d, ignoring", i_type_local);
          run([](const uint8_t*) { return 0.0f; });
          break;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during point cloud conversion: %s", e.what());
    return;
  }

  this->last_raw_point_count_ = raw_scan->points.size();

  if (this->sensor == dlio::SensorType::LUMINAR && has_time_field && this->verbose_ &&
      !raw_scan->points.empty()) {
    logLuminarTimestampStats(raw_scan->points.size(), *raw_scan, 0);
  }

  // Store as original scan for deskewing
  this->original_scan = raw_scan;

  // Save dt for geometric observer BEFORE deskew (which overwrites prev_scan_stamp)
  this->observer_dt_ = (this->prev_scan_stamp > 0.0)
                        ? this->scan_stamp.seconds() - this->prev_scan_stamp
                        : 0.0;

  // Deskew using IMU
  this->deskewPointcloud();

  RCLCPP_DEBUG(this->get_logger(), "After deskewing: current_scan has %lu points",
               this->current_scan->points.size());

  // Preprocess the deskewed scan
  RCLCPP_DEBUG(this->get_logger(), "Before preprocessing: current_scan has %lu points",
               this->current_scan->points.size());

  this->preprocessPointCloud(this->current_scan);

  RCLCPP_DEBUG(this->get_logger(), "After preprocessing: current_scan has %lu points",
               this->current_scan->points.size());
  this->last_preprocessed_point_count_ = this->current_scan->points.size();

  if (this->current_scan->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud empty after preprocessing (original had %lu points)",
                raw_scan->points.size());

    if (this->debug_verbose_scan_log_) {
      RCLCPP_WARN(this->get_logger(),
                  "SCAN DEBUG | stamp=%.3f frame=%s raw=%zu pre=0 status=empty_after_preprocess guess=%s",
                  this->scan_stamp.seconds(), this->last_scan_input_frame_.c_str(),
                  this->last_raw_point_count_, poseSummary(this->current_pose).c_str());
    }
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "After preprocessing: %lu points", this->current_scan->points.size());

  // Perform localization
  RCLCPP_DEBUG(this->get_logger(), "Calling performLocalization()...");
  this->performLocalization();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization() completed");

  // Publish results
  RCLCPP_DEBUG(this->get_logger(), "Calling publishPose()...");
  this->publishPose();
  RCLCPP_DEBUG(this->get_logger(), "publishPose() completed");
}

void gicp_localization::LocalizationNode::callbackAuxPointCloud(
    int aux_index, sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (aux_index < 0 || static_cast<size_t>(aux_index) >= this->aux_lidars_.size()) {
    return;
  }
  {
    auto& aux = *this->aux_lidars_[aux_index];
    std::lock_guard<std::mutex> lk(aux.mtx);
    aux.buffer.push_back(std::move(msg));
    while (aux.buffer.size() > this->concat_buffer_size_) {
      aux.buffer.pop_front();
    }
  }

  if (this->concat_enabled_ && this->concat_delay_primary_until_aux_ &&
      this->sensor == dlio::SensorType::LUMINAR) {
    this->drainDelayedPrimaryClouds();
  }
}

bool gicp_localization::LocalizationNode::delayedPrimaryReady(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary) {

  if (!primary || this->aux_lidars_.empty()) {
    return true;
  }

  double primary_time_key = rclcpp::Time(primary->header.stamp).seconds();
  const bool use_luminar_time_key =
      this->sensor == dlio::SensorType::LUMINAR &&
      luminarCloudTimeMidpointSec(*primary, primary_time_key);

  for (auto& aux_ptr : this->aux_lidars_) {
    auto& aux = *aux_ptr;
    double latest_aux_time_key = -std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lk(aux.mtx);
      if (aux.buffer.empty()) {
        return false;
      }
      for (const auto& msg : aux.buffer) {
        double aux_time_key = rclcpp::Time(msg->header.stamp).seconds();
        if (use_luminar_time_key) {
          double aux_midpoint_sec = 0.0;
          if (!luminarCloudTimeMidpointSec(*msg, aux_midpoint_sec)) {
            continue;
          }
          aux_time_key = aux_midpoint_sec;
        }
        latest_aux_time_key = std::max(latest_aux_time_key, aux_time_key);
      }
    }
    if (!std::isfinite(latest_aux_time_key)) {
      return false;
    }
    constexpr double kCoverageSlackSec = 1e-4;
    if (latest_aux_time_key + kCoverageSlackSec < primary_time_key) {
      return false;
    }
  }

  return true;
}

void gicp_localization::LocalizationNode::drainDelayedPrimaryClouds() {
  std::unique_lock<std::mutex> process_lock(this->pointcloud_process_mtx_, std::try_to_lock);
  if (!process_lock.owns_lock()) {
    return;
  }

  while (true) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr primary;
    bool forced = false;
    {
      std::lock_guard<std::mutex> lk(this->delayed_primary_mtx_);
      if (this->delayed_primary_buffer_.empty()) {
        return;
      }
      forced = this->delayed_primary_buffer_.size() > this->concat_buffer_size_;
      if (!forced && !this->delayedPrimaryReady(this->delayed_primary_buffer_.front())) {
        return;
      }
      primary = this->delayed_primary_buffer_.front();
      this->delayed_primary_buffer_.pop_front();
    }

    if (forced) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: processing delayed primary without full aux timestamp coverage");
    }
    this->processPointCloud(primary);
  }
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr
gicp_localization::LocalizationNode::mergeAuxClouds(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary) {

  if (this->aux_lidars_.empty()) return primary;

  const double t_primary = rclcpp::Time(primary->header.stamp).seconds();
  double primary_time_key = t_primary;
  const bool use_luminar_time_key =
      this->sensor == dlio::SensorType::LUMINAR &&
      luminarCloudTimeMidpointSec(*primary, primary_time_key);
  const uint32_t point_step = primary->point_step;
  const std::string& primary_frame = primary->header.frame_id;

  int x_off, y_off, z_off;
  if (!findXYZOffsets(*primary, x_off, y_off, z_off)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lidar_concat: cannot find xyz fields in primary cloud (frame='%s')",
                         primary_frame.c_str());
    return primary;
  }

  // Start the merged cloud as a copy of the primary; we'll append aux bytes.
  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  size_t total_points = static_cast<size_t>(primary->width) * primary->height;
  size_t merged_aux_count = 0;

  // Reserve once for primary + all aux clouds (assuming roughly equal sizes).
  // Avoids per-aux reallocations as we grow merged->data.
  merged->data.reserve(primary->data.size() * (1 + this->aux_lidars_.size()));

  for (auto& aux_ptr : this->aux_lidars_) {
    auto& aux = *aux_ptr;

    // Cache T_primary_aux from TF on first use. Skip this aux until TF is available.
    if (!aux.extrinsic_cached) {
      try {
        auto tf = this->tf_buffer->lookupTransform(
            primary_frame, aux.frame, tf2::TimePointZero);
        Eigen::Quaternionf q(
            tf.transform.rotation.w, tf.transform.rotation.x,
            tf.transform.rotation.y, tf.transform.rotation.z);
        Eigen::Vector3f t(
            tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z);
        aux.T_primary_aux.setIdentity();
        aux.T_primary_aux.block<3, 3>(0, 0) = q.toRotationMatrix();
        aux.T_primary_aux.block<3, 1>(0, 3) = t;
        aux.extrinsic_cached = true;
        RCLCPP_INFO(this->get_logger(),
                    "lidar_concat: cached T(%s <- %s): t=[%.3f, %.3f, %.3f]",
                    primary_frame.c_str(), aux.frame.c_str(), t.x(), t.y(), t.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "lidar_concat: waiting for TF '%s' -> '%s': %s",
                             primary_frame.c_str(), aux.frame.c_str(), ex.what());
        continue;
      }
    }

    // Pick the aux scan closest to the primary in capture time. For Luminar,
    // header.stamp can be sensor-specific (right is delayed by ~100 ms in the
    // validation bags), so use the per-point timestamp midpoint when available.
    // Other sensors keep the legacy header-nearest behavior.
    sensor_msgs::msg::PointCloud2::ConstSharedPtr match;
    double best_dt = std::numeric_limits<double>::max();
    {
      std::lock_guard<std::mutex> lk(aux.mtx);
      for (const auto& msg : aux.buffer) {
        double aux_time_key = rclcpp::Time(msg->header.stamp).seconds();
        if (use_luminar_time_key) {
          double aux_midpoint_sec = 0.0;
          if (!luminarCloudTimeMidpointSec(*msg, aux_midpoint_sec)) {
            continue;
          }
          aux_time_key = aux_midpoint_sec;
        }
        const double dt = std::abs(aux_time_key - primary_time_key);
        if (dt < best_dt) {
          best_dt = dt;
          match = msg;
        }
      }
    }
    if (!match || best_dt > this->concat_time_threshold_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "lidar_concat: no match for '%s' within %.3fs of primary time key=%.3f (best_dt=%.3fs)",
                   aux.topic.c_str(), this->concat_time_threshold_, primary_time_key,
                   match ? best_dt : -1.0);
      continue;
    }
    if (match->point_step != point_step) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — point_step mismatch (%u vs primary %u)",
                           aux.topic.c_str(), match->point_step, point_step);
      continue;
    }

    // Append aux bytes directly into merged->data, then transform xyz + shift
    // timestamps in place over the just-appended region. No intermediate copy.
    // If validation fails after the append, roll back the resize so a malformed
    // aux scan can't leak into the merged cloud in its own (un-transformed) frame.
    const size_t old_size = merged->data.size();
    merged->data.insert(merged->data.end(), match->data.begin(), match->data.end());
    uint8_t* appended = merged->data.data() + old_size;
    const size_t aux_pts = match->data.size() / point_step;

    int ax, ay, az;
    if (!findXYZOffsets(*match, ax, ay, az)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — no x/y/z fields in PointCloud2",
                           aux.topic.c_str());
      merged->data.resize(old_size);
      continue;
    }
    transformCloudData(appended, aux_pts, point_step, ax, ay, az, aux.T_primary_aux);

    int time_off;
    uint8_t time_dt_type;
    int time_count;
    const bool has_time_field = findTimeField(*match, time_off, time_dt_type, time_count);
    if (has_time_field) {
      // dt = aux header - primary header. Adding dt rebases relative aux
      // per-point times onto the primary clock. Luminar UINT8[8] timestamps
      // are already absolute epoch ns and must remain untouched.
      const double dt = rclcpp::Time(match->header.stamp).seconds() - t_primary;
      const bool luminar_u64 = (this->sensor == dlio::SensorType::LUMINAR);
      shiftCloudTimestamps(appended, aux_pts, point_step, time_off, time_dt_type, time_count, dt, luminar_u64);
    } else if (this->deskew_) {
      // Without per-point timestamps the aux rays would deskew against the
      // primary scan's IMU integration with a stale (aux-header) reference,
      // smearing them. Drop the aux when deskew is enabled and times are absent.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — deskew enabled but no time field found",
                           aux.topic.c_str());
      merged->data.resize(old_size);
      continue;
    }

    total_points += static_cast<size_t>(match->width) * match->height;
    ++merged_aux_count;
  }

  // The merged cloud is unorganized (height=1); width = total appended points.
  merged->width = static_cast<uint32_t>(total_points);
  merged->height = 1;
  merged->is_dense = false;
  merged->row_step = point_step * static_cast<uint32_t>(total_points);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "lidar_concat: merged %zu/%zu aux scans, total %zu points",
                       merged_aux_count, this->aux_lidars_.size(), total_points);

  return merged;
}

void gicp_localization::LocalizationNode::deskewPointcloud() {

  if (!this->deskew_ || !this->first_imu_received) {
    this->current_scan = this->original_scan;

    // Even without per-point deskewing, integrate IMU to get a motion-predicted
    // T_prior so GICP starts from an IMU-advanced pose instead of the stale last result.
    if (this->first_imu_received && this->prev_scan_stamp > 0.0) {
      std::vector<double> single_ts = {this->scan_stamp.seconds()};
      {
        double latest_imu = 0.0;
        {
          std::lock_guard<std::mutex> lock(this->mtx_imu);
          if (!this->imu_buffer.empty()) latest_imu = this->imu_buffer.front().stamp;
        }
        if (latest_imu > 0.0 && single_ts[0] > latest_imu)
          single_ts[0] = latest_imu;
      }
      auto frames = this->integrateImu(this->prev_scan_stamp, this->basePose.q,
                                        this->basePose.p, this->prev_vel, single_ts);
      if (frames.size() == 1 && matrixFinite(frames[0])) {
        this->T_prior = frames[0];
      } else {
        this->T_prior = this->current_pose;
      }
    } else {
      this->T_prior = this->current_pose;
    }

    this->prev_scan_stamp = this->scan_stamp.seconds();
    return;
  }

  pcl::PointCloud<PointType>::Ptr deskewed_scan_ =
      std::make_shared<pcl::PointCloud<PointType>>(1, this->original_scan->points.size());

  // Individual point timestamps should be relative to this time
  double sweep_ref_time = this->scan_stamp.seconds();

  // Sort points by timestamp and build list of timestamps
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<double(const PointType&)> extract_point_time_from_point;
  bool deskew_time_ready = false;

  if (this->sensor == dlio::SensorType::OUSTER) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.t < p2.t; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) {
      return sweep_ref_time + static_cast<double>(pt.t) * 1e-9;
    };
    deskew_time_ready = true;
  } else if (this->sensor == dlio::SensorType::VELODYNE) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.time < p2.time; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) {
      return sweep_ref_time + static_cast<double>(pt.time);
    };
    deskew_time_ready = true;
  } else if (this->sensor == dlio::SensorType::HESAI) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.timestamp < p2.timestamp; };
    extract_point_time_from_point = [](const PointType& pt) { return pt.timestamp; };
    deskew_time_ready = true;
  } else if (this->sensor == dlio::SensorType::LIVOX) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.timestamp < p2.timestamp; };
    extract_point_time_from_point = [](const PointType& pt) { return pt.timestamp * 1e-9; };
    deskew_time_ready = true;
  } else if (this->sensor == dlio::SensorType::LUMINAR) {
    // Luminar UINT8[8] timestamps are validated as absolute epoch
    // nanoseconds. GICP deskew only needs intra-scan offsets; subtracting the
    // scan minimum keeps the absolute epoch from leaking into the IMU query
    // while preserving the point timing span required by the validation plan.
    uint64_t min_ts = std::numeric_limits<uint64_t>::max();
    uint64_t max_ts = 0;
    for (const auto& pt : this->original_scan->points) {
      const uint64_t ts = luminarPointTimestampNs(pt);
      min_ts = std::min(min_ts, ts);
      max_ts = std::max(max_ts, ts);
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Luminar scan: min_ts=%lu max_ts=%lu ns, sweep_ref=%.3f s",
                         min_ts, max_ts, sweep_ref_time);
    point_time_cmp = [](const PointType& p1, const PointType& p2) {
      return luminarPointTimestampNs(p1) < luminarPointTimestampNs(p2);
    };
    extract_point_time_from_point = [sweep_ref_time, min_ts](const PointType& pt) {
      return sweep_ref_time + static_cast<double>(luminarPointTimestampNs(pt) - min_ts) * 1e-9;
    };
    deskew_time_ready = true;
  }

  if (!deskew_time_ready) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unsupported localization/sensor_type for deskew; using scan without motion "
                         "compensation");
    this->current_scan = this->original_scan;
    this->prev_scan_stamp = this->scan_stamp.seconds();
    return;
  }

  // Copy points into deskewed_scan_ in order of timestamp
  std::partial_sort_copy(this->original_scan->points.begin(), this->original_scan->points.end(),
                         deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

  // Extract timestamps from points and build list of unique timestamps
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;

  double prev_timestamp = -1.0;
  for (size_t i = 0; i < deskewed_scan_->points.size(); i++) {
    double curr_timestamp = extract_point_time_from_point(deskewed_scan_->points[i]);
    if (std::abs(curr_timestamp - prev_timestamp) > 1e-9) {  // Unique timestamp
      timestamps.push_back(curr_timestamp);
      unique_time_indices.push_back(i);
      prev_timestamp = curr_timestamp;
    }
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  if (this->sensor == dlio::SensorType::LUMINAR && this->verbose_ && !deskewed_scan_->points.empty()) {
    logLuminarTimestampStats(deskewed_scan_->points.size(), *deskewed_scan_, timestamps.size());
  }

  if (timestamps.empty()) {
    RCLCPP_WARN(this->get_logger(), "No timestamps extracted from point cloud, skipping deskewing");
    this->current_scan = this->original_scan;
    return;
  }

  int median_pt_index = timestamps.size() / 2;

  // Don't process scans on first iteration
  if (this->prev_scan_stamp == 0.0) {
    this->prev_scan_stamp = this->scan_stamp.seconds();
    this->T_prior = this->current_pose;
    this->current_scan = deskewed_scan_;
    return;
  }

  // Check if we have sufficient IMU history
  // We need IMU data from BEFORE prev_scan_stamp to integrate
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (this->imu_buffer.empty()) {
      RCLCPP_WARN(this->get_logger(), "IMU buffer is empty, skipping deskewing");
      this->current_scan = this->original_scan;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return;
    }

    // Check if oldest IMU is before prev_scan_stamp (need some margin)
    double oldest_imu_time = this->imu_buffer.back().stamp;
    double margin = 0.1;  // 100ms margin

    if (oldest_imu_time > this->prev_scan_stamp - margin) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for sufficient IMU history (oldest: %.3f, need: %.3f). Skipping deskewing.",
                           oldest_imu_time, this->prev_scan_stamp);
      this->T_prior = this->current_pose;
      this->current_scan = deskewed_scan_;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return;
    }
  }

  // IMU prior & deskewing
  RCLCPP_DEBUG(this->get_logger(),
               "Integrating IMU: prev_stamp=%.3f, pos=[%.2f,%.2f,%.2f], vel=[%.2f,%.2f,%.2f]",
               this->prev_scan_stamp,
               this->basePose.p.x(), this->basePose.p.y(), this->basePose.p.z(),
               this->prev_vel.x(), this->prev_vel.y(), this->prev_vel.z());

  // Clamp timestamps to IMU buffer extent. The latest point can be a few ms
  // ahead of the latest IMU sample due to callback ordering; clamping avoids
  // rejecting the integration.
  {
    double latest_imu = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->mtx_imu);
      if (!this->imu_buffer.empty()) latest_imu = this->imu_buffer.front().stamp;
    }
    if (latest_imu > 0.0) {
      for (auto& ts : timestamps) {
        if (ts > latest_imu) ts = latest_imu;
      }
    }
  }

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->basePose.q, this->basePose.p,
                              this->prev_vel, timestamps);

  // If there are no frames between the start and end of the sweep, use previous transform
  if (frames.size() != timestamps.size()) {
    RCLCPP_WARN(this->get_logger(),
                "IMU integration failed! Got %lu frames for %lu timestamps. "
                "Time range: [%.3f, %.3f], IMU buffer size: %lu, first IMU: %.3f",
                frames.size(), timestamps.size(),
                this->prev_scan_stamp, timestamps.back(),
                this->imu_buffer.size(),
                this->imu_buffer.empty() ? 0.0 : this->imu_buffer.back().stamp);
    this->T_prior = this->current_pose;
    this->current_scan = deskewed_scan_;
    this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Deskewing OK: %lu frames, scan time [%.3f, %.3f]",
                       frames.size(), timestamps.front(), timestamps.back());

  // Update prior to be the estimated pose at the median time of the scan
  this->T_prior = frames[median_pt_index];
  const Eigen::Matrix4f T_ref_lidar = this->T_prior * this->extrinsics.baselink2lidar_T;
  const Eigen::Matrix4f T_ref_lidar_inv = T_ref_lidar.inverse();

  // Deskew each point into the LiDAR frame at the reference (median) time.
  // Keeping the source cloud in a local LiDAR frame lets GICP solve the
  // absolute map pose from the IMU prior instead of baking IMU translation
  // drift directly into world-frame point coordinates.
  #pragma omp parallel for
  for (size_t i = 0; i < timestamps.size(); i++) {
    Eigen::Matrix4f T = T_ref_lidar_inv * frames[i] * this->extrinsics.baselink2lidar_T;

    // Transform point to the reference LiDAR frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
      auto &pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }

  this->current_scan = deskewed_scan_;
  this->prev_scan_stamp = this->scan_stamp.seconds();
}

void gicp_localization::LocalizationNode::preprocessPointCloud(pcl::PointCloud<PointType>::Ptr& cloud) {

  size_t original_size = cloud->points.size();

  // Crop box filter
  if (this->crop_size_ > 0.0 && this->crop_size_ < 1000.0) {  // Only apply if reasonable size
    pcl::CropBox<PointType> crop;
    crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
    crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));
    crop.setInputCloud(cloud);
    crop.filter(*cloud);
    RCLCPP_DEBUG(this->get_logger(), "Crop box filter: %lu -> %lu points", original_size, cloud->points.size());
  }

  // Voxel filter
  if (this->vf_use_) {
    size_t before_voxel = cloud->points.size();
    pcl::PointCloud<PointType> filtered;
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
    voxel.setInputCloud(cloud);
    voxel.filter(filtered);
    if (filtered.points.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Voxel filter removed ALL %lu input points (res=%.3f m) — "
                  "check point coordinates and frame; keeping unfiltered cloud for this scan",
                  before_voxel, this->vf_res_);
      // Leave cloud unchanged — no copy needed.
    } else {
      *cloud = std::move(filtered);
      const double reduction = 1.0 - static_cast<double>(cloud->points.size()) /
                                         static_cast<double>(before_voxel);
      if (reduction > 0.99) {
        RCLCPP_WARN(this->get_logger(),
                    "Voxel filter removed %.1f%% of points (%lu -> %lu, res=%.3f m) — "
                    "check point frame/coordinates",
                    reduction * 100.0, before_voxel, cloud->points.size(), this->vf_res_);
      }
      RCLCPP_DEBUG(this->get_logger(), "Voxel filter: %lu -> %lu points", before_voxel, cloud->points.size());
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Preprocessing: %lu -> %lu points total", original_size, cloud->points.size());
}

void gicp_localization::LocalizationNode::performLocalization() {

  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Acquiring mutex lock...");
  std::lock_guard<std::mutex> lock(this->pose_mutex);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Mutex acquired");

  // Set source cloud
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Setting input source (%lu points)...",
               this->current_scan->points.size());
  this->gicp.setInputSource(this->current_scan);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Input source set");

  // Align using IMU-based prior as initial guess (if deskewing is enabled)
  // align() requires an output cloud parameter (PCL API), but we never use the
  // transformed cloud — LsqRegistration skips the fill, so this stays empty.
  pcl::PointCloud<PointType> aligned_scratch;

  // The source cloud is in a LiDAR frame for both modes. Without deskew it is
  // the raw scan frame; with deskew it is the median-time reference LiDAR frame.
  // Seed/solve in map<-lidar, then convert the optimizer output back to
  // map<-base.
  const Eigen::Matrix4f T_base_lidar = this->extrinsics.baselink2lidar_T;
  const Eigen::Matrix4f T_lidar_base = T_base_lidar.inverse();
  Eigen::Matrix4f initial_guess = this->T_prior * T_base_lidar;
  Eigen::Matrix4f guess_pose_map = this->T_prior;

  double guess_from_last_trans = 0.0;
  double guess_from_last_rot_deg = 0.0;
  if (this->last_gicp_valid_) {
    guess_from_last_trans = deltaTranslationNorm(this->last_gicp_pose_, guess_pose_map);
    guess_from_last_rot_deg = rotationDistanceDeg(this->last_gicp_pose_, guess_pose_map);
  }

  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Starting GICP alignment...");
  auto start = std::chrono::high_resolution_clock::now();
  this->gicp.align(aligned_scratch, initial_guess);
  auto end = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: GICP alignment completed");

  double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

  double fitness_score = this->gicp.getFitnessScore();
  this->last_fitness_score_ = fitness_score;
  double final_error = this->gicp.getFinalError();
  bool converged = this->gicp.hasConverged();
  const int num_correspondences = this->gicp.num_correspondences;
  const double correspondence_ratio =
      this->current_scan->points.empty()
          ? 0.0
          : static_cast<double>(num_correspondences) / static_cast<double>(this->current_scan->points.size());
  const Eigen::Matrix<double, 6, 6>& final_hessian = this->gicp.getFinalHessian();
  const double hessian_condition = hessianConditionProxy(final_hessian);

  const Eigen::Matrix4f optimizer_solution = this->gicp.getFinalTransformation();
  const Eigen::Matrix4f candidate_pose = optimizer_solution * T_lidar_base;
  const bool candidate_pose_valid = matrixFinite(candidate_pose);

  double guess_to_solution_trans = -1.0;
  double guess_to_solution_rot_deg = -1.0;
  if (candidate_pose_valid) {
    guess_to_solution_trans = deltaTranslationNorm(guess_pose_map, candidate_pose);
    guess_to_solution_rot_deg = rotationDistanceDeg(guess_pose_map, candidate_pose);
  }

  double scan_dt = 0.0;
  if (this->last_gicp_valid_) {
    scan_dt = (this->scan_stamp - this->last_gicp_stamp_).seconds();
  }

  double imu_buffer_span = -1.0;
  double scan_to_latest_imu_lag = -1.0;
  {
    std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      const double latest_imu_stamp = this->imu_buffer.front().stamp;
      const double oldest_imu_stamp = this->imu_buffer.back().stamp;
      imu_buffer_span = latest_imu_stamp - oldest_imu_stamp;
      scan_to_latest_imu_lag = this->scan_stamp.seconds() - latest_imu_stamp;
    }
  }

  // Jump is measured against the IMU-predicted prior, not the last GICP pose.
  // This asks "did GICP disagree with IMU?" (catastrophic failure indicator)
  // instead of "did the vehicle move far?" (expected at highway speed + time gaps).
  double jump_trans = -1.0;
  double jump_rot_deg = -1.0;
  if (candidate_pose_valid) {
    jump_trans = deltaTranslationNorm(this->T_prior, candidate_pose);
    jump_rot_deg = rotationDistanceDeg(this->T_prior, candidate_pose);
  }
  const bool large_jump = candidate_pose_valid &&
                          (jump_trans > this->debug_jump_trans_m_ || jump_rot_deg > this->debug_jump_rot_deg_);

  // Ground-truth divergence cross-check (optional). Compares the scan's accepted-or-candidate
  // pose to a time-matched ground-truth odom sample. Only computes; does NOT influence
  // accept/reject decisions — purely a diagnostic.
  double gt_pos_err = -1.0;
  double gt_rot_err_deg = -1.0;
  double gt_dt = 0.0;
  if (this->gt_odom_enabled_ && this->gt_odom_received_.load() && candidate_pose_valid) {
    GtSample gt;
    if (this->getGtPoseAt(this->scan_stamp.seconds(), gt)) {
      const Eigen::Vector3f cand_p = candidate_pose.block<3, 1>(0, 3);
      const Eigen::Quaternionf cand_q(Eigen::Matrix3f(candidate_pose.block<3, 3>(0, 0)));
      // Bring the GT sample from msg.child_frame_id (gt_body) into base_frame
      // using the same TF composition the snap helper uses. For the AV-24
      // single-source P1 config this is a no-op (identity TF, gt_body ==
      // base_frame == gps_antenna_top), but applying the composition explicitly
      // keeps the cross-check correct under any future gt_odom source change
      // (e.g. re-pointing the remap back at /localization/global/odom at cg).
      // Without this composition, the cross-check carries a constant baseline
      // bias equal to the gt_body -> base_frame lever arm.
      Eigen::Vector3f gt_p_in_base;
      Eigen::Quaternionf gt_q_in_base;
      if (!this->composeGtPoseInBase(gt, gt_p_in_base, gt_q_in_base)) {
        // Extrinsic not cached yet -- fall back to gt.p/gt.q directly.
        // Acceptable for early-startup diagnostic noise; the cache fills on
        // the first successfully-received GT message.
        gt_p_in_base = gt.p;
        gt_q_in_base = gt.q;
      }
      gt_pos_err = (cand_p - gt_p_in_base).norm();
      Eigen::Quaternionf dq = cand_q.normalized() * gt_q_in_base.normalized().conjugate();
      const double w = std::clamp(static_cast<double>(std::abs(dq.w())), 0.0, 1.0);
      gt_rot_err_deg = 2.0 * std::acos(w) * 180.0 / M_PI;
      gt_dt = this->scan_stamp.seconds() - gt.stamp;
    }
  }

  if (this->debug_pub_enabled_) {
    auto publish_float = [](const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub, double value) {
      std_msgs::msg::Float64 msg;
      msg.data = value;
      pub->publish(msg);
    };

    publish_float(this->dbg_fitness_pub, fitness_score);
    publish_float(this->dbg_gicp_elapsed_ms_pub, elapsed_ms);
    publish_float(this->dbg_corr_norm_pub, guess_to_solution_trans);
    publish_float(this->dbg_scan_dt_pub, scan_dt);
    publish_float(this->dbg_imu_age_pub, imu_buffer_span);
    publish_float(this->dbg_num_correspondences_pub, static_cast<double>(num_correspondences));
    publish_float(this->dbg_correspondence_ratio_pub, correspondence_ratio);
    publish_float(this->dbg_final_error_pub, final_error);
    publish_float(this->dbg_guess_to_solution_trans_pub, guess_to_solution_trans);
    publish_float(this->dbg_guess_to_solution_rot_deg_pub, guess_to_solution_rot_deg);
    publish_float(this->dbg_guess_from_last_trans_pub, guess_from_last_trans);
    publish_float(this->dbg_guess_from_last_rot_deg_pub, guess_from_last_rot_deg);
    publish_float(this->dbg_raw_points_pub, static_cast<double>(this->last_raw_point_count_));
    publish_float(this->dbg_preprocessed_points_pub, static_cast<double>(this->last_preprocessed_point_count_));
    publish_float(this->dbg_imu_buffer_span_pub, imu_buffer_span);
    publish_float(this->dbg_scan_to_latest_imu_lag_pub, scan_to_latest_imu_lag);
    publish_float(this->dbg_hessian_condition_pub, hessian_condition);
    publish_float(this->dbg_jump_trans_pub, jump_trans);
    publish_float(this->dbg_jump_rot_deg_pub, jump_rot_deg);

    std_msgs::msg::Bool converged_msg;
    converged_msg.data = (converged || (candidate_pose_valid && fitness_score <= this->gicp_fitness_reject_threshold_)) && candidate_pose_valid;
    this->dbg_converged_pub->publish(converged_msg);

    if (gt_pos_err >= 0.0) {
      publish_float(this->dbg_gt_pos_err_pub, gt_pos_err);
      publish_float(this->dbg_gt_rot_deg_pub, gt_rot_err_deg);
    }

    this->dbg_initial_guess_pose_pub->publish(
        poseStampedFromMatrix(guess_pose_map, this->scan_stamp, this->map_frame));
    if (candidate_pose_valid) {
      this->dbg_final_pose_pub->publish(
          poseStampedFromMatrix(candidate_pose, this->scan_stamp, this->map_frame));
    }

    if (this->dbg_pose_markers_pub->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray markers;

      visualization_msgs::msg::Marker clear_marker;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear_marker);

      markers.markers.push_back(
          makeArrowMarker(guess_pose_map, this->map_frame, this->scan_stamp, 0, "gicp_debug_guess", 1.0f, 0.55f, 0.0f));

      if (candidate_pose_valid) {
        markers.markers.push_back(
            makeArrowMarker(candidate_pose, this->map_frame, this->scan_stamp, 1, "gicp_debug_solution", 0.0f, 0.9f, 0.2f));

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.stamp = this->scan_stamp;
        line_marker.header.frame_id = this->map_frame;
        line_marker.ns = "gicp_debug_delta";
        line_marker.id = 2;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.12;
        line_marker.color.a = 1.0f;
        line_marker.color.r = 1.0f;
        line_marker.color.g = 1.0f;
        line_marker.color.b = 0.0f;

        geometry_msgs::msg::Point guess_point;
        guess_point.x = guess_pose_map(0, 3);
        guess_point.y = guess_pose_map(1, 3);
        guess_point.z = guess_pose_map(2, 3);
        line_marker.points.push_back(guess_point);

        geometry_msgs::msg::Point final_point;
        final_point.x = candidate_pose(0, 3);
        final_point.y = candidate_pose(1, 3);
        final_point.z = candidate_pose(2, 3);
        line_marker.points.push_back(final_point);
        markers.markers.push_back(line_marker);
      }

      this->dbg_pose_markers_pub->publish(markers);
    }
  }

  auto build_scan_debug_log = [&](const char* status) {
    std::ostringstream oss;
    oss << std::fixed
        << "SCAN DEBUG | status=" << status
        << " stamp=" << std::setprecision(3) << this->scan_stamp.seconds()
        << " input_frame=" << this->last_scan_input_frame_
        << " raw=" << this->last_raw_point_count_
        << " pre=" << this->last_preprocessed_point_count_
        << " guess={" << poseSummary(guess_pose_map) << "}"
        << " guess_from_last=[" << scalarSummary(guess_from_last_trans) << "m,"
        << scalarSummary(guess_from_last_rot_deg) << "deg]"
        << " gicp_ms=" << scalarSummary(elapsed_ms, 2)
        << " converged=" << (converged ? "true" : "false")
        << " fitness=" << scalarSummary(fitness_score, 6)
        << " final_error=" << scalarSummary(final_error, 6)
        << " correspondences=" << num_correspondences << "/" << this->current_scan->points.size()
        << " ratio=" << scalarSummary(correspondence_ratio, 3)
        << " guess_to_solution=[" << scalarSummary(guess_to_solution_trans) << "m,"
        << scalarSummary(guess_to_solution_rot_deg) << "deg]"
        << " jump=[" << scalarSummary(jump_trans) << "m," << scalarSummary(jump_rot_deg) << "deg]"
        << " imu_buffer_span=" << scalarSummary(imu_buffer_span) << "s"
        << " scan_to_latest_imu_lag=" << scalarSummary(scan_to_latest_imu_lag) << "s"
        << " hessian_cond=" << scalarSummary(hessian_condition, 3)
        << " candidate={" << poseSummary(candidate_pose) << "}";

    if (this->last_gicp_valid_) {
      oss << " last_good={" << poseSummary(this->last_gicp_pose_) << "}";
    }
    if (this->gt_odom_enabled_) {
      if (gt_pos_err >= 0.0) {
        oss << " gt_err=[" << scalarSummary(gt_pos_err, 3) << "m,"
            << scalarSummary(gt_rot_err_deg, 2) << "deg,dt="
            << scalarSummary(gt_dt, 3) << "s]";
      } else {
        oss << " gt_err=unavailable";
      }
    }
    return oss.str();
  };

  // Accept results that have good fitness even when the optimizer didn't formally
  // converge (hit maxIterations before epsilon was met). At highway speed the
  // initial guess can be 1-3 m away, so the solver may need more steps than
  // maxIterations to satisfy the tight epsilon — but the result is still accurate.
  const bool effectively_converged = converged ||
      (candidate_pose_valid && fitness_score <= this->gicp_fitness_reject_threshold_);

  bool gicp_rejected_fitness = false;
  bool gicp_rejected_jump = false;
  bool gicp_rejected_hessian = false;
  bool gicp_rejected_gt = false;
  if (effectively_converged && candidate_pose_valid) {
    if (fitness_score > this->gicp_fitness_reject_threshold_) {
      gicp_rejected_fitness = true;
    } else if (this->gicp_hessian_cond_max_ > 0.0 &&
               std::isfinite(hessian_condition) &&
               hessian_condition > this->gicp_hessian_cond_max_ &&
               ((this->gicp_hessian_fitness_warn_ > 0.0 &&
                 fitness_score > this->gicp_hessian_fitness_warn_) ||
                (this->gicp_hessian_trans_warn_m_ > 0.0 &&
                 guess_to_solution_trans > this->gicp_hessian_trans_warn_m_) ||
                (this->gicp_hessian_rot_warn_deg_ > 0.0 &&
                 guess_to_solution_rot_deg > this->gicp_hessian_rot_warn_deg_) ||
                (this->gicp_hessian_fitness_warn_ <= 0.0 &&
                 this->gicp_hessian_trans_warn_m_ <= 0.0 &&
                 this->gicp_hessian_rot_warn_deg_ <= 0.0))) {
      // Combined geometric degeneracy gate. High hessian condition alone is
      // harmless when the IMU prior was already good and GICP barely moved.
      // Reject only when geometry is degenerate AND any of these slide signals
      // fire: elevated fitness (wrong basin), large translation correction,
      // or large rotation correction. In degenerate geometry the optimizer can
      // slide along the unconstrained axis; the magnitude of that slide is
      // exactly the discrepancy between IMU prior and GICP candidate.
      // (All warns disabled = legacy "hessian alone" behavior.)
      gicp_rejected_hessian = true;
    } else if (this->gicp_reject_large_jumps_ && large_jump) {
      gicp_rejected_jump = true;
    } else if (this->gt_rejection_enabled_ && gt_pos_err >= 0.0 &&
               ((this->gt_rejection_max_pos_err_m_ > 0.0 &&
                 gt_pos_err > this->gt_rejection_max_pos_err_m_) ||
                (this->gt_rejection_max_rot_err_deg_ > 0.0 &&
                 gt_rot_err_deg > this->gt_rejection_max_rot_err_deg_))) {
      gicp_rejected_gt = true;
    }
  }
  const bool gicp_accepted = effectively_converged && candidate_pose_valid &&
                             !gicp_rejected_fitness && !gicp_rejected_hessian &&
                             !gicp_rejected_jump && !gicp_rejected_gt;

  if (!candidate_pose_valid) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("invalid_solution").c_str());
  } else if (!effectively_converged) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("failed_to_converge").c_str());
  } else if (gicp_rejected_fitness) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (fitness=%.4f > threshold=%.4f): %s",
                fitness_score, this->gicp_fitness_reject_threshold_,
                build_scan_debug_log("rejected_fitness").c_str());
  } else if (gicp_rejected_hessian) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (hessian_cond=%.3e > %.3e AND [fitness=%.4f|trans=%.3fm|rot=%.3fdeg] crossed [%.4f|%.3fm|%.3fdeg] — degenerate slide): %s",
                hessian_condition, this->gicp_hessian_cond_max_,
                fitness_score, guess_to_solution_trans, guess_to_solution_rot_deg,
                this->gicp_hessian_fitness_warn_,
                this->gicp_hessian_trans_warn_m_,
                this->gicp_hessian_rot_warn_deg_,
                build_scan_debug_log("rejected_hessian").c_str());
  } else if (gicp_rejected_jump) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (jump dT=%.3fm dR=%.2fdeg): %s",
                jump_trans, jump_rot_deg, build_scan_debug_log("rejected_jump").c_str());
  } else if (gicp_rejected_gt) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (GT divergence pos=%.3fm rot=%.2fdeg exceeds %.3fm/%.2fdeg): %s",
                gt_pos_err, gt_rot_err_deg,
                this->gt_rejection_max_pos_err_m_,
                this->gt_rejection_max_rot_err_deg_,
                build_scan_debug_log("rejected_gt").c_str());
  } else if (large_jump) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("large_jump").c_str());
  } else if (this->debug_verbose_scan_log_) {
    RCLCPP_INFO(this->get_logger(), "%s", build_scan_debug_log("ok").c_str());
  }

  if (gicp_accepted) {
    this->current_pose = candidate_pose;

    // Update lidar pose for next iteration
    Eigen::Vector3f new_p = this->current_pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotSO3 = this->current_pose.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotSO3);
    q.normalize();

    this->basePose.p = new_p;
    this->basePose.q = q;

    // Validate GICP result before using it
    bool gicp_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) && std::isfinite(new_p.z()) &&
                      std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());

    if (!gicp_valid) {
      RCLCPP_WARN(this->get_logger(), "GICP result contains invalid values, skipping geometric observer update");
    } else {
      // Initialize or update geometric observer
      if (!this->geo.first_opt_done) {
        // First time: initialize state to GICP result
        this->state.p = new_p;
        this->state.q = q;
        this->state.v.lin.w = Eigen::Vector3f::Zero();
        this->state.v.lin.b = Eigen::Vector3f::Zero();
        this->state.v.ang.w = Eigen::Vector3f::Zero();
        this->state.v.ang.b = Eigen::Vector3f::Zero();
        this->state.b.accel = Eigen::Vector3f::Zero();
        this->state.b.gyro = Eigen::Vector3f::Zero();

        // Initialize geo tracking
        this->geo.prev_p = this->state.p;
        this->geo.prev_q = this->state.q;
        this->geo.prev_vel = Eigen::Vector3f::Zero();

        // Mark as initialized
        this->geo.first_opt_done = true;

        RCLCPP_INFO(this->get_logger(), "Geometric observer initialized to pos=[%.2f,%.2f,%.2f]",
                    new_p.x(), new_p.y(), new_p.z());
      } else {
        // Update geometric observer with GICP measurement (skip on first scan)
        this->updateState();
      }
    }

    // Use geometric observer velocity for next IMU integration
    {
      std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
      this->prev_vel = this->geo.prev_vel;
    }

    if (this->debug_jump_log_enabled_ && gicp_valid && this->last_gicp_valid_) {
      if (large_jump) {
        const Eigen::Vector3f t_prior = this->T_prior.block<3, 1>(0, 3);
        const Eigen::Vector3f t_corr = optimizer_solution.block<3, 1>(0, 3);
        RCLCPP_WARN(this->get_logger(),
                    "JUMP DETECTED: dT=%.3fm dR=%.2fdeg | dt=%.3fs fitness=%.6f | prior=[%.2f,%.2f,%.2f] corr=[%.2f,%.2f,%.2f]",
                    jump_trans, jump_rot_deg, scan_dt, fitness_score,
                    t_prior.x(), t_prior.y(), t_prior.z(),
                    t_corr.x(), t_corr.y(), t_corr.z());
      }
    }

    // Update last GICP pose after computing jump metrics
    if (gicp_valid) {
      this->last_gicp_pose_ = this->current_pose;
      this->last_gicp_stamp_ = this->scan_stamp;
      this->last_gicp_valid_ = true;
    }

    // Reset consecutive-failure counter on any accepted scan so the GT-recovery
    // trigger only fires on sustained losing streaks.
    this->consecutive_failures_ = 0;

    // Log pose and correction
    Eigen::Vector3f t_corr = optimizer_solution.block<3, 1>(0, 3);
    RCLCPP_INFO(this->get_logger(),
                "Localization: ✓ %s | fitness=%.6f | time=%.2fms | "
                "correction=[%.3f, %.3f, %.3f] | pose=[%.2f, %.2f, %.2f]",
                converged ? "CONVERGED" : "ACCEPTED(fitness-ok)",
                fitness_score, elapsed_ms,
                t_corr.x(), t_corr.y(), t_corr.z(),
                this->basePose.p.x(), this->basePose.p.y(), this->basePose.p.z());
  } else {
    // Any non-accepted scan (failed_to_converge, rejected_fitness, rejected_jump, invalid_solution)
    // falls back to the IMU-integrated prior. Freezing at last_gicp_pose_ causes cascade
    // divergence at feature-poor corners: each subsequent scan's guess drifts further from
    // reality, fitness gets worse, and the optimizer never recovers.
    ++this->consecutive_failures_;
    const char* reason = !candidate_pose_valid ? "invalid solution"
                       : !effectively_converged ? "failed to converge"
                       : gicp_rejected_fitness ? "fitness rejected"
                       : gicp_rejected_hessian ? "degenerate geometry"
                       : gicp_rejected_gt ? "GT divergence rejected"
                       : "jump rejected";
    if (matrixFinite(this->T_prior)) {
      this->current_pose = this->T_prior;
      const Eigen::Vector3f new_p = this->T_prior.block<3, 1>(0, 3);
      Eigen::Quaternionf q(this->T_prior.block<3, 3>(0, 0));
      q.normalize();
      this->basePose.p = new_p;
      this->basePose.q = q;
      {
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->prev_vel = this->geo.prev_vel;
      }
      RCLCPP_WARN(this->get_logger(),
                  "Localization: ⚠ GICP %s — holding IMU dead-reckoning pose [%.2f, %.2f, %.2f] | fitness=%.4f time=%.2fms",
                  reason, new_p.x(), new_p.y(), new_p.z(), fitness_score, elapsed_ms);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Localization: ⚠ GICP %s — holding last accepted pose (no valid T_prior) | fitness=%.4f time=%.2fms",
                  reason, fitness_score, elapsed_ms);
    }

    // GT-driven pose recovery: if enabled and the failure streak has hit the
    // configured threshold, snap state.{pose,velocity} to the time-matched GT
    // sample (transformed into base_frame). The snap overrides the dead-reckoned
    // pose and resets the counter; logs its own warn line.
    this->maybeSnapPoseToGT(reason);
  }
}

void gicp_localization::LocalizationNode::publishPose() {

  std::lock_guard<std::mutex> lock(this->pose_mutex);

  // Extract position and orientation from localized pose
  Eigen::Vector3f position = this->current_pose.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation = this->current_pose.block<3, 3>(0, 0);
  Eigen::Quaternionf orientation(rotation);
  orientation.normalize();

  // Publish PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->scan_stamp;
  pose_msg.header.frame_id = this->map_frame;
  pose_msg.pose.position.x = position.x();
  pose_msg.pose.position.y = position.y();
  pose_msg.pose.position.z = position.z();
  pose_msg.pose.orientation.w = orientation.w();
  pose_msg.pose.orientation.x = orientation.x();
  pose_msg.pose.orientation.y = orientation.y();
  pose_msg.pose.orientation.z = orientation.z();

  // Publish GICP-corrected pose
  // With unreliable IMU, we publish only GICP results instead of propagated poses
  this->pose_pub->publish(pose_msg);

  // Add to trajectory deque (O(1) pop_front when capping). Only build the Path
  // message + DDS-publish when a subscriber actually exists.
  if (this->path_buffer_.size() >= 10000) this->path_buffer_.pop_front();
  this->path_buffer_.push_back(pose_msg);
  if (this->path_pub && this->path_pub->get_subscription_count() > 0) {
    this->path_msg.header.stamp = this->scan_stamp;
    this->path_msg.header.frame_id = this->map_frame;
    this->path_msg.poses.assign(this->path_buffer_.begin(), this->path_buffer_.end());
    this->path_pub->publish(this->path_msg);
  }

  // Publish UTM-frame pose/path
  if (this->utm_enabled_) {
    Eigen::Matrix4f T_utm_base = this->T_utm_map_ * this->current_pose;
    Eigen::Vector3f utm_pos = T_utm_base.block<3, 1>(0, 3);
    Eigen::Quaternionf utm_q(T_utm_base.block<3, 3>(0, 0));
    utm_q.normalize();

    geometry_msgs::msg::PoseStamped utm_pose_msg;
    utm_pose_msg.header.stamp = this->scan_stamp;
    utm_pose_msg.header.frame_id = this->utm_frame;
    utm_pose_msg.pose.position.x = utm_pos.x();
    utm_pose_msg.pose.position.y = utm_pos.y();
    utm_pose_msg.pose.position.z = utm_pos.z();
    utm_pose_msg.pose.orientation.w = utm_q.w();
    utm_pose_msg.pose.orientation.x = utm_q.x();
    utm_pose_msg.pose.orientation.y = utm_q.y();
    utm_pose_msg.pose.orientation.z = utm_q.z();
    this->utm_pose_pub->publish(utm_pose_msg);

    if (this->utm_path_buffer_.size() >= 10000) this->utm_path_buffer_.pop_front();
    this->utm_path_buffer_.push_back(utm_pose_msg);
    if (this->utm_path_pub && this->utm_path_pub->get_subscription_count() > 0) {
      this->utm_path_msg_.header.stamp = this->scan_stamp;
      this->utm_path_msg_.header.frame_id = this->utm_frame;
      this->utm_path_msg_.poses.assign(this->utm_path_buffer_.begin(), this->utm_path_buffer_.end());
      this->utm_path_pub->publish(this->utm_path_msg_);
    }
  }

  // Publish TF
  if (this->publish_tf_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = pose_msg.header;
    transform_stamped.child_frame_id = this->base_frame;
    transform_stamped.transform.translation.x = position.x();
    transform_stamped.transform.translation.y = position.y();
    transform_stamped.transform.translation.z = position.z();
    transform_stamped.transform.rotation = pose_msg.pose.orientation;
    this->tf_broadcaster->sendTransform(transform_stamped);
  }
}

void gicp_localization::LocalizationNode::callbackGtOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  GtSample s;
  s.stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  s.p = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  s.q = Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  s.q.normalize();
  s.v_lin_body = Eigen::Vector3f(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y,
                                 msg->twist.twist.linear.z);
  s.v_ang_body = Eigen::Vector3f(msg->twist.twist.angular.x,
                                 msg->twist.twist.angular.y,
                                 msg->twist.twist.angular.z);

  // RTK quality gate (P1-native): inspect Atlas-reported pose covariance on
  // the gt_odom message itself. Drops the entire sample -- no buffer push,
  // no initial-pose seed, no snap, no cross-check. Runs BEFORE TF caching so
  // a degraded GNSS sample can't even seed the cache state. Disabled gate is
  // a pass-through.
  //
  // pose.covariance is laid out as a row-major 6x6 (x,y,z,roll,pitch,yaw):
  //   [0]=cov_xx  [7]=cov_yy  [14]=cov_zz  (position variances in m^2)
  // Reject if any horizontal variance exceeds max_pose_var_xy OR vertical
  // variance exceeds max_pose_var_z. Atlas keeps these in the ~5e-5 m^2 band
  // when RTK-fixed and orders of magnitude higher when degraded.
  if (this->rtk_gate_enabled_) {
    const double cov_xx = msg->pose.covariance[0];
    const double cov_yy = msg->pose.covariance[7];
    const double cov_zz = msg->pose.covariance[14];
    const bool xy_bad = (cov_xx > this->rtk_gate_max_pose_var_xy_) ||
                        (cov_yy > this->rtk_gate_max_pose_var_xy_);
    const bool z_bad  = (cov_zz > this->rtk_gate_max_pose_var_z_);
    if (xy_bad || z_bad) {
      this->rtk_rejected_covariance_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "RTK gate: dropping gt_odom -- pose covariance "
                           "exceeds threshold "
                           "(cov_xx=%.4g cov_yy=%.4g cov_zz=%.4g; "
                           "max_xy=%.4g max_z=%.4g). Rejected total=%lu.",
                           cov_xx, cov_yy, cov_zz,
                           this->rtk_gate_max_pose_var_xy_,
                           this->rtk_gate_max_pose_var_z_,
                           this->rtk_rejected_covariance_.load());
      return;
    }
  }

  // Cache base_frame ← gt_body_frame TF on the first message (mirrors the IMU
  // extrinsic caching pattern in callbackImu). Required before the snap helper
  // and the diagnostic cross-check can compose poses; callback keeps appending
  // samples even while TF is missing.  Runs BEFORE the odom-init block below
  // so composeGtPoseInBase has the extrinsic ready to bring the first GT
  // sample into base_frame coordinates before applyInitialPose seeds the state.
  if (!this->gt_extrinsics_cached_) {
    if (this->gt_body_frame_.empty()) {
      this->gt_body_frame_ = msg->child_frame_id;
      if (this->gt_body_frame_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GT odom message has empty child_frame_id; assuming GT body == base_frame ('%s')",
                             this->base_frame.c_str());
        this->gt_body_frame_ = this->base_frame;
      }
    }
    if (this->gt_body_frame_ == this->base_frame) {
      this->T_base_gtbody_.setIdentity();
      this->gt_extrinsics_cached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "GT recovery: gt_body == base_frame ('%s'), using identity extrinsic",
                  this->base_frame.c_str());
    } else {
      try {
        auto tf_bg = this->tf_buffer->lookupTransform(
            this->base_frame, this->gt_body_frame_, tf2::TimePointZero);
        Eigen::Quaternionf q_bg(
            tf_bg.transform.rotation.w, tf_bg.transform.rotation.x,
            tf_bg.transform.rotation.y, tf_bg.transform.rotation.z);
        Eigen::Vector3f t_bg(
            tf_bg.transform.translation.x, tf_bg.transform.translation.y,
            tf_bg.transform.translation.z);
        this->T_base_gtbody_.setIdentity();
        this->T_base_gtbody_.block<3, 3>(0, 0) = q_bg.toRotationMatrix();
        this->T_base_gtbody_.block<3, 1>(0, 3) = t_bg;
        this->gt_extrinsics_cached_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "GT recovery: cached %s ← %s extrinsic: t=[%.3f,%.3f,%.3f]",
                    this->base_frame.c_str(), this->gt_body_frame_.c_str(),
                    t_bg.x(), t_bg.y(), t_bg.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GT recovery: cannot cache %s ← %s TF: %s — deferring snap",
                             this->base_frame.c_str(), this->gt_body_frame_.c_str(), ex.what());
      }
    }
  }

  // Odom init: on the first GT odom message (after the TF cache above is
  // populated), seed state from GT so the node starts at the correct location
  // even when the bag begins mid-run.  Composes through T_base_gtbody_ so the
  // seeded state.p lands at base_frame, not at gt_body_frame -- otherwise an
  // off-base gt_odom source would seed the state with a constant lever-arm
  // offset (the same defect that previously biased the cross-check). For the
  // AV-24 single-source P1 config the composition is identity since gt_body
  // == base_frame == gps_antenna_top. If the extrinsic hasn't cached yet (TF lookup
  // deferred), skip this message and try again on the next one rather than
  // seeding from a frame we can't compose.
  // Overrides any param-based initial pose. Sets first_opt_done so odom starts
  // publishing immediately without waiting for the first accepted GICP scan.
  if (this->use_odom_init_ && !this->use_odom_init_applied_) {
    Eigen::Vector3f init_p;
    Eigen::Quaternionf init_q;
    if (this->composeGtPoseInBase(s, init_p, init_q)) {
      this->use_odom_init_applied_ = true;
      const rclcpp::Time stamp_ros(msg->header.stamp, RCL_ROS_TIME);
      this->applyInitialPose(init_p, init_q, stamp_ros, "gt_odom");
      {
        std::lock_guard<std::mutex> lock(this->geo.mtx);
        this->geo.first_opt_done = true;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Odom init: pose set from GT odom at t=%.3f gt_pos=[%.2f,%.2f,%.2f] "
                  "-> base_pos=[%.2f,%.2f,%.2f] (gt_body='%s')",
                  s.stamp, s.p.x(), s.p.y(), s.p.z(),
                  init_p.x(), init_p.y(), init_p.z(),
                  this->gt_body_frame_.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Odom init: deferring -- gt_body -> base extrinsic not cached yet");
    }
  }

  bool first_gt_sample = false;
  {
    std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
    if (!this->gt_odom_buffer_.empty() && s.stamp <= this->gt_odom_buffer_.back().stamp) {
      // Out-of-order or duplicate timestamp; drop to keep buffer monotone.
      return;
    }
    this->gt_odom_buffer_.push_back(s);
    while (this->gt_odom_buffer_.size() > this->gt_odom_buffer_size_) {
      this->gt_odom_buffer_.pop_front();
    }
    first_gt_sample = !this->gt_odom_received_.exchange(true);
  }

  if (first_gt_sample) {
    RCLCPP_INFO(this->get_logger(),
                "First ground-truth odom received at stamp=%.3f frame=%s child_frame=%s",
                s.stamp, msg->header.frame_id.c_str(),
                msg->child_frame_id.empty() ? "(empty)" : msg->child_frame_id.c_str());
  }

  if (this->gt_recovery_enabled_ &&
      this->gt_recovery_hold_on_lidar_timeout_ &&
      this->initialized.load() &&
      this->gt_extrinsics_cached_) {
    const double last_lidar_stamp = this->last_pointcloud_stamp_.load();
    const double lidar_gap = last_lidar_stamp > 0.0
                                 ? (s.stamp - last_lidar_stamp)
                                 : 0.0;
    if (lidar_gap > this->gt_recovery_lidar_timeout_sec_) {
      const rclcpp::Time stamp_ros(msg->header.stamp, RCL_ROS_TIME);
      if (this->applyGtSampleToState(s, stamp_ros, "LiDAR timeout hold",
                                     this->consecutive_failures_,
                                     false, false, true, true, false)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "GT recovery: holding state on GT after %.3fs without LiDAR "
                             "(last_lidar=%.3f gt=%.3f)",
                             lidar_gap, last_lidar_stamp, s.stamp);
      }
    }
  }
}

bool gicp_localization::LocalizationNode::getGtPoseAt(double stamp, GtSample& out) {
  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (this->gt_odom_buffer_.size() < 2) {
    if (this->gt_odom_buffer_.size() == 1 &&
        std::abs(this->gt_odom_buffer_.front().stamp - stamp) <= this->gt_odom_max_dt_) {
      out = this->gt_odom_buffer_.front();
      return true;
    }
    return false;
  }
  // Buffer is monotone non-decreasing. Find the first sample with stamp >= query.
  auto it = std::lower_bound(
      this->gt_odom_buffer_.begin(), this->gt_odom_buffer_.end(), stamp,
      [](const GtSample& s, double t) { return s.stamp < t; });

  if (it == this->gt_odom_buffer_.begin()) {
    if (std::abs(it->stamp - stamp) > this->gt_odom_max_dt_) return false;
    out = *it; return true;
  }
  if (it == this->gt_odom_buffer_.end()) {
    auto last = std::prev(it);
    if (std::abs(last->stamp - stamp) > this->gt_odom_max_dt_) return false;
    out = *last; return true;
  }
  auto a = std::prev(it);
  auto b = it;
  const double dt_total = b->stamp - a->stamp;
  if (dt_total <= 0.0 || std::min(stamp - a->stamp, b->stamp - stamp) > this->gt_odom_max_dt_) {
    return false;
  }
  const float u = static_cast<float>((stamp - a->stamp) / dt_total);
  out.stamp = stamp;
  out.p = (1.0f - u) * a->p + u * b->p;
  out.q = a->q.slerp(u, b->q).normalized();
  out.v_lin_body = (1.0f - u) * a->v_lin_body + u * b->v_lin_body;
  out.v_ang_body = (1.0f - u) * a->v_ang_body + u * b->v_ang_body;
  return true;
}

bool gicp_localization::LocalizationNode::composeGtPoseInBase(
    const GtSample& gt, Eigen::Vector3f& p_out,
    Eigen::Quaternionf& q_out) const {
  if (!this->gt_extrinsics_cached_) {
    // Extrinsic not cached yet (first message hasn't fully run the cache
    // block, or TF lookup deferred).  Caller decides whether to fall back
    // to gt.p/gt.q directly or skip this cycle.
    return false;
  }
  // T_map_base = T_map_gtbody * inv(T_base_gtbody).  Decomposed:
  //   q_out = gt.q * inv(R_base_gtbody)
  //   p_out = gt.p - q_out * t_base_gtbody
  // Same math as the snap helper -- factored out so the cross-check and the
  // first-message applyInitialPose path use identical composition rather than
  // re-deriving (or skipping) the lever-arm.
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Quaternionf q_gtbody_in_base(R_base_gtbody);
  q_out = (gt.q * q_gtbody_in_base.conjugate()).normalized();
  p_out = gt.p - q_out * t_base_gtbody;
  return true;
}

bool gicp_localization::LocalizationNode::composeGtTwistInBase(
    const GtSample& gt, Eigen::Vector3f& v_lin_body_out,
    Eigen::Vector3f& v_ang_body_out) const {
  if (!this->gt_extrinsics_cached_) {
    return false;
  }
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Matrix3f R_gtbody_base = R_base_gtbody.transpose();
  const Eigen::Vector3f t_gtbody_base = -R_gtbody_base * t_base_gtbody;

  v_ang_body_out = R_gtbody_base * gt.v_ang_body;
  v_lin_body_out = R_gtbody_base * (gt.v_lin_body + gt.v_ang_body.cross(t_gtbody_base));
  return true;
}

// RTK-driven IMU bias calibration. Pairs each IMU sample with a time-matched GT
// pose/twist and accumulates the bias residual. Linear acceleration in world
// frame is estimated by finite-differencing v_world between successive paired
// samples. On window fill, biases are averaged, the state is seeded from the
// latest GT, imu_calibrated_ is flipped, and the function returns true.
bool gicp_localization::LocalizationNode::tryRtkCalibrationStep(
    double stamp, const Eigen::Vector3f& measured_gyro,
    const Eigen::Vector3f& measured_accel) {
  GtSample gt;
  if (!this->getGtPoseAt(stamp, gt)) {
    // GT not yet available at this IMU stamp (e.g., IMU briefly ahead of buffer).
    // Don't error — just skip this sample.
    return false;
  }
  Eigen::Vector3f gt_p_in_base;
  Eigen::Quaternionf gt_q_in_base;
  Eigen::Vector3f gt_v_lin_base_body;
  Eigen::Vector3f gt_v_ang_base_body;
  if (!this->composeGtPoseInBase(gt, gt_p_in_base, gt_q_in_base) ||
      !this->composeGtTwistInBase(gt, gt_v_lin_base_body, gt_v_ang_base_body)) {
    // GT sample exists, but base<-gt_body TF is not cached yet.
    return false;
  }
  GtSample seed = gt;
  seed.p = gt_p_in_base;
  seed.q = gt_q_in_base;
  seed.v_lin_body = gt_v_lin_base_body;
  seed.v_ang_body = gt_v_ang_base_body;
  this->latest_rtk_seed_ = seed;
  this->has_latest_rtk_seed_ = true;

  // Body acceleration in world frame, from finite-differencing v_world across
  // consecutive paired samples. Skip the first sample (no derivative possible).
  const Eigen::Matrix3f R = seed.q.toRotationMatrix();
  const Eigen::Vector3f v_world = R * seed.v_lin_body;

  if (!this->has_prev_gt_for_accel_) {
    this->has_prev_gt_for_accel_ = true;
    this->prev_gt_stamp_ = stamp;
    this->prev_v_world_ = v_world;
    return false;
  }

  const double dt = stamp - this->prev_gt_stamp_;
  if (dt <= 1e-4) {
    // Sample too close in time — derivative would explode. Skip.
    return false;
  }
  const Eigen::Vector3f a_world = (v_world - this->prev_v_world_) / static_cast<float>(dt);
  this->prev_gt_stamp_ = stamp;
  this->prev_v_world_ = v_world;

  // Specific-force convention: at rest body-upright, the IMU reads ~(0,0,+g) in body
  // (confirmed by stationary-calibration gravity_dir output on this rig). Generalized
  // to moving: expected_accel_body = R^T * (a_world + (0,0,+g)). Sign of g is +,
  // not −, because the accelerometer measures proper acceleration (= inertial − g_world
  // = inertial + (0,0,+g) when world Z points up).
  const Eigen::Vector3f g_world(0.0f, 0.0f, +static_cast<float>(this->gravity_));
  const Eigen::Vector3f expected_accel_body = R.transpose() * (a_world + g_world);
  const Eigen::Vector3f expected_gyro_body = seed.v_ang_body;

  const Eigen::Vector3f gyro_res = measured_gyro - expected_gyro_body;
  const Eigen::Vector3f accel_res = measured_accel - expected_accel_body;

  this->rtk_gyro_bias_sum_ += gyro_res;
  this->rtk_accel_bias_sum_ += accel_res;
  this->rtk_gyro_bias_sq_sum_ += gyro_res.cwiseProduct(gyro_res);
  this->rtk_accel_bias_sq_sum_ += accel_res.cwiseProduct(accel_res);
  this->rtk_calib_count_++;

  const double elapsed = stamp - this->rtk_calib_start_stamp_;
  if (elapsed < this->rtk_calib_window_sec_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "IMU calibrating (RTK-driven)... %.1f/%.1fs (%d samples)",
                         elapsed, this->rtk_calib_window_sec_, this->rtk_calib_count_);
    return false;
  }
  if (this->rtk_calib_count_ < 2) {
    // Window expired but we got essentially no useful pairings (e.g., GT buffer
    // empty most of the window). Give up on RTK init and let the caller
    // decide — return false but signal via a warn.
    RCLCPP_WARN(this->get_logger(),
                "RTK init: window expired with only %d residual samples; cannot calibrate. "
                "Falling back to stationary path.", this->rtk_calib_count_);
    this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    this->imu_calib_start_stamp_ = stamp;
    return false;
  }

  const float n = static_cast<float>(this->rtk_calib_count_);
  const Eigen::Vector3f gyro_bias = this->rtk_gyro_bias_sum_ / n;
  const Eigen::Vector3f accel_bias = this->rtk_accel_bias_sum_ / n;

  // Sanity check on the averaged bias magnitudes. Noise variance is not a useful
  // signal here — finite-differencing GT velocity at IMU rate amplifies cm-level
  // GPS noise into ~10 m/s² of accel-residual stddev even on a stationary vehicle.
  // The mean averages that out cleanly, so we only reject if the averaged bias
  // itself is implausible. Typical biases on real IMUs: gyro <0.05 rad/s,
  // accel <0.3 m/s². Generous thresholds here so a moderately drifted IMU is
  // still accepted.
  if (gyro_bias.norm() > 1.0f || accel_bias.norm() > 5.0f) {
    RCLCPP_WARN(this->get_logger(),
                "RTK init: averaged bias magnitudes implausible (|gyro|=%.3f rad/s, "
                "|accel|=%.3f m/s^2); falling back to stationary calibration",
                gyro_bias.norm(), accel_bias.norm());
    this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    this->imu_calib_start_stamp_ = stamp;
    this->rtk_calib_count_ = 0;
    this->rtk_gyro_bias_sum_.setZero();
    this->rtk_accel_bias_sum_.setZero();
    this->rtk_gyro_bias_sq_sum_.setZero();
    this->rtk_accel_bias_sq_sum_.setZero();
    return false;
  }

  // Apply biases + seed state from the latest GT sample.
  this->state.b.gyro = gyro_bias;
  this->state.b.accel = accel_bias;
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = this->latest_rtk_seed_.p;
    this->state.q = this->latest_rtk_seed_.q;
    this->state.v.lin.b = this->latest_rtk_seed_.v_lin_body;
    this->state.v.lin.w = this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_lin_body;
    this->state.v.ang.b = this->latest_rtk_seed_.v_ang_body;
    this->state.v.ang.w = this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_ang_body;
    this->geo.prev_p = this->latest_rtk_seed_.p;
    this->geo.prev_q = this->latest_rtk_seed_.q;
    this->geo.prev_vel = this->state.v.lin.w;
  }

  this->imu_calibrated_ = true;
  RCLCPP_INFO(this->get_logger(),
              "IMU calibrated (RTK-driven, %d samples, %.1fs): "
              "gyro_bias=[%.4f,%.4f,%.4f] accel_bias=[%.3f,%.3f,%.3f] "
              "seed_pos=[%.2f,%.2f,%.2f] seed_v=[%.2f,%.2f,%.2f]m/s",
              this->rtk_calib_count_, elapsed,
              gyro_bias.x(), gyro_bias.y(), gyro_bias.z(),
              accel_bias.x(), accel_bias.y(), accel_bias.z(),
              this->latest_rtk_seed_.p.x(), this->latest_rtk_seed_.p.y(), this->latest_rtk_seed_.p.z(),
              this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z());
  return true;
}

bool gicp_localization::LocalizationNode::applyGtSampleToState(
    const GtSample& gt, const rclcpp::Time& stamp,
    const char* reason, int failure_count,
    bool pose_mutex_already_held,
    bool publish_snap_marker,
    bool update_gicp_anchor,
    bool reset_failures,
    bool log_snap) {
  Eigen::Vector3f p_new;
  Eigen::Quaternionf q_new;
  if (!this->composeGtPoseInBase(gt, p_new, q_new)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "GT recovery: composeGtPoseInBase returned false; deferring correction.");
    return false;
  }

  Eigen::Vector3f v_base_body;
  Eigen::Vector3f omega_base_body;
  const bool twist_is_zero = gt.v_lin_body.norm() < 0.05f && gt.v_ang_body.norm() < 0.01f;
  if (twist_is_zero) {
    static bool warned_zero_twist = false;
    if (!warned_zero_twist) {
      warned_zero_twist = true;
      RCLCPP_INFO(this->get_logger(),
                  "GT recovery: incoming GT twist appears empty (lin=%.3f, ang=%.3f rad/s); "
                  "correcting with zero velocity",
                  gt.v_lin_body.norm(), gt.v_ang_body.norm());
    }
    v_base_body = Eigen::Vector3f::Zero();
    omega_base_body = Eigen::Vector3f::Zero();
  } else if (!this->composeGtTwistInBase(gt, v_base_body, omega_base_body)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "GT recovery: composeGtTwistInBase returned false; deferring correction.");
    return false;
  }

  const Eigen::Vector3f v_base_world = q_new * v_base_body;
  const Eigen::Vector3f omega_base_world = q_new * omega_base_body;

  auto write_state = [&]() {
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = q_new.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = p_new;
    if (update_gicp_anchor) {
      this->last_gicp_pose_ = this->current_pose;
      this->last_gicp_stamp_ = stamp;
      this->last_gicp_valid_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(this->geo.mtx);
      this->state.p = p_new;
      this->state.q = q_new;
      this->state.v.lin.b = v_base_body;
      this->state.v.lin.w = v_base_world;
      this->state.v.ang.b = omega_base_body;
      this->state.v.ang.w = omega_base_world;
      // state.b.gyro and state.b.accel intentionally preserved.
      this->geo.prev_p = p_new;
      this->geo.prev_q = q_new;
      this->geo.prev_vel = v_base_world;
      ++this->geo.update_seq;  // discard any in-flight propagateState computations
    }
    this->basePose.p = p_new;
    this->basePose.q = q_new;
    this->prev_vel = v_base_world;
  };

  if (pose_mutex_already_held) {
    write_state();
  } else {
    std::lock_guard<std::mutex> pose_lock(this->pose_mutex);
    write_state();
  }

  if (log_snap) {
    RCLCPP_WARN(this->get_logger(),
                "Localization: snapped pose to GT (%s after %d consecutive non-accepts) - "
                "pose=[%.2f,%.2f,%.2f] v=[%.2f,%.2f,%.2f]m/s omega=[%.2f,%.2f,%.2f]rad/s | gt_body=%s",
                reason, failure_count,
                p_new.x(), p_new.y(), p_new.z(),
                v_base_world.x(), v_base_world.y(), v_base_world.z(),
                omega_base_body.x(), omega_base_body.y(), omega_base_body.z(),
                this->gt_body_frame_.c_str());
  }

  if (publish_snap_marker) {
    geometry_msgs::msg::PoseStamped snap_msg;
    snap_msg.header.stamp = stamp;
    snap_msg.header.frame_id = this->map_frame;
    snap_msg.pose.position.x = p_new.x();
    snap_msg.pose.position.y = p_new.y();
    snap_msg.pose.position.z = p_new.z();
    snap_msg.pose.orientation.w = q_new.w();
    snap_msg.pose.orientation.x = q_new.x();
    snap_msg.pose.orientation.y = q_new.y();
    snap_msg.pose.orientation.z = q_new.z();
    if (this->gt_snap_pub) this->gt_snap_pub->publish(snap_msg);
  }

  if (reset_failures) {
    this->consecutive_failures_ = 0;
  }
  return true;
}

bool gicp_localization::LocalizationNode::maybeSnapPoseToGT(const char* reason) {
  // DIAGNOSTIC: prove helper is being called. Remove once snap behavior verified.
  RCLCPP_INFO(this->get_logger(),
              "GT recovery: maybeSnapPoseToGT entered (enabled=%d streak=%d/%d gt_received=%d cached=%d) reason='%s'",
              this->gt_recovery_enabled_,
              this->consecutive_failures_, this->gt_recovery_min_consecutive_failures_,
              this->gt_odom_received_.load(), this->gt_extrinsics_cached_, reason);
  // Guards. Below-threshold guard is silent (frequent on every rejection until
  // streak builds up); the others log throttled info so a misconfiguration
  // doesn't silently disable recovery.
  if (!this->gt_recovery_enabled_) return false;
  if (this->consecutive_failures_ < this->gt_recovery_min_consecutive_failures_) return false;
  if (!this->gt_odom_received_.load()) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — no GT odom received yet (streak=%d)",
                this->consecutive_failures_);
    return false;
  }
  if (!this->gt_extrinsics_cached_) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — base→%s TF not cached yet (streak=%d)",
                this->gt_body_frame_.c_str(), this->consecutive_failures_);
    return false;
  }

  GtSample gt;
  // Log buffer state before the lookup so we can diagnose silent failures.
  size_t buf_size = 0;
  double buf_oldest = 0.0, buf_newest = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
    buf_size = this->gt_odom_buffer_.size();
    if (buf_size > 0) {
      buf_oldest = this->gt_odom_buffer_.front().stamp;
      buf_newest = this->gt_odom_buffer_.back().stamp;
    }
  }
  bool got = this->getGtPoseAt(this->scan_stamp.seconds(), gt);
  RCLCPP_INFO(this->get_logger(),
              "GT recovery: lookup scan_stamp=%.3f got=%d buf=[size=%zu oldest=%.3f newest=%.3f] max_dt=%.3f",
              this->scan_stamp.seconds(), got, buf_size, buf_oldest, buf_newest, this->gt_odom_max_dt_);
  if (!got) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — no GT sample within max_dt=%.3fs of scan stamp %.3f (streak=%d)",
                this->gt_odom_max_dt_, this->scan_stamp.seconds(), this->consecutive_failures_);
    return false;
  }

  return this->applyGtSampleToState(gt, this->scan_stamp, reason,
                                    this->consecutive_failures_,
                                    true, true, false, true, true);
}

void gicp_localization::LocalizationNode::callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu) {

  double stamp = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;

  Eigen::Vector3f ang_vel(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
  Eigen::Vector3f lin_accel(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

  // One-shot defensive check: warn if the incoming IMU header.frame_id does
  // not match the configured imu_frame. The single-source P1 design assumes
  // both are "gps_antenna_top"; any other combination usually indicates the
  // imu_topic launch arg was re-pointed at a different non-Atlas IMU without
  // also updating localization/imu_frame. The
  // code would otherwise silently treat the foreign IMU's axes / lever-arm
  // as if they were at gps_antenna_top, because the TF lookup base_frame ->
  // imu_frame still returns identity in our yaml. Atomic exchange ensures the
  // warning fires exactly once even under the Reentrant callback group.
  // Empty frame_id is tolerated (some drivers leave it unset); only an
  // explicit mismatch trips the warn.
  const bool imu_frame_mismatch =
      !imu->header.frame_id.empty() &&
      imu->header.frame_id != this->imu_frame;
  if (!this->imu_frame_id_checked_.exchange(true)) {
    if (imu_frame_mismatch) {
      RCLCPP_WARN(this->get_logger(),
                  "IMU header.frame_id='%s' does not match configured "
                  "localization/imu_frame='%s'. Treating the IMU axes and "
                  "lever-arm as if at '%s' regardless of the message label. "
                  "If this is intentional (driver mislabels frame_id), "
                  "suppress this warning by updating localization/imu_frame "
                  "to match. If unintentional, the imu_topic remap is "
                  "probably pointing at the wrong IMU.",
                  imu->header.frame_id.c_str(),
                  this->imu_frame.c_str(),
                  this->imu_frame.c_str());
    }
  }
  if (imu_frame_mismatch && this->imu_require_frame_match_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Rejecting IMU sample: header.frame_id='%s' != localization/imu_frame='%s'",
                         imu->header.frame_id.c_str(), this->imu_frame.c_str());
    return;
  }

  // Cache IMU-to-baselink transform from TF (once)
  if (!this->imu_extrinsics_cached_) {
    try {
      auto tf_bi = this->tf_buffer->lookupTransform(
          this->base_frame, this->imu_frame, tf2::TimePointZero);
      Eigen::Quaternionf q_bi(
          tf_bi.transform.rotation.w, tf_bi.transform.rotation.x,
          tf_bi.transform.rotation.y, tf_bi.transform.rotation.z);
      Eigen::Vector3f t_bi(
          tf_bi.transform.translation.x, tf_bi.transform.translation.y,
          tf_bi.transform.translation.z);
      this->extrinsics.baselink2imu.R = q_bi.toRotationMatrix();
      this->extrinsics.baselink2imu.t = t_bi;
      this->extrinsics.baselink2imu_T.setIdentity();
      this->extrinsics.baselink2imu_T.block<3, 3>(0, 0) = q_bi.toRotationMatrix();
      this->extrinsics.baselink2imu_T.block<3, 1>(0, 3) = t_bi;
      this->imu_extrinsics_cached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Cached baselink->imu extrinsic: t=[%.3f,%.3f,%.3f]",
                  t_bi.x(), t_bi.y(), t_bi.z());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Cannot cache baselink->imu TF: %s", ex.what());
    }
  }

  if (!this->imu_extrinsics_cached_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Skipping IMU sample until baselink->imu TF is cached");
    return;
  }

  // Transform IMU measurements from IMU frame to baselink frame.
  const Eigen::Matrix3f& R = this->extrinsics.baselink2imu.R;
  const Eigen::Vector3f& t = this->extrinsics.baselink2imu.t;

  // Rotate angular velocity and linear acceleration to baselink frame
  Eigen::Vector3f ang_vel_bl = R * ang_vel;
  Eigen::Vector3f lin_accel_bl = R * lin_accel;

  // Lever-arm compensation: t is base->IMU, so r_IMU->base = -t.
  // a_base = a_imu - omega x (omega x t) - alpha x t; alpha is approximated
  // as zero because the angular-acceleration term is small at 100 Hz.
  lin_accel_bl -= ang_vel_bl.cross(ang_vel_bl.cross(t));

  ang_vel = ang_vel_bl;
  lin_accel = lin_accel_bl;

  ImuMeas imu_meas_temp;
  imu_meas_temp.stamp = stamp;
  imu_meas_temp.ang_vel = ang_vel;
  imu_meas_temp.lin_accel = lin_accel;

  // Calculate dt
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      imu_meas_temp.dt = stamp - this->imu_buffer.front().stamp;
    } else {
      imu_meas_temp.dt = 0.0;
    }

    this->imu_buffer.push_front(imu_meas_temp);
    this->imu_meas = imu_meas_temp;
  }

  if (!this->first_imu_received) {
    this->first_imu_received = true;
    this->first_imu_stamp_ = stamp;
    RCLCPP_INFO(this->get_logger(), "First IMU message received");

    // If RTK init is off entirely, go straight to the stationary path.
    if (!this->rtk_init_enabled_ && this->init_phase_.load() == InitPhase::WAITING) {
      this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    }
  }

  // Calibration phase machine. We may be:
  //   WAITING                 — RTK init enabled but no GT received yet
  //   RTK_CALIBRATING         — first GT arrived; accumulating IMU residuals against GT truth
  //   STATIONARY_CALIBRATING  — fallback (no GT in time, or RTK init disabled)
  //   DONE                    — biases applied; propagate normally
  if (!this->imu_calibrated_) {
    InitPhase phase = this->init_phase_.load();

    if (phase == InitPhase::WAITING) {
      if (this->gt_odom_received_.load()) {
        // First GT sample has arrived — start RTK-driven calibration on the next IMU.
        this->init_phase_ = InitPhase::RTK_CALIBRATING;
        this->rtk_calib_start_stamp_ = stamp;
        RCLCPP_INFO(this->get_logger(),
                    "RTK init: GT odom received; starting RTK-driven IMU calibration "
                    "(window=%.1fs)", this->rtk_calib_window_sec_);
        phase = InitPhase::RTK_CALIBRATING;
      } else if (stamp - this->first_imu_stamp_ > this->rtk_fallback_timeout_sec_) {
        // No GT in time — fall back to stationary calibration.
        RCLCPP_WARN(this->get_logger(),
                    "RTK init: no GT odom within %.1fs of first IMU; falling back to "
                    "stationary IMU calibration", this->rtk_fallback_timeout_sec_);
        this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
        this->imu_calib_start_stamp_ = stamp;  // reset so the existing window starts now
        phase = InitPhase::STATIONARY_CALIBRATING;
      } else {
        // Keep waiting. Don't propagate.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "RTK init: waiting for first GT odom (elapsed=%.1f/%.1fs)",
                             stamp - this->first_imu_stamp_, this->rtk_fallback_timeout_sec_);
        return;
      }
    }

    if (phase == InitPhase::RTK_CALIBRATING) {
      if (this->tryRtkCalibrationStep(stamp, ang_vel, lin_accel)) {
        // tryRtkCalibrationStep applied biases + seeded state and set imu_calibrated_.
        this->init_phase_ = InitPhase::DONE;
      } else if (!this->imu_calibrated_) {
        return;  // still accumulating — don't propagate
      }
    } else if (phase == InitPhase::STATIONARY_CALIBRATING) {
      // Original stationary path: assume omega=0 and accel direction = gravity.
      if (this->imu_calib_start_stamp_ < 0.0) {
        this->imu_calib_start_stamp_ = stamp;
      }

      this->imu_calib_gyro_sum_ += ang_vel;
      this->imu_calib_accel_sum_ += lin_accel;
      this->imu_calib_count_++;

      double elapsed = stamp - this->imu_calib_start_stamp_;
      if (elapsed >= this->imu_calib_time_ && this->imu_calib_count_ > 0) {
        Eigen::Vector3f gyro_avg = this->imu_calib_gyro_sum_ / static_cast<float>(this->imu_calib_count_);
        Eigen::Vector3f accel_avg = this->imu_calib_accel_sum_ / static_cast<float>(this->imu_calib_count_);

        this->state.b.gyro = gyro_avg;

        Eigen::Vector3f grav_world(0.f, 0.f, -1.f);
        Eigen::Vector3f grav_body = accel_avg.normalized();
        Eigen::Quaternionf q_init = Eigen::Quaternionf::FromTwoVectors(grav_body, grav_world);

        Eigen::Vector3f expected_grav_body = q_init.conjugate()._transformVector(
            Eigen::Vector3f(0.f, 0.f, -static_cast<float>(this->gravity_)));
        this->state.b.accel = accel_avg - expected_grav_body;

        if (!this->use_param_initial_pose_) {
          std::lock_guard<std::mutex> lock(this->geo.mtx);
          this->state.q = q_init;
          this->geo.prev_q = q_init;
        }

        this->imu_calibrated_ = true;
        this->init_phase_ = InitPhase::DONE;
        RCLCPP_INFO(this->get_logger(),
                    "IMU calibrated (stationary, %d samples, %.1fs): gyro_bias=[%.4f,%.4f,%.4f] "
                    "accel_bias=[%.3f,%.3f,%.3f] gravity_dir=[%.3f,%.3f,%.3f]",
                    this->imu_calib_count_, elapsed,
                    gyro_avg.x(), gyro_avg.y(), gyro_avg.z(),
                    this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z(),
                    grav_body.x(), grav_body.y(), grav_body.z());
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "IMU calibrating (stationary)... %.1f/%.1fs (%d samples)",
                             elapsed, this->imu_calib_time_, this->imu_calib_count_);
        return;
      }
    }
  }

  // Propagate state with geometric observer (only after initialization)
  // Note: counters are member-like but use thread_local to avoid data races
  // when the Reentrant callback group processes IMU concurrently.
  thread_local int propagate_calls = 0;
  thread_local int imu_total = 0;
  thread_local bool logged_first_propagate = false;
  imu_total++;

  if (this->initialized && (this->geo.first_opt_done || this->imu_only_mode_)) {
    this->propagateState();
    propagate_calls++;

    // Log first successful propagation
    if (!logged_first_propagate) {
      RCLCPP_INFO(this->get_logger(), "First IMU propagation successful! Starting high-frequency odometry.");
      logged_first_propagate = true;
    }
  }

  // Debug: Log IMU and propagation rates periodically
  thread_local int imu_count = 0;
  if (++imu_count % 100 == 0) {  // Log every 100 IMU messages (~1 second)
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    RCLCPP_INFO(this->get_logger(), "IMU rate check: %d callbacks, %d propagations, initialized=%d, geo_init=%d",
                imu_total, propagate_calls, this->initialized.load(), this->geo.first_opt_done.load());
    imu_total = 0;
    propagate_calls = 0;
  }
}

bool gicp_localization::LocalizationNode::imuMeasFromTimeRange(
    double start_time, double end_time,
    boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {

  std::lock_guard<std::mutex> lock(this->mtx_imu);

  if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
    // Not enough IMU data yet
    return false;
  }

  auto imu_it = this->imu_buffer.begin();

  auto last_imu_it = imu_it;
  imu_it++;
  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }

  if (imu_it == this->imu_buffer.end()) {
    // not enough IMU measurements
    return false;
  }
  imu_it++;

  // Set reverse iterators (to iterate forward in time)
  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localization::LocalizationNode::integrateImu(
    double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
    Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps) {

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

  if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT guard: empty=%d start=%.6f front=%.6f (start>front=%d)\n",
                   (int)sorted_timestamps.empty(), start_time,
                   sorted_timestamps.empty() ? 0.0 : sorted_timestamps.front(),
                   (int)(!sorted_timestamps.empty() && start_time > sorted_timestamps.front()));
      std::fflush(stderr);
    }
    return empty;
  }

  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
  if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it) == false) {
    double front_s = -1, back_s = -1;
    size_t sz = 0;
    {
      std::lock_guard<std::mutex> lk(this->mtx_imu);
      sz = this->imu_buffer.size();
      if (sz > 0) { front_s = this->imu_buffer.front().stamp; back_s = this->imu_buffer.back().stamp; }
    }
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT range: start=%.6f end=%.6f buf_sz=%zu front=%.6f back=%.6f (front<end=%d)\n",
                   start_time, sorted_timestamps.back(), sz, front_s, back_s,
                   (int)(sz > 0 && front_s < sorted_timestamps.back()));
      std::fflush(stderr);
    }
    return empty;
  }

  if ((begin_imu_it + 1) == end_imu_it) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT begin+1==end: start=%.6f end=%.6f begin.stamp=%.6f end.base.stamp=%.6f\n",
                   start_time, sorted_timestamps.back(),
                   begin_imu_it->stamp, end_imu_it.base()->stamp);
      std::fflush(stderr);
    }
    return empty;
  }

  const ImuMeas& f1 = *begin_imu_it;
  const ImuMeas& f2 = *(begin_imu_it+1);

  // Time between first two IMU samples
  double dt = f2.dt;

  if (dt < 1e-6) {
    if (this->verbose_) {
      std::fprintf(stderr, "[IMU_INT] REJECT dt: f1.stamp=%.6f f2.stamp=%.6f f2.dt=%.9f\n",
                   f1.stamp, f2.stamp, dt);
      std::fflush(stderr);
    }
    return empty;
  }

  // Time between first IMU sample and start_time
  double idt = start_time - f1.stamp;

  // Angular acceleration between first two IMU samples
  Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
  Eigen::Vector3f alpha = alpha_dt / dt;

  // Average angular velocity (reversed) between first IMU sample and start_time
  Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5*alpha*idt);

  // Set q_init to orientation at first IMU sample
  q_init = Eigen::Quaternionf (
    q_init.w() - 0.5*( q_init.x()*omega_i[0] + q_init.y()*omega_i[1] + q_init.z()*omega_i[2] ) * idt,
    q_init.x() + 0.5*( q_init.w()*omega_i[0] - q_init.z()*omega_i[1] + q_init.y()*omega_i[2] ) * idt,
    q_init.y() + 0.5*( q_init.z()*omega_i[0] + q_init.w()*omega_i[1] - q_init.x()*omega_i[2] ) * idt,
    q_init.z() + 0.5*( q_init.x()*omega_i[1] - q_init.y()*omega_i[0] + q_init.w()*omega_i[2] ) * idt
  );
  q_init.normalize();

  // Average angular velocity between first two IMU samples
  Eigen::Vector3f omega = f1.ang_vel + 0.5*alpha_dt;

  // Orientation at second IMU sample
  Eigen::Quaternionf q2 (
    q_init.w() - 0.5*( q_init.x()*omega[0] + q_init.y()*omega[1] + q_init.z()*omega[2] ) * dt,
    q_init.x() + 0.5*( q_init.w()*omega[0] - q_init.z()*omega[1] + q_init.y()*omega[2] ) * dt,
    q_init.y() + 0.5*( q_init.z()*omega[0] + q_init.w()*omega[1] - q_init.x()*omega[2] ) * dt,
    q_init.z() + 0.5*( q_init.x()*omega[1] - q_init.y()*omega[0] + q_init.w()*omega[2] ) * dt
  );
  q2.normalize();

  // Acceleration at first IMU sample
  Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
  a1[2] -= this->gravity_;

  // Acceleration at second IMU sample
  Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
  a2[2] -= this->gravity_;

  // Jerk between first two IMU samples
  Eigen::Vector3f j = (a2 - a1) / dt;

  // Set v_init to velocity at first IMU sample (go backwards from start_time)
  v_init -= a1*idt + 0.5*j*idt*idt;

  // Set p_init to position at first IMU sample (go backwards from start_time)
  p_init -= v_init*idt + 0.5*a1*idt*idt + (1/6.)*j*idt*idt*idt;

  return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localization::LocalizationNode::integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double>& sorted_timestamps,
    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
  a[2] -= this->gravity_;

  // Iterate over IMU measurements and timestamps
  auto prev_imu_it = begin_imu_it;
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != end_imu_it; imu_it++) {

    const ImuMeas& f0 = *prev_imu_it;
    const ImuMeas& f = *imu_it;

    // Time between IMU samples
    double dt = f.dt;

    if (dt < 1e-6) {
      prev_imu_it = imu_it;
      continue;
    }

    // Angular acceleration
    Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;

    // Average angular velocity
    Eigen::Vector3f omega = f0.ang_vel + 0.5*alpha_dt;

    // Orientation
    q = Eigen::Quaternionf (
      q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
      q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
      q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
      q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
    );
    q.normalize();

    // Acceleration
    Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    a[2] -= this->gravity_;

    // Jerk
    Eigen::Vector3f j_dt = a - a0;
    Eigen::Vector3f j = j_dt / dt;

    // Interpolate for given timestamps
    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      // Time between previous IMU sample and given timestamp
      double idt = *stamp_it - f0.stamp;

      // Average angular velocity
      Eigen::Vector3f omega_i = f0.ang_vel + 0.5*alpha*idt;

      // Orientation
      Eigen::Quaternionf q_i (
        q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
        q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
        q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
        q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
      );
      q_i.normalize();

      // Position
      Eigen::Vector3f p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;

      // Transformation
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
      T.block(0, 3, 3, 1) = p_i;

      imu_se3.push_back(T);

      stamp_it++;
    }

    // Position
    p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

    // Velocity
    v += a0*dt + 0.5*j_dt*dt;

    prev_imu_it = imu_it;

  }

  return imu_se3;

}

void gicp_localization::LocalizationNode::propagateState() {

  ImuMeas imu_local;
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    imu_local = this->imu_meas;
  }

  double dt = imu_local.dt;

  if (dt <= 0.0 || dt > 1.0) {
    static int skip_count = 0;
    if (++skip_count % 100 == 0) {
      RCLCPP_WARN(this->get_logger(), "Skipping propagation due to invalid dt: %.6f (skipped %d times)", dt, skip_count);
    }
    return;  // Skip invalid dt
  }

  // Read current state with minimal lock time
  Eigen::Vector3f current_p;
  Eigen::Quaternionf current_q;
  Eigen::Vector3f current_v_lin_w;
  Eigen::Vector3f bias_gyro;
  Eigen::Vector3f bias_accel;
  uint64_t seq_at_read;

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    current_p = this->state.p;
    current_q = this->state.q;
    current_v_lin_w = this->state.v.lin.w;
    bias_gyro = this->state.b.gyro;
    bias_accel = this->state.b.accel;
    seq_at_read = this->geo.update_seq;
  }

  // Do computation without holding lock
  Eigen::Quaternionf qhat = current_q;
  Eigen::Quaternionf omega;
  Eigen::Vector3f world_accel;

  // Apply gyro bias correction
  Eigen::Vector3f ang_vel_corrected = imu_local.ang_vel - bias_gyro;

  // Apply accel bias correction
  Eigen::Vector3f lin_accel_corrected = imu_local.lin_accel - bias_accel;

  // Transform accel from body to world frame and subtract gravity
  world_accel = qhat._transformVector(lin_accel_corrected);

  // Log propagation status periodically
  static int propagate_count = 0;
  if (++propagate_count % 1000 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Geo Observer: pos_z=%.3f vel_z=%.3f | accel_raw_z=%.3f bias_z=%.3f world_accel_z=%.3f | gravity=%.2f",
                current_p.z(), current_v_lin_w.z(),
                imu_local.lin_accel.z(), bias_accel.z(), world_accel.z(),
                this->gravity_);
  }

  // Position propagation (with gravity compensation)
  Eigen::Vector3f new_p = current_p;
  new_p += current_v_lin_w*dt + 0.5f*dt*dt*world_accel;
  new_p[2] -= 0.5f * dt * dt * static_cast<float>(this->gravity_);

  // Velocity propagation (with gravity compensation)
  Eigen::Vector3f new_v_lin_w = current_v_lin_w + world_accel*dt;
  new_v_lin_w[2] -= dt * static_cast<float>(this->gravity_);

  // Ground vehicle Z-velocity damping (same as in updateState)
  new_v_lin_w[2] *= (1.0f - dt * static_cast<float>(this->geo_Kz_damping_));

  // Orientation propagation
  omega.w() = 0;
  omega.vec() = ang_vel_corrected;
  Eigen::Quaternionf tmp = qhat * omega;
  Eigen::Quaternionf new_q;
  new_q.w() = qhat.w() + 0.5 * dt * tmp.w();
  new_q.vec() = qhat.vec() + 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  new_q.normalize();

  // Store angular velocity
  Eigen::Vector3f new_v_ang_b = ang_vel_corrected;
  Eigen::Vector3f new_v_ang_w = new_q.toRotationMatrix() * new_v_ang_b;

  // Validate computed state before publishing
  bool state_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) &&
                     std::isfinite(new_p.z()) && std::isfinite(new_q.w()) &&
                     std::isfinite(new_q.x()) && std::isfinite(new_q.y()) &&
                     std::isfinite(new_q.z()) &&
                     std::isfinite(new_v_lin_w.x()) && std::isfinite(new_v_lin_w.y()) &&
                     std::isfinite(new_v_lin_w.z());

  if (!state_valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Skipping odometry publish - state contains invalid values (p=[%.3f,%.3f,%.3f], q=[%.3f,%.3f,%.3f,%.3f])",
                         new_p.x(), new_p.y(), new_p.z(),
                         new_q.w(), new_q.x(), new_q.y(), new_q.z());
    return;  // Skip publishing if state contains invalid values
  }

  // Use IMU timestamp (from bag file or sensor)
  rclcpp::Time current_time;
  current_time = rclcpp::Time(static_cast<int64_t>(imu_local.stamp * 1e9));

  // Log successful validation on first publish
  static bool logged_first_publish = false;
  if (!logged_first_publish) {
    RCLCPP_INFO(this->get_logger(), "First odometry publish! Using IMU timestamp: %.3f", imu_local.stamp);
    logged_first_publish = true;
  }

  // Build odometry message from computed values (not from this->state to avoid race condition)
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = this->map_frame;
  odom_msg.child_frame_id = this->base_frame;

  // Position and orientation from propagated state
  odom_msg.pose.pose.position.x = new_p.x();
  odom_msg.pose.pose.position.y = new_p.y();
  odom_msg.pose.pose.position.z = new_p.z();
  odom_msg.pose.pose.orientation.w = new_q.w();
  odom_msg.pose.pose.orientation.x = new_q.x();
  odom_msg.pose.pose.orientation.y = new_q.y();
  odom_msg.pose.pose.orientation.z = new_q.z();

  // Velocity from propagated state (in world frame)
  odom_msg.twist.twist.linear.x = new_v_lin_w.x();
  odom_msg.twist.twist.linear.y = new_v_lin_w.y();
  odom_msg.twist.twist.linear.z = new_v_lin_w.z();
  odom_msg.twist.twist.angular.x = new_v_ang_w.x();
  odom_msg.twist.twist.angular.y = new_v_ang_w.y();
  odom_msg.twist.twist.angular.z = new_v_ang_w.z();

  // Pose covariance: diagonal only.
  // When GICP is accepted use sqrt(fitness) as a positional sigma (metres).
  // When dead-reckoning (consecutive GICP failures) inflate linearly per scan.
  {
    const double kBaseSigmaXY  = 0.05;   // m   — floor for accepted scans
    const double kBaseSigmaZ   = 0.10;   // m   — z less constrained by LiDAR
    const double kBaseSigmaRot = 0.01;   // rad — roll/pitch/yaw floor
    const double kFitnessScale = 1.0;    // sigma_xy = max(base, scale * sqrt(fitness))
    const double kDeadReckon   = 0.10;   // m per missed scan added to sigma

    double s_xy, s_z, s_rot;
    if (this->last_gicp_valid_ && this->last_fitness_score_ >= 0.0) {
      double f_sigma = kFitnessScale * std::sqrt(this->last_fitness_score_);
      s_xy  = std::max(kBaseSigmaXY,  f_sigma);
      s_z   = std::max(kBaseSigmaZ,   2.0 * f_sigma);
      s_rot = std::max(kBaseSigmaRot, 0.1 * f_sigma);
    } else {
      double drift = kDeadReckon * static_cast<double>(this->consecutive_failures_);
      s_xy  = kBaseSigmaXY  + drift;
      s_z   = kBaseSigmaZ   + 2.0 * drift;
      s_rot = kBaseSigmaRot + 0.05 * drift;
    }
    auto& c = odom_msg.pose.covariance;
    c.fill(0.0);
    c[0]  = s_xy  * s_xy;   // x
    c[7]  = s_xy  * s_xy;   // y
    c[14] = s_z   * s_z;    // z
    c[21] = s_rot * s_rot;  // roll
    c[28] = s_rot * s_rot;  // pitch
    c[35] = s_rot * s_rot;  // yaw
  }

  this->localized_odom_pub->publish(odom_msg);

  // Publish UTM-frame odometry
  if (this->utm_enabled_) {
    Eigen::Matrix4f T_map_base = Eigen::Matrix4f::Identity();
    T_map_base.block<3, 3>(0, 0) = new_q.toRotationMatrix();
    T_map_base.block<3, 1>(0, 3) = new_p;
    Eigen::Matrix4f T_utm_base = this->T_utm_map_ * T_map_base;
    Eigen::Vector3f utm_p = T_utm_base.block<3, 1>(0, 3);
    Eigen::Quaternionf utm_q(T_utm_base.block<3, 3>(0, 0));
    utm_q.normalize();
    // Rotate velocity into UTM frame
    Eigen::Vector3f utm_v_lin = this->T_utm_map_.block<3, 3>(0, 0) * new_v_lin_w;

    nav_msgs::msg::Odometry utm_odom_msg;
    utm_odom_msg.header.stamp = current_time;
    utm_odom_msg.header.frame_id = this->utm_frame;
    utm_odom_msg.child_frame_id = this->base_frame;
    utm_odom_msg.pose.pose.position.x = utm_p.x();
    utm_odom_msg.pose.pose.position.y = utm_p.y();
    utm_odom_msg.pose.pose.position.z = utm_p.z();
    utm_odom_msg.pose.pose.orientation.w = utm_q.w();
    utm_odom_msg.pose.pose.orientation.x = utm_q.x();
    utm_odom_msg.pose.pose.orientation.y = utm_q.y();
    utm_odom_msg.pose.pose.orientation.z = utm_q.z();
    utm_odom_msg.twist.twist.linear.x = utm_v_lin.x();
    utm_odom_msg.twist.twist.linear.y = utm_v_lin.y();
    utm_odom_msg.twist.twist.linear.z = utm_v_lin.z();
    utm_odom_msg.twist.twist.angular.x = new_v_ang_w.x();
    utm_odom_msg.twist.twist.angular.y = new_v_ang_w.y();
    utm_odom_msg.twist.twist.angular.z = new_v_ang_w.z();
    this->utm_odom_pub->publish(utm_odom_msg);
  }

  if (this->imu_only_mode_) {
    // Publish pose/TF directly from propagated IMU state when GICP is disabled.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = this->map_frame;
    pose_msg.pose.position.x = new_p.x();
    pose_msg.pose.position.y = new_p.y();
    pose_msg.pose.position.z = new_p.z();
    pose_msg.pose.orientation.w = new_q.w();
    pose_msg.pose.orientation.x = new_q.x();
    pose_msg.pose.orientation.y = new_q.y();
    pose_msg.pose.orientation.z = new_q.z();
    this->pose_pub->publish(pose_msg);

    static int path_decimator = 0;
    if (++path_decimator % 10 == 0) {
      std::lock_guard<std::mutex> path_lock(this->pose_mutex);
      if (this->path_buffer_.size() >= 10000) this->path_buffer_.pop_front();
      this->path_buffer_.push_back(pose_msg);
      if (this->path_pub && this->path_pub->get_subscription_count() > 0) {
        this->path_msg.header.stamp = current_time;
        this->path_msg.header.frame_id = this->map_frame;
        this->path_msg.poses.assign(this->path_buffer_.begin(), this->path_buffer_.end());
        this->path_pub->publish(this->path_msg);
      }
    }

    if (this->publish_tf_) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = current_time;
      transform_stamped.header.frame_id = this->map_frame;
      transform_stamped.child_frame_id = this->base_frame;
      transform_stamped.transform.translation.x = new_p.x();
      transform_stamped.transform.translation.y = new_p.y();
      transform_stamped.transform.translation.z = new_p.z();
      transform_stamped.transform.rotation.w = new_q.w();
      transform_stamped.transform.rotation.x = new_q.x();
      transform_stamped.transform.rotation.y = new_q.y();
      transform_stamped.transform.rotation.z = new_q.z();
      this->tf_broadcaster->sendTransform(transform_stamped);
    }
  }

  // Note: Pose publishing is done from publishPose() at GICP rate only
  // With unreliable IMU data, geometric observer propagation is not accurate
  // Better to publish only GICP-corrected poses at ~15 Hz than poorly-propagated poses at 100 Hz

  // Debug: Count published messages
  static int odom_publish_count = 0;
  static auto last_report_time = std::chrono::steady_clock::now();
  odom_publish_count++;

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report_time).count();
  if (elapsed >= 1000) {  // Report every second
    RCLCPP_INFO(this->get_logger(), "Odometry publish rate: %d Hz", odom_publish_count);
    odom_publish_count = 0;
    last_report_time = now;
  }

  // Update state AFTER publishing. If updateState() ran between our read and
  // this write (GICP corrected the state), skip the write to avoid overwriting
  // the fresh GICP correction with stale IMU-propagated values.
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    if (this->geo.update_seq == seq_at_read) {
      this->state.p = new_p;
      this->state.q = new_q;
      this->state.v.lin.w = new_v_lin_w;
      this->state.v.lin.b = new_q.toRotationMatrix().inverse() * new_v_lin_w;
      this->state.v.ang.b = new_v_ang_b;
      this->state.v.ang.w = new_v_ang_w;
    }
  }

  if (this->imu_only_mode_) {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = new_q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = new_p;
  }

  // Don't publish TF from propagated state - only from GICP-corrected pose in publishPose()
  // High-frequency TF from IMU propagation drifts between GICP corrections
  // TF publishing is handled in publishPose() at GICP rate (15 Hz) with corrected pose

}

void gicp_localization::LocalizationNode::updateState() {

  // Lock thread to prevent state from being accessed by propagateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  Eigen::Vector3f pin = this->basePose.p;
  Eigen::Quaternionf qin = this->basePose.q;
  double dt = this->observer_dt_;

  // On very first update after initialization, dt might be large
  // Just skip the update but don't warn
  if (dt <= 0.0) {
    return;  // Skip invalid dt
  }

  if (dt > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Large dt in updateState: %.3f sec, skipping update", dt);
    return;  // Skip if dt is too large (probably first update or dropped scans)
  }

  // Validate inputs
  bool inputs_valid = std::isfinite(pin.x()) && std::isfinite(pin.y()) && std::isfinite(pin.z()) &&
                      std::isfinite(qin.w()) && std::isfinite(qin.x()) && std::isfinite(qin.y()) && std::isfinite(qin.z()) &&
                      std::isfinite(this->state.p.x()) && std::isfinite(this->state.p.y()) && std::isfinite(this->state.p.z()) &&
                      std::isfinite(this->state.q.w()) && std::isfinite(this->state.q.x()) &&
                      std::isfinite(this->state.q.y()) && std::isfinite(this->state.q.z());

  if (!inputs_valid) {
    RCLCPP_WARN(this->get_logger(), "Invalid inputs in updateState - pin=[%.3f,%.3f,%.3f] state.p=[%.3f,%.3f,%.3f]",
                pin.x(), pin.y(), pin.z(), this->state.p.x(), this->state.p.y(), this->state.p.z());
    return;
  }

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Construct error quaternion
  qe = qhat.conjugate() * qin;

  double sgn = 1.0;
  if (qe.w() < 0) {
    sgn = -1.0;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - fabs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  // Position error
  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  // Optional online bias adaptation. Keep disabled by default for fused
  // Point One Atlas INS input so GICP residuals do not chase drift by rewriting the
  // trusted IMU bias estimate. Setting Kab/Kgb > 0 restores the upstream DLIO
  // adaptive observer behavior.
  if (this->geo_Kab_ > 0.0) {
    const double abias_max = this->geo_abias_max_;
    this->state.b.accel -= dt * this->geo_Kab_ * err_body;
    this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);
  }

  if (this->geo_Kgb_ > 0.0) {
    const double gbias_max = this->geo_gbias_max_;
    this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
    this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
    this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
    this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);
  }

  // Proportional observer correction (matching upstream DLIO design)
  // Position correction
  this->state.p += dt * this->geo_Kp_ * err;

  // Velocity correction
  this->state.v.lin.w += dt * this->geo_Kv_ * err;

  // Ground vehicle constraint: damp vertical velocity toward zero.
  // A ground vehicle's true Z-velocity is ~0; residual gravity miscompensation
  // causes vel_z to drift. Apply exponential decay each update.
  this->state.v.lin.w[2] *= (1.0f - dt * this->geo_Kz_damping_);

  // Orientation correction
  this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
  this->state.q.vec() += dt * this->geo_Kq_ * qcorr.vec();
  this->state.q.normalize();

  // Validate updated state
  bool state_valid_after = std::isfinite(this->state.p.x()) && std::isfinite(this->state.p.y()) &&
                           std::isfinite(this->state.p.z()) && std::isfinite(this->state.q.w()) &&
                           std::isfinite(this->state.v.lin.w.x()) && std::isfinite(this->state.v.lin.w.y()) &&
                           std::isfinite(this->state.v.lin.w.z());

  if (!state_valid_after) {
    RCLCPP_ERROR(this->get_logger(), "State became invalid after update! Resetting to GICP measurement.");
    // Reset to valid GICP measurement
    this->state.p = pin;
    this->state.q = qin;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
  }

  // Store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;
  ++this->geo.update_seq;  // Signal propagateState to discard stale computations

  // Log update status periodically
  static int update_count = 0;
  if (++update_count % 20 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Geo Observer | pos_err=[%.3f,%.3f,%.3f]m vel=[%.2f,%.2f,%.2f]m/s | bias_gyro=[%.4f,%.4f,%.4f] bias_accel=[%.3f,%.3f,%.3f]",
                err.x(), err.y(), err.z(),
                this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z(),
                this->state.b.gyro.x(), this->state.b.gyro.y(), this->state.b.gyro.z(),
                this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z());
  }

  RCLCPP_DEBUG(this->get_logger(),
               "Geo Observer: pos_err=[%.3f,%.3f,%.3f] vel=[%.2f,%.2f,%.2f] bias_a=[%.3f,%.3f,%.3f]",
               err.x(), err.y(), err.z(),
               this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z(),
               this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z());

}
