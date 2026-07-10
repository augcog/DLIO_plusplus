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

#include "gicp_plusplus/localization.h"
#include "gicp_plusplus/urdf_transforms.hpp"
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
#include <iterator>
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

// ---------------------------------------------------------------------------
// P1 gating rework: degeneracy-aware partial update ("solution remapping",
// Zhang & Singh ICRA'16). Instead of binary-rejecting a scan whose hessian is
// ill-conditioned, project the GICP correction onto the well-constrained
// eigen-subspace and keep the IMU prior along the degenerate directions.
//
// Frame handling: small_gicp's final hessian is parameterized as [omega; t]
// with the rotation taken about the WORLD ORIGIN (jacobian block is
// skew(transformed_point)). With the vehicle ~hundreds of metres from the map
// origin, a yaw about the origin is numerically indistinguishable from a
// translation, so the raw rotation block mostly measures lever-arm effects.
// We therefore re-center the hessian about the vehicle position c first:
// with new variables [omega; t_hat], t_hat = t + omega x c, the substitution
// x = A x_hat, A = [[I,0],[skew(c),I]] gives H_c = A^T H A whose rotation
// block measures rotations ABOUT THE VEHICLE. Conveniently t_hat is, to first
// order, exactly candidate_p - prior_p, so the projected translation applies
// directly to the pose difference.
// ---------------------------------------------------------------------------
struct DegeneracyProjection {
  bool valid{false};          // analysis ran (hessian finite, eigensolver ok)
  bool modified{false};       // projected pose differs from raw candidate
  bool fully_degenerate{false};  // all 6 axes degenerate -> caller should reject
  bool yaw_vetoed{false};     // yaw-consistency veto zeroed the yaw correction
  int degen_rot_axes{0};
  int degen_trans_axes{0};
  Eigen::Matrix4f projected_pose = Eigen::Matrix4f::Identity();
};

DegeneracyProjection projectDegenerateDelta(const Eigen::Matrix<double, 6, 6>& hessian,
                                            const Eigen::Matrix4f& T_prior,
                                            const Eigen::Matrix4f& candidate,
                                            bool apply_eigen_projection,
                                            bool full6d,
                                            double coupling_length_m,
                                            double rel_floor_6d,
                                            double rel_floor_rot,
                                            double rel_floor_trans,
                                            double veto_yaw_above_deg /* <=0 disables */) {
  DegeneracyProjection out;
  out.projected_pose = candidate;
  // A non-finite hessian only invalidates the EIGEN projection — the yaw veto
  // works on the pose delta alone and must not be silently disabled by it
  // (review fix). Callers treat valid=false as "reject when eigen projection
  // was required", which is still the right contract below.
  if (apply_eigen_projection && !hessian.allFinite()) return out;

  const Eigen::Matrix4d prior = T_prior.cast<double>();
  const Eigen::Matrix4d cand = candidate.cast<double>();
  const Eigen::Matrix3d R_prior = prior.block<3, 3>(0, 0);
  const Eigen::Matrix3d R_cand = cand.block<3, 3>(0, 0);
  const Eigen::Vector3d p_prior = prior.block<3, 1>(0, 3);
  const Eigen::Vector3d p_cand = cand.block<3, 1>(0, 3);

  // World-frame delta: candidate = T_delta * prior.
  const Eigen::Matrix3d R_delta = R_cand * R_prior.transpose();
  Eigen::AngleAxisd aa(R_delta);
  Eigen::Vector3d omega = aa.angle() * aa.axis();   // rotation correction (world axes, about vehicle)
  Eigen::Vector3d t_hat = p_cand - p_prior;         // translation correction of the vehicle

  Eigen::Vector3d omega_p = omega;
  Eigen::Vector3d t_hat_p = t_hat;

  if (apply_eigen_projection) {
    // Re-center the hessian about the vehicle position.
    const Eigen::Matrix<double, 6, 6> H_sym = 0.5 * (hessian + hessian.transpose());
    Eigen::Matrix3d skew_c;
    skew_c << 0.0, -p_prior.z(), p_prior.y(),
              p_prior.z(), 0.0, -p_prior.x(),
             -p_prior.y(), p_prior.x(), 0.0;
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    A.block<3, 3>(3, 0) = skew_c;
    const Eigen::Matrix<double, 6, 6> H_c = A.transpose() * H_sym * A;

    if (full6d) {
      // Full 6D solution remapping (Zhang & Singh). Rotation (rad) and
      // translation (m) are incommensurable, so scale rotation coordinates by
      // a characteristic coupling length L first: x_s = [L*omega; t_hat],
      // x = D x_s with D = diag(I/L, I), H_s = D H_c D. L should be the
      // typical constraint lever arm (~point-cloud radius after the 80 m
      // crop); with it, one unit of any scaled coordinate moves constraint
      // points by comparable metres, making the joint spectrum meaningful and
      // COUPLED rot/trans null directions (e.g. slide-along-a-wall = yaw +
      // lateral mix) visible — a blockwise analysis structurally cannot see
      // those.
      const double L = std::max(coupling_length_m, 1e-3);
      Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();
      D.block<3, 3>(0, 0) /= L;
      const Eigen::Matrix<double, 6, 6> H_s = D * H_c * D;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H_s);
      if (es.info() != Eigen::Success) {
        return out;  // eigensolver failure: leave candidate untouched, valid=false
      }
      const Eigen::Matrix<double, 6, 1> evals = es.eigenvalues().cwiseAbs();
      const double lambda_max = evals.maxCoeff();
      Eigen::Matrix<double, 6, 6> P6 = Eigen::Matrix<double, 6, 6>::Zero();
      int n_degen = 0;
      for (int i = 0; i < 6; ++i) {
        const Eigen::Matrix<double, 6, 1> v = es.eigenvectors().col(i);
        if (lambda_max <= 0.0 || evals[i] < rel_floor_6d * lambda_max) {
          ++n_degen;
          // Report which sub-block the degenerate direction mostly lives in
          // (diagnostic only; the projector itself is fully coupled).
          if (v.head<3>().norm() >= v.tail<3>().norm()) {
            ++out.degen_rot_axes;
          } else {
            ++out.degen_trans_axes;
          }
        } else {
          P6 += v * v.transpose();
        }
      }
      if (n_degen == 6) {
        out.valid = true;
        out.fully_degenerate = true;
        return out;
      }
      Eigen::Matrix<double, 6, 1> dx_s;
      dx_s.head<3>() = L * omega;
      dx_s.tail<3>() = t_hat;
      const Eigen::Matrix<double, 6, 1> dx_s_p = P6 * dx_s;
      omega_p = dx_s_p.head<3>() / L;
      t_hat_p = dx_s_p.tail<3>();
    } else {
      // Blockwise fallback (degeneracy/full6d: false): independent 3x3
      // eigen-analyses of the rot/trans blocks. Unit-mixing-free but blind to
      // coupled rot/trans degeneracy; kept for A/B comparison.
      Eigen::Matrix3d P_rot = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d P_trans = Eigen::Matrix3d::Identity();
      auto blockProjector = [](const Eigen::Matrix3d& block, double rel_floor,
                               Eigen::Matrix3d& projector, int& n_degen) -> bool {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(block);
        if (es.info() != Eigen::Success) return false;
        const Eigen::Vector3d evals = es.eigenvalues().cwiseAbs();
        const double lambda_max = evals.maxCoeff();
        projector.setZero();
        n_degen = 0;
        for (int i = 0; i < 3; ++i) {
          if (lambda_max <= 0.0 || evals[i] < rel_floor * lambda_max) {
            ++n_degen;
          } else {
            const Eigen::Vector3d v = es.eigenvectors().col(i);
            projector += v * v.transpose();
          }
        }
        return true;
      };

      if (!blockProjector(H_c.block<3, 3>(0, 0), rel_floor_rot, P_rot, out.degen_rot_axes) ||
          !blockProjector(H_c.block<3, 3>(3, 3), rel_floor_trans, P_trans, out.degen_trans_axes)) {
        return out;  // eigensolver failure: leave candidate untouched, valid=false
      }
      if (out.degen_rot_axes == 3 && out.degen_trans_axes == 3) {
        out.valid = true;
        out.fully_degenerate = true;
        return out;
      }
      omega_p = P_rot * omega;
      t_hat_p = P_trans * t_hat;
    }
  }

  // Turn-aware yaw-consistency veto: T_prior already contains the
  // IMU-integrated yaw across the scan gap, so omega.z() IS the GICP-vs-IMU
  // yaw disagreement. A large disagreement on a low-confidence match is the
  // wrong-basin entry signature; keep the IMU yaw instead.
  constexpr double kRad2Deg = 180.0 / M_PI;
  if (veto_yaw_above_deg > 0.0 && std::abs(omega_p.z()) * kRad2Deg > veto_yaw_above_deg) {
    omega_p.z() = 0.0;
    out.yaw_vetoed = true;
  }

  const double kEps = 1e-12;
  const bool changed = ((omega_p - omega).norm() > kEps) || ((t_hat_p - t_hat).norm() > kEps);
  out.valid = true;
  if (changed) {
    Eigen::Matrix3d R_delta_p = Eigen::Matrix3d::Identity();
    const double angle = omega_p.norm();
    if (angle > kEps) {
      R_delta_p = Eigen::AngleAxisd(angle, omega_p / angle).toRotationMatrix();
    }
    Eigen::Matrix4d projected = Eigen::Matrix4d::Identity();
    projected.block<3, 3>(0, 0) = R_delta_p * R_prior;
    projected.block<3, 1>(0, 3) = p_prior + t_hat_p;
    out.projected_pose = projected.cast<float>();
    out.modified = true;
  }
  return out;
}

// P1 yaw-safety: yaw component (deg, absolute) of the world-frame delta
// between two poses — the z element of the rotation vector of
// R_b * R_a^T. For a near-level ground vehicle this is the heading
// disagreement; roll/pitch live in the x/y components and are gated
// separately by the total-rotation jump threshold.
double yawInnovationDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Eigen::Matrix3d Ra = a.block<3, 3>(0, 0).cast<double>();
  const Eigen::Matrix3d Rb = b.block<3, 3>(0, 0).cast<double>();
  const Eigen::AngleAxisd aa(Rb * Ra.transpose());
  const Eigen::Vector3d rv = aa.angle() * aa.axis();
  return std::abs(rv.z()) * 180.0 / M_PI;
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

// Full point-field schema comparison, used to decide whether an auxiliary cloud
// can be byte-appended onto the primary cloud. mergeAuxClouds() transforms aux
// points using the AUX cloud's own field offsets, but the MERGED cloud keeps the
// PRIMARY's `fields`, so every downstream reader interprets the appended aux
// bytes with the primary layout. Appending is therefore only safe when the aux
// layout is byte-identical to the primary: same point_step, same endianness, and
// the same ordered set of field {name, offset, datatype, count}. A same-point_step
// cloud with different offsets/datatypes would otherwise be silently misread.
// This is O(#fields) (~10-20 per Luminar scan) — negligible next to GICP / voxel
// filtering / deskew, which run in ms. Returns true on match; on mismatch returns
// false and sets `reason` to a short human-readable description for logging.
bool auxSchemaMatchesPrimary(const sensor_msgs::msg::PointCloud2& aux,
                             const sensor_msgs::msg::PointCloud2& primary,
                             std::string& reason) {
  if (aux.point_step != primary.point_step) {
    reason = "point_step " + std::to_string(aux.point_step) + " vs primary " +
             std::to_string(primary.point_step);
    return false;
  }
  if (aux.is_bigendian != primary.is_bigendian) {
    reason = "endianness differs (aux is_bigendian=" + std::to_string(aux.is_bigendian) + ")";
    return false;
  }
  if (aux.fields.size() != primary.fields.size()) {
    reason = "field count " + std::to_string(aux.fields.size()) + " vs primary " +
             std::to_string(primary.fields.size());
    return false;
  }
  for (size_t i = 0; i < primary.fields.size(); ++i) {
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
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] verdict (heuristic; check raw lines to confirm):\n");
  if (plausible_ns_as_uint64(d_u64_0N) && !plausible_seconds(d_dbl_0N)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   uint64 ns interpretation looks plausible "
                 "(span %ld ns ~= %.3f ms)\n",
                 static_cast<long>(d_u64_0N), d_u64_0N * 1e-6);
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

// Decode a Luminar per-point ABSOLUTE epoch timestamp (uint64 ns) directly from
// PointCloud2 bytes. Single source of truth for which Luminar time encodings are
// accepted, shared by copyPointTimeFromCloud() (the per-point reader) and the
// multi-LiDAR deskew anchor capture in mergeAuxClouds(), so the two can never
// diverge on accepted formats. Returns false for an unsupported datatype.
//   * UINT8[8] / FLOAT64 -> raw uint64 epoch ns (8 bytes, little-endian)
//
// Only 8-byte carriers are accepted because the whole Luminar path treats these
// times as ABSOLUTE epoch ns: mergeAuxClouds() leaves them unshifted and
// deskewPointcloud() anchors on (ts - primary_min). A 32-bit field (UINT32)
// cannot hold an absolute epoch (it wraps every ~4.29 s) -- it would be a
// scan-relative counter, which this absolute path would silently misinterpret
// (dropping the inter-scan offset between aux and primary). So UINT32 is
// intentionally REJECTED here: a Luminar driver emitting UINT32 is unsupported
// and degrades to "no per-point time" (rigid transform) rather than corrupting
// deskew. `bytes_avail` (the field's room within point_step) guards the 8-byte
// read against a malformed/short time field.
inline bool luminarRawTimestampNsFromBytes(const uint8_t* tp, uint8_t datatype, int count, size_t bytes_avail, uint64_t& out) {
  if ((datatype == sensor_msgs::msg::PointField::FLOAT64 ||
       (datatype == sensor_msgs::msg::PointField::UINT8 && count == 8)) &&
      bytes_avail >= sizeof(uint64_t)) {
    std::memcpy(&out, tp, sizeof(uint64_t));
    return true;
  }
  return false;
}

// Copy per-point time from PointCloud2 into the dlio::Point union for the configured sensor.
// `point_step` bounds the field read so a malformed/short time field cannot read past the point.
void copyPointTimeFromCloud(const uint8_t* src, int time_off, uint8_t time_datatype, int time_count,
                           uint32_t point_step, dlio::SensorType sensor, PointType& dst) {
  if (time_off < 0 || static_cast<uint32_t>(time_off) >= point_step) {
    return;
  }
  const uint8_t* tp = src + time_off;
  const size_t bytes_avail = point_step - static_cast<uint32_t>(time_off);

  switch (sensor) {
    case dlio::SensorType::LUMINAR: {
      uint64_t ts_raw = 0;
      if (!luminarRawTimestampNsFromBytes(tp, time_datatype, time_count, bytes_avail, ts_raw)) {
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

// Shift per-point timestamps by `dt` seconds to rebase an aux scan's per-point
// times from its own header.stamp onto the merged cloud's primary header.stamp.
//
// Whether to actually shift depends on the underlying encoding:
//   * SCAN-RELATIVE encodings (FLOAT32/FLOAT64 seconds-since-scan-start,
//     UINT32 nanoseconds-since-scan-start) -> ADD dt so the value reads as
//     "seconds since primary scan start".
//   * ABSOLUTE-EPOCH encodings (Luminar Iris uint64 PTP epoch ns) -> DO NOT
//     shift. The downstream deskewer subtracts the merged cloud's
//     header.stamp to get a scan-relative offset, which already gives the
//     right (T_aux - T_primary + intra-aux-offset) when the value is left
//     at its absolute capture time. Shifting an absolute time by dt would
//     double-count the inter-scan offset and corrupt deskew.
//
// Luminar timestamp format (Luminar Iris Data Output Specification v1.3.0):
//   - The sensor does NOT emit a single uint64 epoch-ns field. It carries
//     48-bit integer epoch SECONDS once per packet header (§2.1, "PTP
//     Timestamp - seconds", UQ48.0) and a 32-bit SUB-SECOND NANOSECOND
//     count per ray (§2.2 / §2.6.3, "PTP Timestamp - nanoseconds", UQ32.0)
//     that wraps every 1 s. All fields are little-endian (§2).
//   - The uint64 epoch-ns this code reads is the upstream ROS driver's
//     reconstruction = header_seconds*1e9 + ray_nanoseconds. Correctness
//     depends on the driver performing that combination; a driver that
//     forwarded the bare 32-bit ns (sub-second sawtooth) would break deskew
//     across each 1 s rollover. Verify against the actual Luminar driver,
//     not the datasheet. (The "epoch time" guidance lives in the PTP
//     sections of the Product Information Guide, not the data layout.)
//
// `luminar_uint64=true` forces the 8 bytes at the time field to be read as
// uint64 regardless of declared datatype, because Luminar publishes the raw
// uint64 bits even when the field is mislabelled FLOAT64; generic FP
// arithmetic on those bits would scramble them.
void shiftCloudTimestamps(uint8_t* data, size_t num_points, uint32_t point_step,
                          int time_off, uint8_t time_datatype, int time_count,
                          double dt, bool luminar_uint64, double abs_clock_shift_s = 0.0) {
  if (time_off < 0) return;

  // Absolute-epoch path (Luminar Iris UINT8[8]): leave the per-point values
  // untouched. Each point already carries its absolute capture time; the
  // deskewer's t_i - merged_header.stamp computation in preprocessPointCloud
  // produces the correct intra-scan offset for both primary and aux points
  // without any rebasing here.
  if (luminar_uint64) {
    (void)dt;
    // P3 yaw-defect fix: absolute-epoch per-point times normally pass through
    // unshifted (each point carries its own capture time). But a CONSTANT
    // clock offset between this aux LiDAR and the primary/IMU clock makes
    // those absolute times land on the wrong segment of the IMU motion during
    // deskew — warping the merged scan during turns and creating false yaw
    // pressure. When a measured offset is configured
    // (lidar_concat/aux_time_offsets), correct the absolute times by it.
    // [REVIEW FIX 2026-07-08 P3] Both accepted absolute-epoch carriers hold
    // the same raw uint64 ns bits (see luminarRawTimestampNsFromBytes):
    // UINT8[8] (deployed Iris) AND a mislabelled FLOAT64 field. The offset
    // correction previously keyed on count == 8 only, silently skipping the
    // FLOAT64 carrier.
    if (abs_clock_shift_s != 0.0 &&
        (time_count == 8 ||
         time_datatype == sensor_msgs::msg::PointField::FLOAT64)) {
      const int64_t shift_ns = static_cast<int64_t>(abs_clock_shift_s * 1e9);
      for (size_t i = 0; i < num_points; ++i) {
        uint8_t* time_ptr = data + i * point_step + time_off;
        uint64_t val;
        std::memcpy(&val, time_ptr, sizeof(uint64_t));
        const int64_t shifted = static_cast<int64_t>(val) + shift_ns;
        val = static_cast<uint64_t>(std::max<int64_t>(0, shifted));
        std::memcpy(time_ptr, &val, sizeof(uint64_t));
      }
    }
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
        // UINT8 count=8 == Luminar Iris uint64 PTP epoch nanoseconds
        // (driver reconstruction of header seconds + per-ray nanoseconds;
        // see this function's header comment block for the format and the
        // deskew-correctness argument). Values are absolute capture times --
        // leave them untouched. For any other UINT8 count (e.g. raw byte
        // runs that are not timestamps), there is nothing sensible to shift.
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

gicp_plusplus::LocalizationNode::LocalizationNode() : Node("gicp_plusplus_node") {

  this->getParams();

  // Initialize flags
  this->initialized = false;
  this->first_imu_received = false;

  // Initialize pose
  this->current_pose = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->observer_prior_pose_ = Eigen::Matrix4f::Identity();
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
  this->base_pose_stamp_ = 0.0;
  this->t_prior_stamp_ = 0.0;
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

  // Initialize sensor type (default to OUSTER, can be configured)
  this->sensor = dlio::SensorType::OUSTER;

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
      std::bind(&gicp_plusplus::LocalizationNode::callbackPointCloud, this, std::placeholders::_1),
      pointcloud_sub_opt);

  this->initial_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = this->initial_pose_cb_group;
  this->initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&gicp_plusplus::LocalizationNode::callbackInitialPose, this, std::placeholders::_1),
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

  // [REVIEW FIX 2026-07-08 P1] MutuallyExclusive, not Reentrant. IMU still
  // runs in PARALLEL with pointcloud processing (different callback groups) —
  // exclusivity only serializes IMU-vs-IMU. Reentrant IMU callbacks could
  // interleave so an older callback propagated a newer sample (or two
  // callbacks propagated the same one), and the propagateState() commit
  // guard (geo.update_seq, which IMU commits do not advance) let the older
  // propagation overwrite the newer state. Serial in-order IMU processing
  // removes the entire class: monotonic buffer, one propagation per sample,
  // ordered commits, and the function-local static counters become safe.
  // Cost: none in practice — propagateState is O(100 us) at ~100 Hz.
  this->imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;

  // Set QoS for IMU subscriber (BEST_EFFORT to match sensor publishers)
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
  imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_qos,
      std::bind(&gicp_plusplus::LocalizationNode::callbackImu, this, std::placeholders::_1),
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
                   "Expected the fused Point One (Atlas) INS IMU path.",
                   resolved_imu_topic.c_str(), oss.str().c_str());
      throw std::runtime_error("IMU topic hard guard mismatch");
    }
  }
  RCLCPP_INFO(this->get_logger(),
              "IMU input topic: %s (expect Point One (Atlas) INS IMU, frame='%s', strict_frame_match=%s)",
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
        std::bind(&gicp_plusplus::LocalizationNode::callbackGtOdom, this, std::placeholders::_1),
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
    // P1 gating rework diagnostics
    this->dbg_fitness_ratio_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/fitness_ratio", 10);
    this->dbg_degen_rot_axes_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/degen_rot_axes", 10);
    this->dbg_degen_trans_axes_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/degen_trans_axes", 10);
    this->dbg_yaw_veto_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/yaw_veto", 10);
    this->dbg_yaw_innovation_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/yaw_innovation_deg", 10);
    this->dbg_yaw_stiffness_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/yaw_marginal_stiffness", 10);
    this->dbg_ins_yaw_diff_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/ins_yaw_diff_deg", 10);
    this->dbg_velocity_shadow_err_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/velocity_shadow_error_m", 10);
    this->dbg_velocity_shadow_age_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/velocity_shadow_age_s", 10);
    this->dbg_velocity_shadow_anchor_age_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/velocity_shadow_anchor_age_s", 10);
    this->dbg_velocity_shadow_available_pub =
        this->create_publisher<std_msgs::msg::Bool>("gicp/localization/debug/velocity_shadow_available", 10);
    this->dbg_snap_applied_pub =
        this->create_publisher<std_msgs::msg::Bool>("gicp/localization/debug/snap_applied", 10);
    // P4#3: per-frame lidar_concat diagnostics (one sample per processed frame).
    this->dbg_merged_aux_count_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/merged_aux_count", 10);
    this->dbg_scan_time_span_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/scan_time_span_s", 10);
    for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
      this->dbg_aux_dt_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(
          "gicp/localization/debug/aux" + std::to_string(i) + "_merge_dt_s", 10));
      this->dbg_aux_points_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(
          "gicp/localization/debug/aux" + std::to_string(i) + "_points", 10));
    }
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

gicp_plusplus::LocalizationNode::~LocalizationNode() {}

bool gicp_plusplus::LocalizationNode::loadUTMTransform(const std::string& path) {
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

void gicp_plusplus::LocalizationNode::getParams() {

  // Frame IDs
  this->declare_parameter<std::string>("localization/map_frame", "map");
  this->declare_parameter<std::string>("localization/base_frame", "base_link");
  this->declare_parameter<std::string>("odom/odom_frame", "odom");
  this->declare_parameter<std::string>("localization/imu_frame", "imu");
  this->declare_parameter<std::string>("localization/lidar_frame", "lidar");
  // Static base_frame<-lidar_frame lever arm (row-major 4x4) used to resolve the
  // extrinsic WITHOUT live TF in offline replay. Empty = rely on URDF
  // (lidar_concat/urdf_path) then live TF. See resolveBaseLidarExtrinsicOffline().
  this->declare_parameter<std::vector<double>>("localization/base_lidar_transform", std::vector<double>{});

  this->get_parameter("localization/map_frame", this->map_frame);
  this->get_parameter("localization/base_frame", this->base_frame);
  this->get_parameter("odom/odom_frame", this->odom_frame);
  this->get_parameter("localization/imu_frame", this->imu_frame);
  this->get_parameter("localization/lidar_frame", this->lidar_frame);
  this->get_parameter("localization/base_lidar_transform", this->base_lidar_static_);

  // Map parameters
  this->declare_parameter<std::string>("localization/map_path", "");
  this->declare_parameter<std::string>("localization/utm_transform_path", "");
  this->declare_parameter<std::string>("localization/utm_frame", "utm");
  this->declare_parameter<bool>("localization/visualize_map", true);
  this->declare_parameter<double>("localization/map_voxel_size_vis", 0.5);
  // Voxel leaf size (m) for the GICP TARGET map / kd-tree. A dense map (e.g. a
  // 49M-point GLIM export) builds a huge kd-tree -> >10 GiB RSS and swap thrash
  // that stalls registration. Downsampling the target to ~0.3 m cuts memory and
  // per-scan search cost with negligible accuracy loss at the matching 0.3 m
  // scan voxel (dlio/preprocessing/voxelFilter/res).
  // 0.0 disables (use the full-resolution map).
  this->declare_parameter<double>("localization/map_voxel_size", 0.3);
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
  this->get_parameter("localization/visualize_map", this->visualize_map_);
  this->get_parameter("localization/map_voxel_size_vis", this->map_voxel_size_vis_);
  this->get_parameter("localization/map_voxel_size", this->map_voxel_size_);
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
  // Default matches cfg/localization.yaml (P2#3: raised from 1; per-frame
  // snapping masked dead-reckoning quality). Keep the two in sync.
  this->declare_parameter<int>("localization/gt_recovery/min_consecutive_failures", 5);
  this->get_parameter("localization/gt_recovery/enable", this->gt_recovery_enabled_);
  this->get_parameter("localization/gt_recovery/min_consecutive_failures",
                      this->gt_recovery_min_consecutive_failures_);
  if (this->gt_recovery_enabled_ && !this->gt_odom_enabled_) {
    RCLCPP_WARN(this->get_logger(),
                "localization/gt_recovery/enable=true but gt_odom/enable=false — forcing gt_odom on so the buffer fills.");
    this->gt_odom_enabled_ = true;
  }
  if (this->gt_recovery_min_consecutive_failures_ < 1) {
    this->gt_recovery_min_consecutive_failures_ = 1;
  }

  // Stateful wrong-basin watchdog. The track is anchored only when the
  // pipeline already has a known pose (odom init / GT snap), then propagated
  // with Atlas velocity and orientation. Ordinary gate evaluations never read
  // Atlas position, so accepted-only GT metrics remain an external diagnostic
  // between explicit recovery anchors.
  this->declare_parameter<bool>("gicp/velocityShadow/enable", false);
  this->declare_parameter<double>("gicp/velocityShadow/maxHorizontalDivergenceM", 4.0);
  this->declare_parameter<double>("gicp/velocityShadow/maxAgeS", 0.15);
  this->declare_parameter<double>("gicp/velocityShadow/maxIntegrationGapS", 0.5);
  this->declare_parameter<double>("gicp/velocityShadow/maxAnchorAgeS", 300.0);
  this->declare_parameter<double>("gicp/velocityShadow/historyDurationS", 2.0);
  this->declare_parameter<bool>("gicp/velocityShadow/failClosedAfterAnchor", true);
  this->get_parameter("gicp/velocityShadow/enable", this->velocity_shadow_enabled_);
  this->get_parameter("gicp/velocityShadow/maxHorizontalDivergenceM",
                      this->velocity_shadow_max_horizontal_divergence_m_);
  this->get_parameter("gicp/velocityShadow/maxAgeS",
                      this->velocity_shadow_max_age_s_);
  this->get_parameter("gicp/velocityShadow/maxIntegrationGapS",
                      this->velocity_shadow_max_integration_gap_s_);
  this->get_parameter("gicp/velocityShadow/maxAnchorAgeS",
                      this->velocity_shadow_max_anchor_age_s_);
  this->get_parameter("gicp/velocityShadow/historyDurationS",
                      this->velocity_shadow_history_duration_s_);
  this->get_parameter("gicp/velocityShadow/failClosedAfterAnchor",
                      this->velocity_shadow_fail_closed_after_anchor_);
  if (!std::isfinite(this->velocity_shadow_max_horizontal_divergence_m_) ||
      this->velocity_shadow_max_horizontal_divergence_m_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "gicp/velocityShadow/maxHorizontalDivergenceM must be finite and >0; using 4.0m");
    this->velocity_shadow_max_horizontal_divergence_m_ = 4.0;
  }
  if (!std::isfinite(this->velocity_shadow_max_age_s_) ||
      this->velocity_shadow_max_age_s_ <= 0.0) {
    this->velocity_shadow_max_age_s_ = 0.15;
  }
  if (!std::isfinite(this->velocity_shadow_max_integration_gap_s_) ||
      this->velocity_shadow_max_integration_gap_s_ <= 0.0) {
    this->velocity_shadow_max_integration_gap_s_ = 0.5;
  }
  if (!std::isfinite(this->velocity_shadow_max_anchor_age_s_) ||
      this->velocity_shadow_max_anchor_age_s_ <= 0.0) {
    this->velocity_shadow_max_anchor_age_s_ = 300.0;
  }
  if (!std::isfinite(this->velocity_shadow_history_duration_s_) ||
      this->velocity_shadow_history_duration_s_ <= 0.0) {
    this->velocity_shadow_history_duration_s_ = 2.0;
  }
  // Retain enough integrated samples to answer any in-window query by
  // interpolation instead of backward-extrapolating from the newest sample.
  this->velocity_shadow_history_duration_s_ = std::max(
      this->velocity_shadow_history_duration_s_,
      2.0 * this->velocity_shadow_max_age_s_ +
          this->velocity_shadow_max_integration_gap_s_);
  if (this->velocity_shadow_enabled_ && !this->gt_odom_enabled_) {
    RCLCPP_WARN(this->get_logger(),
                "gicp/velocityShadow/enable=true requires gt_odom velocity; forcing localization/gt_odom/enable=true");
    this->gt_odom_enabled_ = true;
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

  // P1 gating rework (docs/action_plan_turn_error_20260704.md).
  // Rolling-median fitness baseline: absolute fitness thresholds calibrated on
  // a same-run map (floor 0.03-0.06) are meaningless on cross-run maps (floor
  // ~0.27), so gates operate on fitness / rolling-median instead.
  this->declare_parameter<bool>("gicp/fitnessBaseline/enable", true);
  this->declare_parameter<int>("gicp/fitnessBaseline/window", 201);
  this->declare_parameter<int>("gicp/fitnessBaseline/minSamples", 50);
  // Warm-up seed: expected per-map fitness floor used as the baseline until
  // minSamples accepted frames exist, so ratio gates / yaw veto are live from
  // frame 1 (0 = off; gates absolute-only during warm-up, pre-review behavior).
  this->declare_parameter<double>("gicp/fitnessBaseline/seedBaseline", 0.0);
  // Wrong-basin (bad-accept) gate: run-12 data shows good accepts at ratio
  // median 1.00 / p99 1.92, bad accepts (gt_err>20m) at median 1.34 / p90 2.55.
  this->declare_parameter<double>("gicp/fitnessRatioRejectThreshold", 2.0);
  // Degeneracy partial update: when the condition proxy crosses hessianCondMax,
  // project the correction instead of rejecting the whole scan (the old binary
  // reject produced 253-frame dead-reckoning streaks on run 12).
  this->declare_parameter<bool>("gicp/degeneracy/partialUpdate", true);
  // Full 6D coupled remapping (default) vs independent 3x3 rot/trans blocks.
  // full6d sees COUPLED rot/trans null directions (slide-along-a-wall = yaw +
  // lateral mix) that a blockwise analysis structurally cannot; rotation
  // coordinates are made commensurable with translation via couplingLengthM
  // (the typical constraint lever arm, ~point-cloud radius after cropping).
  this->declare_parameter<bool>("gicp/degeneracy/full6d", true);
  this->declare_parameter<double>("gicp/degeneracy/couplingLengthM", 20.0);
  this->declare_parameter<double>("gicp/degeneracy/relFloor6d", 0.02);
  this->declare_parameter<double>("gicp/degeneracy/relFloorRot", 0.02);
  this->declare_parameter<double>("gicp/degeneracy/relFloorTrans", 0.02);
  // Turn-aware yaw-consistency veto (Codex finding 1).
  this->declare_parameter<bool>("gicp/yawGate/enable", true);
  this->declare_parameter<double>("gicp/yawGate/maxCorrDeg", 1.5);
  this->declare_parameter<double>("gicp/yawGate/fitnessRatio", 1.2);
  // P1 yaw-safety: UNCONDITIONAL veto tier — fires regardless of fitness
  // ratio. The IMU prior's yaw cannot be this wrong over one scan gap on a
  // ground vehicle; any larger GICP yaw "correction" is a wrong basin with
  // plausible fitness (runs 19/20). <=0 disables the hard tier.
  this->declare_parameter<double>("gicp/yawGate/hardMaxCorrDeg", 8.0);
  // PR#6: bounds on the non-converged low-fitness fallback (see
  // performLocalization). Defaults from the PR validation replay.
  this->declare_parameter<double>("gicp/nonConvergedFitnessOkMaxTransM", 3.0);
  this->declare_parameter<double>("gicp/nonConvergedFitnessOkMaxRotDeg", 5.0);
  this->declare_parameter<int>("gicp/minCorrespondences", 500);
  this->declare_parameter<double>("gicp/minCorrespondenceRatio", 0.2);
  // ---- Algorithmic yaw-defect fixes (2026-07-05 deep-cause report) ----
  // Registration DoF for a ground vehicle. "4dof" (default) fixes roll+pitch
  // to the IMU prior inside the optimizer (translation+yaw free); "3dof"
  // additionally fixes yaw (translation-only registration); "6dof" restores
  // the unconstrained upstream behavior. Every full6dofEveryN-th scan runs
  // full 6-DoF so roll/pitch stay anchored to map structure (0 = never).
  this->declare_parameter<std::string>("gicp/dof/mode", "4dof");
  this->declare_parameter<int>("gicp/dof/full6dofEveryN", 10);
  // Soft attitude prior INSIDE the LM optimizer (info in rad^-2 on the
  // world-tangent residual vs the IMU-integrated initial guess). 0 = off.
  // Calibrate yawInfo against debug/yaw_marginal_stiffness (start at ~0.2x
  // the published median). rollPitchInfo only matters in 6dof mode.
  this->declare_parameter<double>("gicp/prior/yawInfo", 0.0);
  this->declare_parameter<double>("gicp/prior/rollPitchInfo", 0.0);
  // INS heading/pose prior (division of labor): IMU for propagation/deskew,
  // filtered_odom for the stable heading (+ optional position) prior.
  this->declare_parameter<bool>("localization/ins_prior/enable", true);
  this->declare_parameter<double>("localization/ins_prior/yaw_blend", 0.25);
  this->declare_parameter<double>("localization/ins_prior/max_yaw_step_deg", 2.0);
  this->declare_parameter<double>("localization/ins_prior/sanity_max_yaw_deg", 30.0);
  this->declare_parameter<double>("localization/ins_prior/pos_blend", 0.0);
  this->declare_parameter<bool>("localization/ins_prior/require_rtk_fixed", true);
  this->declare_parameter<double>("localization/ins_prior/max_yaw_sigma_deg", 3.0);

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
  this->get_parameter("gicp/fitnessBaseline/enable", this->fitness_baseline_enable_);
  this->get_parameter("gicp/fitnessBaseline/window", this->fitness_baseline_window_);
  this->get_parameter("gicp/fitnessBaseline/minSamples", this->fitness_baseline_min_samples_);
  this->get_parameter("gicp/fitnessBaseline/seedBaseline", this->fitness_baseline_seed_);
  this->get_parameter("gicp/fitnessRatioRejectThreshold", this->fitness_ratio_reject_);
  this->get_parameter("gicp/degeneracy/partialUpdate", this->degen_partial_update_enable_);
  this->get_parameter("gicp/degeneracy/full6d", this->degen_full6d_);
  this->get_parameter("gicp/degeneracy/couplingLengthM", this->degen_coupling_length_m_);
  this->get_parameter("gicp/degeneracy/relFloor6d", this->degen_rel_floor_6d_);
  this->get_parameter("gicp/degeneracy/relFloorRot", this->degen_rel_floor_rot_);
  this->get_parameter("gicp/degeneracy/relFloorTrans", this->degen_rel_floor_trans_);
  this->get_parameter("gicp/yawGate/enable", this->yaw_gate_enable_);
  this->get_parameter("gicp/yawGate/maxCorrDeg", this->yaw_gate_max_corr_deg_);
  this->get_parameter("gicp/yawGate/fitnessRatio", this->yaw_gate_fitness_ratio_);
  this->get_parameter("gicp/yawGate/hardMaxCorrDeg", this->yaw_gate_hard_max_corr_deg_);
  this->get_parameter("gicp/nonConvergedFitnessOkMaxTransM", this->gicp_nonconv_ok_max_trans_m_);
  this->get_parameter("gicp/nonConvergedFitnessOkMaxRotDeg", this->gicp_nonconv_ok_max_rot_deg_);
  this->get_parameter("gicp/minCorrespondences", this->gicp_min_correspondences_);
  this->get_parameter("gicp/minCorrespondenceRatio", this->gicp_min_corr_ratio_);
  this->get_parameter("gicp/dof/mode", this->gicp_dof_mode_);
  this->get_parameter("gicp/dof/full6dofEveryN", this->gicp_full6dof_every_n_);
  this->get_parameter("gicp/prior/yawInfo", this->gicp_prior_yaw_info_);
  this->get_parameter("gicp/prior/rollPitchInfo", this->gicp_prior_rollpitch_info_);
  this->get_parameter("localization/ins_prior/enable", this->ins_prior_enable_);
  this->get_parameter("localization/ins_prior/yaw_blend", this->ins_prior_yaw_blend_);
  this->get_parameter("localization/ins_prior/max_yaw_step_deg", this->ins_prior_max_yaw_step_deg_);
  this->get_parameter("localization/ins_prior/sanity_max_yaw_deg", this->ins_prior_sanity_max_yaw_deg_);
  this->get_parameter("localization/ins_prior/pos_blend", this->ins_prior_pos_blend_);
  this->get_parameter("localization/ins_prior/require_rtk_fixed", this->ins_prior_require_rtk_);
  this->get_parameter("localization/ins_prior/max_yaw_sigma_deg", this->ins_prior_max_yaw_sigma_deg_);
  // [REVIEW FIX 2026-07-08 P3] Sanitize: defaults are safe, but bad YAML values
  // would invert the prior's semantics silently — a negative max_yaw_step_deg
  // reaches std::clamp with REVERSED bounds (UB), a negative yaw_blend steers
  // AWAY from the INS, and a negative sanity threshold always trips (silently
  // disabling the prior). max_yaw_sigma_deg <= 0 is a documented gate-disable
  // and is left as-is.
  {
    auto sanitize = [this](const char* name, double& v, double lo, double hi,
                           double fallback) {
      if (!std::isfinite(v) || v < lo || v > hi) {
        RCLCPP_WARN(this->get_logger(),
                    "localization/ins_prior/%s = %.3f outside [%.3f, %.3f]; using %.3f",
                    name, v, lo, hi, fallback);
        v = fallback;
      }
    };
    sanitize("yaw_blend", this->ins_prior_yaw_blend_, 0.0, 1.0, 0.25);
    sanitize("max_yaw_step_deg", this->ins_prior_max_yaw_step_deg_, 0.0, 90.0, 2.0);
    sanitize("sanity_max_yaw_deg", this->ins_prior_sanity_max_yaw_deg_, 1e-3, 180.0, 30.0);
    sanitize("pos_blend", this->ins_prior_pos_blend_, 0.0, 1.0, 0.0);
  }
  RCLCPP_INFO(this->get_logger(),
              "INS prior: %s (yaw_blend=%.2f, max_step=%.1fdeg, sanity=%.1fdeg, pos_blend=%.2f, rtk_only=%s, max_yaw_sigma=%.1fdeg) — "
              "IMU=/gps_p1/imu for propagation/deskew, filtered_odom for stable heading",
              this->ins_prior_enable_ ? "ENABLED" : "disabled",
              this->ins_prior_yaw_blend_, this->ins_prior_max_yaw_step_deg_,
              this->ins_prior_sanity_max_yaw_deg_, this->ins_prior_pos_blend_,
              this->ins_prior_require_rtk_ ? "yes" : "no", this->ins_prior_max_yaw_sigma_deg_);
  if (this->gicp_dof_mode_ != "6dof" && this->gicp_dof_mode_ != "4dof" && this->gicp_dof_mode_ != "3dof") {
    RCLCPP_WARN(this->get_logger(), "gicp/dof/mode '%s' unknown; falling back to 6dof",
                this->gicp_dof_mode_.c_str());
    this->gicp_dof_mode_ = "6dof";
  }
  RCLCPP_INFO(this->get_logger(),
              "GICP DoF: %s (full 6dof every %d scans), rotation prior info yaw=%.1f rp=%.1f rad^-2",
              this->gicp_dof_mode_.c_str(), this->gicp_full6dof_every_n_,
              this->gicp_prior_yaw_info_, this->gicp_prior_rollpitch_info_);
  if (this->fitness_baseline_window_ < 3) this->fitness_baseline_window_ = 3;
  if (this->fitness_baseline_min_samples_ < 3) this->fitness_baseline_min_samples_ = 3;
  RCLCPP_INFO(this->get_logger(),
              "P1 gating: fitness baseline %s (window=%d, min=%d), ratio_reject=%.2f, "
              "partial_update=%s (%s, L=%.1fm, floor6d=%.3f, block floors rot=%.3f trans=%.3f), "
              "yaw_gate=%s (max=%.2fdeg, ratio>%.2f)",
              this->fitness_baseline_enable_ ? "ON" : "OFF",
              this->fitness_baseline_window_, this->fitness_baseline_min_samples_,
              this->fitness_ratio_reject_,
              this->degen_partial_update_enable_ ? "ON" : "OFF",
              this->degen_full6d_ ? "full6d" : "blockwise",
              this->degen_coupling_length_m_, this->degen_rel_floor_6d_,
              this->degen_rel_floor_rot_, this->degen_rel_floor_trans_,
              this->yaw_gate_enable_ ? "ON" : "OFF",
              this->yaw_gate_max_corr_deg_, this->yaw_gate_fitness_ratio_);

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
  // the primary sensor frame via TF (URDF), and per-point timestamps are rebased
  // by the inter-header dt so the merged sweep shares one clock.
  this->declare_parameter<bool>("localization/lidar_concat/enabled", false);
  this->declare_parameter<std::vector<std::string>>("localization/lidar_concat/aux_topics", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("localization/lidar_concat/aux_frames", std::vector<std::string>{});
  this->declare_parameter<double>("localization/lidar_concat/time_threshold", 0.05);
  // P4#3: 200 for parity with GLIM's lidar_concat. At 10 Hz a depth of 20 is
  // only 2 s of aux history — a brief aux-stream stall or replay burst drops
  // the matching scan and the frame silently degrades to fewer LiDARs (run-12
  // throttled logs: only ~35% of sampled scans merged 2/2). 200 = ~20 s.
  this->declare_parameter<int>("localization/lidar_concat/buffer_size", 200);
  // Yaw-defect fix (P3): constant per-aux clock offset (seconds, ADDED to the
  // aux header stamp AND to Luminar absolute per-point times) — absorbs a
  // constant offset between an aux LiDAR's clock and the primary/IMU clock
  // that would otherwise warp the merged scan during turns and create false
  // yaw pressure. Read the value straight off the per-aux offset diagnostic
  // ("header offset vs primary ... mean=..."); runs 19/20 showed 80-90 ms.
  // Order matches aux_topics; missing entries = 0.
  this->declare_parameter<std::vector<double>>("localization/lidar_concat/aux_time_offsets",
                                               std::vector<double>{});
  // Offline aux-extrinsic resolution (mirrors GLIM; no live TF needed).
  this->declare_parameter<std::string>("localization/lidar_concat/primary_frame", "luminar_front");
  this->declare_parameter<std::string>("localization/lidar_concat/urdf_path", "");
  this->declare_parameter<std::vector<double>>("localization/lidar_concat/aux_static_transforms", std::vector<double>{});
  // Strict merge guard. When require_all_aux=true, a scan that fails to merge every
  // configured aux is NOT localized on fewer LiDARs -- it is skipped. Brief transient
  // misses (buffers warming up, a dropped aux frame) are tolerated up to
  // max_consecutive_aux_merge_failures; past that, the node aborts if
  // abort_on_merge_failure=true, otherwise it keeps skipping non-fatally (louder
  // warning). require_all_aux=false localizes on whatever aux merged (no skip/abort).
  this->declare_parameter<bool>("localization/lidar_concat/require_all_aux", false);
  this->declare_parameter<bool>("localization/lidar_concat/abort_on_merge_failure", true);
  this->declare_parameter<int>("localization/lidar_concat/max_consecutive_aux_merge_failures", 10);

  this->get_parameter("localization/lidar_concat/enabled", this->concat_enabled_);
  std::vector<std::string> aux_topics_param, aux_frames_param;
  this->get_parameter("localization/lidar_concat/aux_topics", aux_topics_param);
  this->get_parameter("localization/lidar_concat/aux_frames", aux_frames_param);
  this->get_parameter("localization/lidar_concat/time_threshold", this->concat_time_threshold_);
  int concat_buffer_size_int = 20;
  this->get_parameter("localization/lidar_concat/buffer_size", concat_buffer_size_int);
  this->get_parameter("localization/lidar_concat/aux_time_offsets", this->concat_aux_time_offsets_);
  this->concat_buffer_size_ = static_cast<size_t>(std::max(1, concat_buffer_size_int));
  this->get_parameter("localization/lidar_concat/primary_frame", this->concat_primary_frame_);
  this->get_parameter("localization/lidar_concat/urdf_path", this->concat_urdf_path_);
  this->get_parameter("localization/lidar_concat/require_all_aux", this->concat_require_all_aux_);
  this->get_parameter("localization/lidar_concat/abort_on_merge_failure", this->concat_abort_on_merge_failure_);
  this->get_parameter("localization/lidar_concat/max_consecutive_aux_merge_failures", this->concat_max_consec_fail_);
  std::vector<double> aux_static_flat;
  this->get_parameter("localization/lidar_concat/aux_static_transforms", aux_static_flat);

  if (this->concat_enabled_) {
    if (aux_topics_param.size() != aux_frames_param.size()) {
      // A misconfigured REQUIRED merge must not silently degrade to primary-only.
      // Hard-fail only when the strict path is also set to abort; otherwise warn
      // and disable concat (non-fatal, consistent with abort_on_merge_failure=false).
      if (this->concat_require_all_aux_ && this->concat_abort_on_merge_failure_) {
        RCLCPP_FATAL(this->get_logger(),
                     "lidar_concat: aux_topics size (%zu) != aux_frames size (%zu) with require_all_aux=true and "
                     "abort_on_merge_failure=true; refusing to start. Fix the config, or set require_all_aux=false / "
                     "abort_on_merge_failure=false.",
                     aux_topics_param.size(), aux_frames_param.size());
        throw std::runtime_error("lidar_concat: aux_topics/aux_frames size mismatch (require_all_aux)");
      }
      RCLCPP_ERROR(this->get_logger(),
                   "lidar_concat: aux_topics size (%zu) != aux_frames size (%zu); disabling concat",
                   aux_topics_param.size(), aux_frames_param.size());
      this->concat_enabled_ = false;
    } else if (aux_topics_param.empty()) {
      if (this->concat_require_all_aux_ && this->concat_abort_on_merge_failure_) {
        RCLCPP_FATAL(this->get_logger(),
                     "lidar_concat enabled with no aux_topics, require_all_aux=true and abort_on_merge_failure=true; "
                     "refusing to start. Configure aux_topics/aux_frames, or set require_all_aux=false / "
                     "abort_on_merge_failure=false.");
        throw std::runtime_error("lidar_concat: enabled but no aux_topics configured (require_all_aux)");
      }
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
                  "lidar_concat enabled: %zu aux lidars, time_threshold=%.3fs, buffer_size=%zu",
                  this->aux_lidars_.size(), this->concat_time_threshold_, this->concat_buffer_size_);
      if (this->concat_aux_time_offsets_.size() < this->aux_lidars_.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "lidar_concat: aux_time_offsets has %zu/%zu entries; missing entries default to 0.0. "
                    "If aux LiDAR clocks have a constant offset vs the primary/IMU clock, configure "
                    "localization/lidar_concat/aux_time_offsets in aux_topics order.",
                    this->concat_aux_time_offsets_.size(), this->aux_lidars_.size());
      } else if (this->concat_aux_time_offsets_.size() > this->aux_lidars_.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "lidar_concat: aux_time_offsets has %zu entries for %zu aux lidars; extra entries will be ignored",
                    this->concat_aux_time_offsets_.size(), this->aux_lidars_.size());
      }
      for (const auto& a : this->aux_lidars_) {
        RCLCPP_INFO(this->get_logger(), "  aux lidar: topic='%s' frame='%s'",
                    a->topic.c_str(), a->frame.c_str());
      }

      // Split the flat static-transform array (16 row-major doubles per aux, in
      // aux order) into per-aux 4x4 matrices for the offline resolver.
      std::vector<std::vector<double>> aux_static_transforms;
      if (!aux_static_flat.empty()) {
        if (aux_static_flat.size() == 16 * this->aux_lidars_.size()) {
          aux_static_transforms.resize(this->aux_lidars_.size());
          for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
            aux_static_transforms[i].assign(aux_static_flat.begin() + 16 * i, aux_static_flat.begin() + 16 * (i + 1));
          }
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "lidar_concat: aux_static_transforms has %zu values, expected %zu (16 x %zu aux); ignoring",
                      aux_static_flat.size(), 16 * this->aux_lidars_.size(), this->aux_lidars_.size());
        }
      }

      // Resolve aux extrinsics now, without live TF (URDF > static > TF-at-runtime).
      this->resolveAuxExtrinsicsOffline(aux_static_transforms);
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
  // online IMU bias adaptation defaults off for the fused Point One (Atlas) INS path; the
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

  // P3: delta-form observer correction. The GICP measurement and its IMU
  // prior are both stamped at the scan's median point time, 0.1-0.3 s before
  // the correction is applied (half sweep + queueing + GICP solve). Legacy
  // behavior pulls the CURRENT state toward that stale absolute pose, which
  // is a systematic backward/yaw-lag drag during turns (accepted-frame gt_err
  // scaled with yaw rate on run 12). Delta form instead applies
  // T_corr = T_meas * inv(T_prior) — the time-free IMU-drift correction — to
  // the current state, so a perfect IMU/GICP agreement produces a ZERO
  // correction regardless of latency. false = legacy absolute-target observer.
  this->declare_parameter<bool>("odom/geo/delta_correction", true);
  this->get_parameter("odom/geo/delta_correction", this->geo_delta_correction_);

  // Observer-correction stability bounds (P2#1).
  this->declare_parameter<double>("odom/geo/observer_dt_max", 0.15);
  this->declare_parameter<double>("odom/geo/max_pos_correction", 0.0);
  this->declare_parameter<double>("odom/geo/max_vel_correction", 0.0);
  // P1 yaw-safety fix #3: per-update ORIENTATION clamps (position/velocity
  // already had them). Yaw clamp ON by default — the failure mode it bounds
  // (one bad accepted scan yanking heading tens of degrees) is exactly the
  // runs-19/20 signature. 0 disables.
  this->declare_parameter<double>("odom/geo/max_yaw_correction_deg", 5.0);
  this->declare_parameter<double>("odom/geo/max_rot_correction_deg", 0.0);
  this->get_parameter("odom/geo/observer_dt_max", this->geo_observer_dt_max_);
  this->get_parameter("odom/geo/max_pos_correction", this->geo_max_pos_correction_);
  this->get_parameter("odom/geo/max_vel_correction", this->geo_max_vel_correction_);
  this->get_parameter("odom/geo/max_yaw_correction_deg", this->geo_max_yaw_correction_deg_);
  this->get_parameter("odom/geo/max_rot_correction_deg", this->geo_max_rot_correction_deg_);
  if (this->geo_observer_dt_max_ <= 0.0) {
    this->geo_observer_dt_max_ = 0.15;  // guard against a non-positive cap disabling all corrections
  }

  // Time/speed-based dead-reckoning covariance growth (P3).
  this->declare_parameter<double>("odom/geo/dr_cov_time_rate", 0.5);
  this->declare_parameter<double>("odom/geo/dr_cov_dist_frac", 0.05);
  this->get_parameter("odom/geo/dr_cov_time_rate", this->dr_cov_time_rate_);
  this->get_parameter("odom/geo/dr_cov_dist_frac", this->dr_cov_dist_frac_);

  // Debug parameters
  this->declare_parameter<bool>("localization/debug/enable_pub", true);
  this->declare_parameter<bool>("localization/debug/enable_jump_log", true);
  this->declare_parameter<bool>("localization/debug/verbose_scan_log", false);
  this->declare_parameter<bool>("localization/debug/small_gicp_lm_debug", false);
  this->declare_parameter<bool>("localization/debug/nano_gicp_lm_debug", false);  // deprecated compatibility alias
  this->declare_parameter<double>("localization/debug/jump_trans_m", 1.0);
  this->declare_parameter<double>("localization/debug/jump_rot_deg", 10.0);
  this->declare_parameter<bool>("localization/verbose", true);

  this->get_parameter("localization/debug/enable_pub", this->debug_pub_enabled_);
  this->get_parameter("localization/debug/enable_jump_log", this->debug_jump_log_enabled_);
  this->get_parameter("localization/debug/verbose_scan_log", this->debug_verbose_scan_log_);
  this->get_parameter("localization/debug/small_gicp_lm_debug", this->debug_lm_print_);
  bool legacy_nano_lm_debug = false;
  this->get_parameter("localization/debug/nano_gicp_lm_debug", legacy_nano_lm_debug);
  this->debug_lm_print_ = this->debug_lm_print_ || legacy_nano_lm_debug;
  this->get_parameter("localization/debug/jump_trans_m", this->debug_jump_trans_m_);
  this->get_parameter("localization/debug/jump_rot_deg", this->debug_jump_rot_deg_);

  // Speed/scan_dt-aware jump-gate scaling (P2#2). 0 reproduces the fixed thresholds.
  this->declare_parameter<double>("localization/jump/trans_speed_scale", 1.5);
  this->declare_parameter<double>("localization/jump/rot_dt_scale_deg", 60.0);
  // P1 yaw-safety: yaw split out of the 3D rotation jump gate. The generic
  // 30 + 60*dt envelope admits re-acquisition but is physically absurd as a
  // YAW budget on a ground vehicle; yaw gets its own tight limit with an
  // absolute cap scan_dt scaling can never lift into the tens of degrees.
  this->declare_parameter<double>("localization/jump/yaw_max_deg", 10.0);
  this->declare_parameter<double>("localization/jump/yaw_dt_scale_deg", 10.0);
  this->declare_parameter<double>("localization/jump/yaw_total_max_deg", 15.0);
  this->get_parameter("localization/jump/trans_speed_scale", this->jump_trans_speed_scale_);
  this->get_parameter("localization/jump/rot_dt_scale_deg", this->jump_rot_dt_scale_deg_);
  this->get_parameter("localization/jump/yaw_max_deg", this->jump_yaw_max_deg_);
  this->get_parameter("localization/jump/yaw_dt_scale_deg", this->jump_yaw_dt_scale_deg_);
  this->get_parameter("localization/jump/yaw_total_max_deg", this->jump_yaw_total_max_deg_);

  this->get_parameter("localization/verbose", this->verbose_);

  // Suppress INFO/DEBUG logs when verbose is off; WARN/ERROR still pass through.
  // REVIEW FIX (commit 0e4916c follow-up): the WARN clamp also silenced the
  // INFO-level "SCAN DEBUG | status=ok/ok_partial" lines even when
  // debug/verbose_scan_log=true — the replay scorecard
  // (scripts/analyze_scan_debug_log.py) would then see only rejected frames
  // and report nonsense acceptance/streak/bucket stats. Keep INFO alive when
  // the per-scan evidence log is requested; verbose_ still gates the raw
  // stderr debug prints independently.
  if (!this->verbose_ && !this->debug_verbose_scan_log_) {
    rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_WARN);
  } else if (!this->verbose_) {
    RCLCPP_INFO(this->get_logger(),
                "localization/verbose=false but debug/verbose_scan_log=true: keeping INFO "
                "log level so per-scan SCAN DEBUG evidence reaches the log");
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
              "GT recovery: %s (min consecutive failures=%d)",
              this->gt_recovery_enabled_ ? "ENABLED" : "DISABLED",
              this->gt_recovery_min_consecutive_failures_);
  RCLCPP_INFO(this->get_logger(),
              "Velocity shadow gate: %s (max horizontal divergence=%.2fm, max extrapolation age=%.3fs, max integration gap=%.2fs, max anchor age=%.1fs, history=%.2fs, fail-closed-after-anchor=%s; Atlas position unused between anchors)",
              this->velocity_shadow_enabled_ ? "ENABLED" : "disabled",
              this->velocity_shadow_max_horizontal_divergence_m_,
              this->velocity_shadow_max_age_s_,
              this->velocity_shadow_max_integration_gap_s_,
              this->velocity_shadow_max_anchor_age_s_,
              this->velocity_shadow_history_duration_s_,
              this->velocity_shadow_fail_closed_after_anchor_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Debug: publish=%s jump_log=%s thresholds=[%.2fm, %.1fdeg]",
              this->debug_pub_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_log_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_trans_m_, this->debug_jump_rot_deg_);
  RCLCPP_INFO(this->get_logger(), "Debug detail: verbose_scan_log=%s small_gicp_lm_debug=%s",
              this->debug_verbose_scan_log_ ? "ENABLED" : "DISABLED",
              this->debug_lm_print_ ? "ENABLED" : "DISABLED");
}

bool gicp_plusplus::LocalizationNode::loadMap() {

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

  // Downsample the GICP TARGET map (in place) before it becomes the kd-tree.
  // A dense map (e.g. a ~49M-point GLIM export) otherwise builds a multi-GB
  // kd-tree that exhausts RAM/swap and stalls registration for seconds. Voxel
  // downsampling to ~0.3 m cuts the point count (and kd-tree memory) by ~10x
  // with negligible accuracy impact at the matching 0.3 m scan voxel
  // (dlio/preprocessing/voxelFilter/res). The dense cloud is
  // released as soon as the filter swaps in the downsampled result.
  if (this->map_voxel_size_ > 0.0) {
    const size_t before = this->map_cloud->points.size();
    auto map_ds = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(static_cast<float>(this->map_voxel_size_),
                   static_cast<float>(this->map_voxel_size_),
                   static_cast<float>(this->map_voxel_size_));
    vg.setInputCloud(this->map_cloud);
    vg.filter(*map_ds);
    if (map_ds->points.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "map_voxel_size=%.3f produced an empty map; keeping the full-resolution map",
                  this->map_voxel_size_);
    } else {
      this->map_cloud = map_ds;  // releases the dense cloud
      RCLCPP_INFO(this->get_logger(),
                  "Downsampled GICP target map: %lu -> %lu points (voxel=%.3f m)",
                  before, this->map_cloud->points.size(), this->map_voxel_size_);
    }
  }

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

void gicp_plusplus::LocalizationNode::start() {
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

void gicp_plusplus::LocalizationNode::applyInitialPoseFromParams() {

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

  // Block new scan callbacks before changing pose/observer/shadow state. A
  // mid-run reinitialization must not expose one frame of the new pose against
  // the old velocity-shadow track.
  this->initialized = false;
  if (this->velocity_shadow_enabled_) {
    std::lock_guard<std::mutex> shadow_lock(this->velocity_shadow_mtx_);
    this->velocity_shadow_initialized_ = false;
    this->velocity_shadow_history_.clear();
    RCLCPP_WARN(this->get_logger(),
                "Velocity shadow: invalidated by parameter initial pose; waiting for an explicit GT/RTK anchor");
  }

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = position;
    // [REVIEW FIX 2026-07-08 P3] initialized is published LAST (below), after
    // basePose/observer state are written — a MultiThreadedExecutor scan
    // callback could otherwise pass the atomic initialized check and start
    // deskew from a stale/default basePose.
  }

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = position;
    this->state.q = orientation;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    // [REVIEW FIX 2026-07-08 P3] Preserve calibrated IMU biases: reinit moves
    // the POSE estimate, it does not invalidate the sensor calibration.
    // Zeroing here while imu_calibrated_ stayed true meant callbackImu kept
    // subtracting a zero bias — calibration silently stopped being used.
    if (!this->imu_calibrated_.load()) {
      this->state.b.accel = Eigen::Vector3f::Zero();
      this->state.b.gyro = Eigen::Vector3f::Zero();
    }
    this->geo.prev_p = position;
    this->geo.prev_q = orientation;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  {
    std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);  // [P2 FIX 2026-07-09]
    this->basePose.p = position;
    this->basePose.q = orientation;
    this->base_pose_stamp_ = 0.0;  // parameter pose has no timestamp
  }
  this->initialized = true;  // publish only after ALL pose state is consistent

  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = this->now();

  RCLCPP_INFO(this->get_logger(),
              "Initial pose loaded from parameters at [%.2f, %.2f, %.2f] m with RPY [%.2f, %.2f, %.2f] rad",
              this->initial_pose_x_, this->initial_pose_y_, this->initial_pose_z_,
              this->initial_pose_roll_, this->initial_pose_pitch_, this->initial_pose_yaw_);
}

void gicp_plusplus::LocalizationNode::applyInitialPose(const Eigen::Vector3f& p,
                                                          const Eigen::Quaternionf& q_in,
                                                          const rclcpp::Time& stamp,
                                                          const std::string& source) {

  Eigen::Quaternionf q = q_in;
  if (q.squaredNorm() < 1e-10f) {
    RCLCPP_WARN(this->get_logger(), "Received near-zero quaternion in initial pose, ignoring");
    return;
  }
  q.normalize();

  // Serialize publication of the new seed with watchdog invalidation. This
  // closes the previous one-frame window where a concurrent scan could use
  // the new pose while velocityShadowAt still returned the old track.
  const bool was_initialized = this->initialized.exchange(false);
  if (this->velocity_shadow_enabled_ && source != "gt_odom") {
    std::lock_guard<std::mutex> shadow_lock(this->velocity_shadow_mtx_);
    this->velocity_shadow_initialized_ = false;
    this->velocity_shadow_history_.clear();
    RCLCPP_WARN(this->get_logger(),
                "Velocity shadow: invalidated by initial-pose reset from %s; waiting for an explicit GT/RTK anchor",
                source.c_str());
  }

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = p;
    // [REVIEW FIX 2026-07-08 P3] initialized is published LAST (below): with a
    // MultiThreadedExecutor, a scan callback could pass the atomic
    // initialized check here and start deskew from the stale/default basePose
    // that is only written further down.
    // [P2 FIX 2026-07-09] Overwrite scan_stamp only on the FIRST seed:
    // callbackPointCloud writes it on the scan thread without this mutex, so
    // a mid-run reinit retimed an in-flight scan's deskew/publish.
    if (stamp.nanoseconds() > 0 && !was_initialized) {
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
    // [REVIEW FIX 2026-07-08 P3] Same bias preservation as the param-pose
    // path — /initialpose (RViz) reinit must not discard RTK/stationary
    // calibration while imu_calibrated_ remains true.
    if (!this->imu_calibrated_.load()) {
      this->state.b.accel = Eigen::Vector3f::Zero();
      this->state.b.gyro = Eigen::Vector3f::Zero();
    }
    this->geo.prev_p = p;
    this->geo.prev_q = q;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  {
    // [P2 FIX 2026-07-09] Seed writes land atomically between scans.
    std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
    this->basePose.p = p;
    this->basePose.q = q;
    // [REVIEW FIX 2026-07-08] Valid at the provided stamp if there is one;
    // otherwise unknown (0) -> the INS prior falls back to prev_scan_stamp.
    this->base_pose_stamp_ = (stamp.nanoseconds() > 0) ? stamp.seconds() : 0.0;
  }
  this->initialized = true;  // publish only after ALL pose state is consistent

  // Clear trajectory path on reinitialization
  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = stamp.nanoseconds() > 0 ? stamp : this->now();

  RCLCPP_INFO(this->get_logger(), "Received initial pose (%s) at [%.2f, %.2f, %.2f]",
              source.c_str(), p.x(), p.y(), p.z());
}

void gicp_plusplus::LocalizationNode::callbackInitialPose(
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

void gicp_plusplus::LocalizationNode::callbackPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_in) {

  if (this->imu_only_mode_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "IMU-only mode enabled: skipping pointcloud/GICP updates.");
    return;
  }

  // Multi-LiDAR concatenation: merge nearest aux scans into the primary cloud
  // before any other processing. Downstream steps (TF cache, manual field
  // extraction, Luminar timestamp read, Y-flip, deskew, GICP) all run on the
  // merged cloud unchanged — primary frame_id, point_step, and field layout
  // are preserved.
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc =
      this->concat_enabled_ ? this->mergeAuxClouds(pc_in) : pc_in;

  // Strict multi-LiDAR merge (require_all_aux) returns nullptr when it skips an
  // incomplete scan so the degraded cloud is never registered. Drop this scan;
  // IMU/geometric propagation continues until a complete merged scan arrives.
  if (!pc) {
    return;
  }

  // Cache base_link -> lidar extrinsic from TF once. With
  // robot_state_publisher providing the URDF TF tree, this is the true
  // lever arm from the vehicle chassis (base_link) to the LiDAR sensor.
  // Deskewing below chains this as `frames[i] * baselink2lidar_T` so the
  // incoming points stay in their native LiDAR frame until then.
  if (!this->extrinsics_cached_) {
    // Resolve base_link -> lidar WITHOUT live TF first (URDF / static), so replay
    // without /tf_static or robot_state_publisher still localizes. Only fall back
    // to a live TF lookup if neither URDF nor a static transform is configured.
    if (this->resolveBaseLidarExtrinsicOffline(pc->header.frame_id)) {
      this->extrinsics_cached_ = true;
    } else {
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
                    "Cached baselink->lidar extrinsic from TF '%s': t=[%.3f,%.3f,%.3f]",
                    pc->header.frame_id.c_str(), t_bl.x(), t_bl.y(), t_bl.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for baselink->lidar TF ('%s' -> '%s'): %s. "
                             "Set localization/lidar_concat/urdf_path or localization/base_lidar_transform "
                             "for offline replay without /tf_static.",
                             this->base_frame.c_str(), pc->header.frame_id.c_str(), ex.what());
        return;
      }
    }
    // Apply the configured initial pose once the lever arm is known (either path).
    if (this->extrinsics_cached_ && this->pending_initial_pose_) {
      this->applyInitialPoseFromParams();
    }
  }

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
        copyPointTimeFromCloud(src, time_off, time_datatype, time_count, point_step, this->sensor, dst);
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

  // Crop box filter in SENSOR frame, BEFORE deskew. deskewPointcloud()
  // transforms points into the world frame, so cropping afterward (in
  // preprocessPointCloud) would clip a box centered on the MAP ORIGIN, deleting
  // the whole scan once the vehicle is more than crop_size_ from the origin. A
  // crop box is inherently a sensor-relative near/far-field filter, so it must
  // run here on the raw lidar-frame cloud. This also covers the deskew-fallback
  // paths (which leave the scan in sensor frame).
  this->cropBoxFilterSensorFrame(this->original_scan);

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

void gicp_plusplus::LocalizationNode::callbackAuxPointCloud(
    int aux_index, sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (aux_index < 0 || static_cast<size_t>(aux_index) >= this->aux_lidars_.size()) {
    return;
  }
  auto& aux = *this->aux_lidars_[aux_index];
  std::lock_guard<std::mutex> lk(aux.mtx);
  aux.buffer.push_back(std::move(msg));
  while (aux.buffer.size() > this->concat_buffer_size_) {
    aux.buffer.pop_front();
  }
}

// Resolve every aux LiDAR's T_primary_aux at startup WITHOUT live TF, so the
// multi-LiDAR merge works in offline replay (no robot_state_publisher / no
// /tf_static). Priority per aux: (1) URDF (the same av24.urdf GLIM reads, single
// source of truth), (2) a static row-major 4x4 from yaml, (3) leave unresolved
// so mergeAuxClouds() falls back to a runtime TF lookup. Any aux left unresolved
// here still works online exactly as before.
void gicp_plusplus::LocalizationNode::resolveAuxExtrinsicsOffline(
    const std::vector<std::vector<double>>& static_transforms) {
  // (1) URDF: parse once, resolve primary_frame <- aux.frame for each aux.
  std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> urdf_transforms;
  bool urdf_ok = false;
  if (!this->concat_urdf_path_.empty() && !this->concat_primary_frame_.empty()) {
    try {
      urdf_transforms = gicp_plusplus::parse_urdf_transforms(this->concat_urdf_path_);
      urdf_ok = true;
      RCLCPP_INFO(this->get_logger(), "lidar_concat: loaded URDF '%s' (primary_frame='%s')",
                  this->concat_urdf_path_.c_str(), this->concat_primary_frame_.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "lidar_concat: URDF parse failed (%s); falling back to static/TF", e.what());
    }
  }

  for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
    auto& aux = *this->aux_lidars_[i];

    if (urdf_ok) {
      try {
        const Eigen::Isometry3d T = gicp_plusplus::compute_transform(
            urdf_transforms, this->concat_primary_frame_, aux.frame);
        aux.T_primary_aux = T.matrix().cast<float>();
        aux.extrinsic_cached = true;
        aux.extrinsic_source = "urdf";
        const Eigen::Vector3f t = aux.T_primary_aux.block<3, 1>(0, 3);
        RCLCPP_INFO(this->get_logger(), "lidar_concat: resolved T(%s <- %s) from URDF: t=[%.3f, %.3f, %.3f]",
                    this->concat_primary_frame_.c_str(), aux.frame.c_str(), t.x(), t.y(), t.z());
        continue;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "lidar_concat: URDF has no %s <- %s chain (%s); trying static/TF",
                    this->concat_primary_frame_.c_str(), aux.frame.c_str(), e.what());
      }
    }

    // (2) Static row-major 4x4 from yaml.
    if (i < static_transforms.size() && static_transforms[i].size() == 16) {
      Eigen::Matrix4f M;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          M(r, c) = static_cast<float>(static_transforms[i][r * 4 + c]);
      aux.T_primary_aux = M;
      aux.extrinsic_cached = true;
      aux.extrinsic_source = "static";
      RCLCPP_INFO(this->get_logger(), "lidar_concat: resolved T(primary <- %s) from static yaml: t=[%.3f, %.3f, %.3f]",
                  aux.frame.c_str(), M(0, 3), M(1, 3), M(2, 3));
      continue;
    }

    // (3) Unresolved -> runtime TF fallback (existing behavior in mergeAuxClouds).
    aux.extrinsic_source = "tf";
    RCLCPP_WARN(this->get_logger(),
                "lidar_concat: aux '%s' has no URDF/static extrinsic; will rely on live TF '%s' <- '%s' "
                "(requires /tf_static at runtime -- set urdf_path or aux_static_transforms for offline replay)",
                aux.topic.c_str(), this->concat_primary_frame_.c_str(), aux.frame.c_str());
  }
}

// Resolve the base_frame <- lidar_frame lever arm WITHOUT live TF, so full GICP
// localization works in offline replay (no robot_state_publisher / /tf_static).
// Priority: (1) URDF (the same av24.urdf used for aux extrinsics, via
// lidar_concat/urdf_path), (2) a static row-major 4x4 from yaml. Returns false if
// neither is available, leaving the caller to fall back to a live TF lookup.
bool gicp_plusplus::LocalizationNode::resolveBaseLidarExtrinsicOffline(const std::string& lidar_frame) {
  auto apply = [this](const Eigen::Matrix4f& T) {
    this->extrinsics.baselink2lidar.R = T.block<3, 3>(0, 0);
    this->extrinsics.baselink2lidar.t = T.block<3, 1>(0, 3);
    this->extrinsics.baselink2lidar_T = T;
  };

  // (1) URDF.
  if (!this->concat_urdf_path_.empty()) {
    try {
      auto urdf = gicp_plusplus::parse_urdf_transforms(this->concat_urdf_path_);
      const Eigen::Matrix4f T = gicp_plusplus::compute_transform(urdf, this->base_frame, lidar_frame).matrix().cast<float>();
      apply(T);
      RCLCPP_INFO(this->get_logger(),
                  "Resolved baselink->lidar (%s <- %s) from URDF: t=[%.3f, %.3f, %.3f]",
                  this->base_frame.c_str(), lidar_frame.c_str(), T(0, 3), T(1, 3), T(2, 3));
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
                  "baselink->lidar URDF resolution failed (%s <- %s): %s; trying static/TF",
                  this->base_frame.c_str(), lidar_frame.c_str(), e.what());
    }
  }

  // (2) Static row-major 4x4 from yaml.
  if (this->base_lidar_static_.size() == 16) {
    Eigen::Matrix4f T;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        T(r, c) = static_cast<float>(this->base_lidar_static_[r * 4 + c]);
    apply(T);
    RCLCPP_INFO(this->get_logger(),
                "Resolved baselink->lidar from static yaml: t=[%.3f, %.3f, %.3f]", T(0, 3), T(1, 3), T(2, 3));
    return true;
  }

  return false;
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr
gicp_plusplus::LocalizationNode::mergeAuxClouds(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary) {

  this->luminar_primary_min_ts_valid_ = false;

  // P4#3: reset the per-frame concat diagnostics. Any early return below
  // leaves them at "nothing merged", which is exactly what happened.
  this->concat_last_merged_aux_ = 0;
  this->concat_last_aux_dt_.assign(this->aux_lidars_.size(),
                                   std::numeric_limits<double>::quiet_NaN());
  this->concat_last_aux_points_.assign(this->aux_lidars_.size(), 0);

  if (this->aux_lidars_.empty()) return primary;

  // Strict-merge failure handler. Single routing point for every "required merge
  // can't complete" path: incomplete aux merge AND primary-precondition failures
  // (missing XYZ, non-tight/padded primary).
  //   - require_all_aux=false -> returns false: degraded merging is allowed, the
  //     caller localizes on whatever LiDARs merged (front + any available aux).
  //   - require_all_aux=true  -> returns true: the caller returns nullptr so the
  //     degraded cloud is NEVER registered (the scan is skipped; IMU propagation
  //     continues). Past max_consecutive_aux_merge_failures the node either aborts
  //     (abort_on_merge_failure=true) or keeps skipping with a louder warning
  //     (abort_on_merge_failure=false). A fully merged scan resets the counter.
  auto on_required_failure = [this](size_t got, const char* reason) -> bool {
    if (!this->concat_require_all_aux_) return false;  // degraded merging allowed
    ++this->concat_consec_fail_;
    const bool over_budget = this->concat_consec_fail_ > this->concat_max_consec_fail_;
    if (over_budget && this->concat_abort_on_merge_failure_) {
      RCLCPP_FATAL(this->get_logger(),
                   "lidar_concat: multi-LiDAR merge REQUIRED but could not complete (%s) for %d consecutive "
                   "scans; abort_on_merge_failure=true -> shutting down. Set require_all_aux=false to localize "
                   "on available LiDARs, or abort_on_merge_failure=false to keep skipping non-fatally.",
                   reason, this->concat_consec_fail_);
      rclcpp::shutdown();
      throw std::runtime_error("lidar_concat: required multi-LiDAR merge failed");
    }
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "lidar_concat: REQUIRED merge incomplete (%zu/%zu aux): %s for %d consecutive scan(s) "
                          "(budget %d)%s -- skipping scan (degraded cloud NOT registered).",
                          got, this->aux_lidars_.size(), reason, this->concat_consec_fail_,
                          this->concat_max_consec_fail_, over_budget ? ", budget exceeded (non-fatal)" : "");
    return true;  // skip this scan
  };

  // Capture the PRIMARY scan's earliest per-point timestamp BEFORE appending any
  // aux cloud. deskewPointcloud() anchors Luminar merged-cloud timing on this --
  // NOT on the global merged minimum. An aux scan that began before the primary
  // carries smaller absolute epoch timestamps; anchoring at the global min would
  // map that aux point to the primary header stamp and deskew the entire merged
  // sweep late (a real motion-prior/deskew bias at AV speeds).
  if (this->sensor == dlio::SensorType::LUMINAR) {
    int p_t_off;
    uint8_t p_t_dt;
    int p_t_cnt;
    const size_t n_primary = static_cast<size_t>(primary->width) * primary->height;
    // Guard the raw 8-byte reads below: the time field must fit within point_step,
    // AND the byte buffer must actually hold all n_primary points. This capture runs
    // BEFORE the tight-cloud guard further down, so a truncated/malformed primary
    // (data.size() < n_primary*point_step) would otherwise read past data.end().
    if (findTimeField(*primary, p_t_off, p_t_dt, p_t_cnt) && p_t_off >= 0 &&
        primary->point_step > 0 && static_cast<uint32_t>(p_t_off) < primary->point_step &&
        primary->data.size() >= n_primary * static_cast<size_t>(primary->point_step)) {
      const size_t bytes_avail = primary->point_step - static_cast<uint32_t>(p_t_off);
      uint64_t pmin = std::numeric_limits<uint64_t>::max();
      bool any = false;
      for (size_t i = 0; i < n_primary; i++) {
        // Decode via the shared helper so the anchor matches the per-point reader
        // (copyPointTimeFromCloud) on the accepted absolute encodings (UINT8[8] /
        // FLOAT64). bytes_avail guards the 8-byte read against a short field.
        uint64_t ts = 0;
        if (luminarRawTimestampNsFromBytes(primary->data.data() + i * primary->point_step + p_t_off, p_t_dt, p_t_cnt,
                                           bytes_avail, ts)) {
          pmin = std::min(pmin, ts);
          any = true;
        }
      }
      if (any) {
        this->luminar_primary_min_ts_ns_ = pmin;
        this->luminar_primary_min_ts_valid_ = true;
      }
    }
  }

  const double t_primary = rclcpp::Time(primary->header.stamp).seconds();
  const uint32_t point_step = primary->point_step;
  const std::string& primary_frame = primary->header.frame_id;

  int x_off, y_off, z_off;
  if (!findXYZOffsets(*primary, x_off, y_off, z_off)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lidar_concat: cannot find xyz fields in primary cloud (frame='%s')",
                         primary_frame.c_str());
    if (on_required_failure(0, "primary cloud missing xyz fields")) return nullptr;
    return primary;
  }

  // Concatenation treats each cloud as a TIGHT array of point_step-sized points
  // (it byte-appends aux data and re-counts by point_step). A row-padded cloud
  // (row_step > width*point_step, i.e. data.size() != width*height*point_step)
  // would make the byte-count include padding. Organized-but-tight (height>1, no
  // padding) is fine to flatten; only padding is rejected. Reject the primary
  // loudly rather than silently miscounting -- Luminar clouds are unorganized and
  // tight (PCAP reader emits height=1, row_step=point_step*width).
  if (point_step == 0 || primary->data.size() != static_cast<size_t>(primary->width) * primary->height * point_step) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lidar_concat: primary cloud is organized/padded (data=%zu, width=%u, height=%u, step=%u); "
                         "skipping concat (only tight clouds can be byte-appended)",
                         primary->data.size(), primary->width, primary->height, point_step);
    if (on_required_failure(0, "primary cloud organized/padded (non-tight)")) return nullptr;
    return primary;
  }

  // Start the merged cloud as a copy of the primary; we'll append aux bytes.
  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  // Count points from the byte buffer (equals width*height for the tight cloud
  // validated above) so merged width/row_step always match the appended bytes.
  size_t total_points = primary->data.size() / point_step;
  size_t merged_aux_count = 0;

  // Reserve once for primary + all aux clouds (assuming roughly equal sizes).
  // Avoids per-aux reallocations as we grow merged->data.
  merged->data.reserve(primary->data.size() * (1 + this->aux_lidars_.size()));

  for (size_t aux_i = 0; aux_i < this->aux_lidars_.size(); ++aux_i) {
    auto& aux = *this->aux_lidars_[aux_i];

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

    // Pick the aux scan whose header is closest in time to the primary header,
    // within the configured threshold.
    sensor_msgs::msg::PointCloud2::ConstSharedPtr match;
    // P3 yaw-defect fix: correct a measured constant clock offset before any
    // time comparison — matching, rebasing, and diagnostics all see the
    // corrected aux timeline.
    const double aux_clock_off = (aux_i < this->concat_aux_time_offsets_.size())
                                     ? this->concat_aux_time_offsets_[aux_i] : 0.0;
    double best_dt = std::numeric_limits<double>::max();
    {
      std::lock_guard<std::mutex> lk(aux.mtx);
      for (const auto& msg : aux.buffer) {
        const double dt = std::abs(rclcpp::Time(msg->header.stamp).seconds() + aux_clock_off - t_primary);
        if (dt < best_dt) {
          best_dt = dt;
          match = msg;
        }
      }
    }
    if (!match || best_dt > this->concat_time_threshold_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "lidar_concat: no match for '%s' within %.3fs of primary t=%.3f (best_dt=%.3fs)",
                   aux.topic.c_str(), this->concat_time_threshold_, t_primary,
                   match ? best_dt : -1.0);
      continue;
    }
    // Validate the FULL field schema, not just point_step: the merged cloud
    // keeps the primary's `fields`, so an aux scan with the same point_step but
    // different field offsets/datatypes would be silently misread downstream.
    std::string schema_reason;
    if (!auxSchemaMatchesPrimary(*match, *primary, schema_reason)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — PointCloud2 schema mismatch vs primary: %s. "
                           "(Merged cloud uses the primary field layout; appending mismatched aux bytes "
                           "would misread them. Normalize the aux layout upstream to enable concatenation.)",
                           aux.topic.c_str(), schema_reason.c_str());
      continue;
    }

    // Append aux bytes directly into merged->data, then transform xyz + shift
    // timestamps in place over the just-appended region. No intermediate copy.
    // If validation fails after the append, roll back the resize so a malformed
    // aux scan can't leak into the merged cloud in its own (un-transformed) frame.
    // Reject an organized/padded or otherwise non-tight aux: byte-appending it
    // (or counting by point_step) would desync points from the field layout.
    // Requires data.size() == width*height*point_step (subsumes the multiple-of-
    // point_step check). Organized-but-tight is acceptable; only padding fails.
    if ((match->data.size() % point_step) != 0 ||
        match->data.size() != static_cast<size_t>(match->width) * match->height * point_step) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — non-tight cloud (data=%zu, width=%u, height=%u, step=%u)",
                           aux.topic.c_str(), match->data.size(), match->width, match->height, point_step);
      continue;
    }
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
    const bool luminar_u64 = (this->sensor == dlio::SensorType::LUMINAR);
    // [REVIEW FIX 2026-07-08 P3] For Luminar, "a time field exists" is not
    // "the time field is usable": the decoder accepts ONLY the 8-byte
    // absolute carriers (UINT8[8] raw uint64 epoch ns, or the same bits
    // mislabelled FLOAT64). Any other schema (e.g. a UINT32 relative counter)
    // is rejected per point later, silently collapsing deskew to a rigid
    // transform for rays that were merged as if they carried time. Treat an
    // unsupported Luminar schema like a missing time field here so the aux is
    // DROPPED under deskew instead.
    const bool usable_time_field = has_time_field &&
        (!luminar_u64 ||
         time_dt_type == sensor_msgs::msg::PointField::FLOAT64 ||
         (time_dt_type == sensor_msgs::msg::PointField::UINT8 && time_count == 8));
    if (usable_time_field) {
      // dt = aux header - primary header. Adding dt rebases aux per-point times
      // onto the primary clock so deskewing sees one coherent sweep.
      const double dt = rclcpp::Time(match->header.stamp).seconds() + aux_clock_off - t_primary;
      shiftCloudTimestamps(appended, aux_pts, point_step, time_off, time_dt_type, time_count, dt, luminar_u64,
                           aux_clock_off);
    } else if (this->deskew_) {
      // Without per-point timestamps the aux rays would deskew against the
      // primary scan's IMU integration with a stale (aux-header) reference,
      // smearing them. Drop the aux when deskew is enabled and times are
      // absent or (Luminar) unusable.
      if (has_time_field) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "lidar_concat: skipping '%s' — unsupported Luminar time schema "
                             "(datatype=%u count=%d; need UINT8[8] or FLOAT64 epoch-ns)",
                             aux.topic.c_str(), static_cast<unsigned>(time_dt_type), time_count);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "lidar_concat: skipping '%s' — deskew enabled but no time field found",
                             aux.topic.c_str());
      }
      merged->data.resize(old_size);
      continue;
    }

    // Accumulate the byte-derived count (matches the bytes actually appended),
    // not width*height, so merged->width/row_step stay consistent with data.
    total_points += aux_pts;
    ++merged_aux_count;

    // P4#3: per-frame + running merge-timing diagnostics. signed_dt is the
    // aux-vs-primary HEADER offset (not the |dt| used for matching): a stable
    // nonzero mean across the run is the signature of a constant per-aux
    // clock offset against the P1 timebase, which a per-aux time-offset
    // correction upstream could absorb (and which inflates deskew error at
    // high yaw rates).
    const double signed_dt = rclcpp::Time(match->header.stamp).seconds() + aux_clock_off - t_primary;
    this->concat_last_aux_dt_[aux_i] = signed_dt;
    this->concat_last_aux_points_[aux_i] = static_cast<int>(aux_pts);
    aux.dt_sum += signed_dt;
    aux.dt_min = std::min(aux.dt_min, signed_dt);
    aux.dt_max = std::max(aux.dt_max, signed_dt);
    if (++aux.dt_count % 512 == 0) {  // ~every 50 s at 10 Hz
      RCLCPP_INFO(this->get_logger(),
                  "lidar_concat: '%s' header offset vs primary over %lu merges: "
                  "mean=%+.1f ms, min=%+.1f ms, max=%+.1f ms%s",
                  aux.topic.c_str(), static_cast<unsigned long>(aux.dt_count),
                  1e3 * aux.dt_sum / static_cast<double>(aux.dt_count),
                  1e3 * aux.dt_min, 1e3 * aux.dt_max,
                  std::abs(aux.dt_sum / static_cast<double>(aux.dt_count)) > 0.02
                      ? " — mean >20 ms: likely constant clock offset, consider a per-aux time correction"
                      : "");
    }
  }

  this->concat_last_merged_aux_ = static_cast<int>(merged_aux_count);

  // The merged cloud is unorganized (height=1); width = total appended points.
  // Compute row_step in size_t so the point_step*total_points multiply cannot
  // overflow before the (message-mandated) uint32 assignment.
  merged->width = static_cast<uint32_t>(total_points);
  merged->height = 1;
  merged->is_dense = false;
  merged->row_step = static_cast<uint32_t>(static_cast<size_t>(point_step) * total_points);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "lidar_concat: merged %zu/%zu aux scans, total %zu points",
                       merged_aux_count, this->aux_lidars_.size(), total_points);

  // Strict guard: a REQUIRED multi-LiDAR merge that stays incomplete must not be
  // silently localized on fewer LiDARs (the run_5 "merged 0/2" failure mode). The
  // primary-precondition bail-outs above route through the same handler. When
  // require_all_aux is set, an incomplete merge SKIPS the scan (returns nullptr);
  // otherwise the degraded cloud is returned and localized. A fully merged scan
  // resets the budget.
  if (merged_aux_count < this->aux_lidars_.size()) {
    if (on_required_failure(merged_aux_count, "incomplete aux merge")) return nullptr;
  } else {
    this->concat_consec_fail_ = 0;
  }

  return merged;
}


// ---- INS heading/pose prior (division of labor, 2026-07-06) -----------------
// /gps_p1/imu drives IMU-RATE PROPAGATION + DESKEW; /gps_p1/filtered_odom
// supplies the STABLE HEADING (and optionally position) prior. The
// gyro-integrated prior chain inherits heading drift from the last accepted
// GICP pose; the Atlas INS heading is drift-free (dual-antenna aided).
//
// The correction is applied to basePose BEFORE integrateImu/deskew, so the
// deskewed world-frame cloud, T_prior, the initial guess, the 4-DoF fixed
// axes, the soft rotation-prior target, the yaw veto/innovation gates, and
// the delta-form observer ALL inherit the stable heading consistently.
// (Correcting T_prior after deskew would mislabel an already-placed cloud.)
//
// BLENDED and BOUNDED, never snapped: a constant map-vs-ENU yaw misalignment
// would otherwise be forced into every prior. A persistent nonzero ins_dyaw
// (debug topic / SCAN DEBUG field) MEASURES that misalignment — investigate
// it rather than raising the blend. With this correction in place, the hard
// yaw veto and yaw innovation gates are anchored to a drift-free reference.
void gicp_plusplus::LocalizationNode::applyInsHeadingPriorToBasePose() {
  this->last_ins_yaw_diff_deg_ = std::numeric_limits<double>::quiet_NaN();
  if (!this->ins_prior_enable_ || !this->gt_odom_enabled_ ||
      !this->gt_odom_received_.load() || this->prev_scan_stamp <= 0.0) {
    return;
  }
  // [REVIEW FIX 2026-07-08] Query the INS at the time basePose is actually
  // valid at. basePose is the pose at the MEDIAN POINT TIME of the previous
  // scan (frames[median_pt_index] / the accepted candidate), while
  // prev_scan_stamp is the scan HEADER time -- typically ~half a sweep
  // (~50 ms) earlier. Comparing the median-time seed attitude against the
  // header-time INS attitude injected a yaw-rate-proportional bias on turns
  // (50 ms * 30 deg/s = 1.5 deg), which the blend then pulled INTO the seed.
  // base_pose_stamp_ is maintained at every basePose write site; fall back to
  // prev_scan_stamp only if it was never set (pre-first-scan states).
  const double seed_stamp =
      (this->base_pose_stamp_ > 0.0) ? this->base_pose_stamp_ : this->prev_scan_stamp;
  GtSample ins;
  if (!this->getGtPoseAt(seed_stamp, ins)) return;
  if (this->ins_prior_require_rtk_ && !this->gtSampleIsRtkFixed(ins)) return;
  // [REVIEW FIX 2026-07-08] Heading-quality gate, mirroring GLIM gnss_global's
  // orientation_prior_max_yaw_sigma_deg. The position RTK gate above says
  // nothing about dual-antenna heading health: Atlas can be position-FIXED
  // while the heading solution is degraded (baseline outage, single-antenna
  // fallback), and a 0.25 blend at 2 deg/scan would happily steer the seed
  // toward that degraded heading. Yaw variance (rad^2) arrives per-sample in
  // pose.covariance[35] via the adapter; unpopulated (<=0) passes for compat
  // with GT sources that do not fill it (same convention as GLIM).
  if (this->ins_prior_max_yaw_sigma_deg_ > 0.0 && ins.cov_yaw > 0.0) {
    const double yaw_sigma_deg = std::sqrt(ins.cov_yaw) * 180.0 / M_PI;
    if (yaw_sigma_deg > this->ins_prior_max_yaw_sigma_deg_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "INS prior: heading quality gate — yaw sigma %.2f deg > %.2f deg, "
                           "prior NOT applied (position RTK may still be FIXED).",
                           yaw_sigma_deg, this->ins_prior_max_yaw_sigma_deg_);
      return;
    }
  }

  Eigen::Vector3f ins_p;
  Eigen::Quaternionf ins_q;
  if (!this->composeGtPoseInBase(ins, ins_p, ins_q)) {
    ins_p = ins.p;
    ins_q = ins.q;
  }

  // World-frame yaw difference: INS attitude vs the integration seed.
  const Eigen::Matrix3d R_seed = this->basePose.q.normalized().toRotationMatrix().cast<double>();
  const Eigen::Matrix3d R_ins = ins_q.normalized().toRotationMatrix().cast<double>();
  const Eigen::AngleAxisd aa(R_ins * R_seed.transpose());
  const double dyaw_deg = (aa.angle() * aa.axis()).z() * 180.0 / M_PI;
  this->last_ins_yaw_diff_deg_ = dyaw_deg;

  if (std::abs(dyaw_deg) > this->ins_prior_sanity_max_yaw_deg_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "INS prior: |INS-vs-prior yaw|=%.1f deg exceeds sanity %.1f deg — NOT "
                         "applied. Check map/ENU yaw alignment or INS attitude health.",
                         dyaw_deg, this->ins_prior_sanity_max_yaw_deg_);
    return;
  }

  double step_deg = this->ins_prior_yaw_blend_ * dyaw_deg;
  step_deg = std::clamp(step_deg, -this->ins_prior_max_yaw_step_deg_,
                        this->ins_prior_max_yaw_step_deg_);
  if (step_deg != 0.0) {
    const Eigen::Quaternionf rz(Eigen::AngleAxisf(
        static_cast<float>(step_deg * M_PI / 180.0), Eigen::Vector3f::UnitZ()));
    // Heading-only: rotate the seed ATTITUDE about world Z; position is
    // untouched (a heading prior must not translate the vehicle).
    this->basePose.q = (rz * this->basePose.q).normalized();
    // [REVIEW FIX 2026-07-08] Apply the SAME world-Z step to the IMU-rate
    // observer state. The observer runs in delta form: updateState() applies
    // T_corr = T_meas * T_prior^-1 to the current state, so when GICP simply
    // CONFIRMS the corrected prior the delta is identity and state.q — the
    // attitude the IMU-rate odometry (propagateState) publishes from — would
    // never inherit the heading fix; only the scan-time chain would converge.
    // Rotating seed and observer state by the identical bounded step keeps the
    // two chains consistent (their relative geometry is unchanged, so the
    // delta correction is unaffected) and makes the "INS pull decays the yaw
    // error within a few scans" property true for the IMU-rate output too.
    // World-frame quantities rotate; body-frame ones (v.lin.b, v.ang.b,
    // biases) and position (rotation is about the vehicle) are invariant.
    {
      std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
      this->state.q = (rz * this->state.q).normalized();
      this->state.v.lin.w = rz * this->state.v.lin.w;
      this->state.v.ang.w = rz * this->state.v.ang.w;
      this->geo.prev_q = (rz * this->geo.prev_q).normalized();
      this->geo.prev_vel = rz * this->geo.prev_vel;
      ++this->geo.update_seq;  // discard any in-flight propagateState computations
    }
    // Scan-thread integration seed velocity is world-frame as well.
    this->prev_vel = rz * this->prev_vel;
  }
  // Optional position blend (default 0 = OFF). NOTE the evaluation
  // circularity: with this on, gt_pos_err against the same INS is no longer
  // an independent metric — use held-out segments when scoring.
  if (this->ins_prior_pos_blend_ > 0.0) {
    const Eigen::Vector3f dp = static_cast<float>(this->ins_prior_pos_blend_) *
                               (ins_p - this->basePose.p);
    this->basePose.p += dp;
    // [REVIEW FIX 2026-07-08] Mirror the yaw handling: shift the IMU-rate
    // observer state by the SAME translation so the delta-form observer sees
    // a consistent correction. Without this, a GICP result that merely
    // confirms the shifted prior yields an identity delta and the IMU-rate
    // odometry (propagateState from state.p) never inherits the position
    // pull — scan-time and IMU-rate state split again. Velocities and
    // attitude are invariant under a pure translation.
    {
      std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
      this->state.p += dp;
      this->geo.prev_p += dp;
      ++this->geo.update_seq;  // discard any in-flight propagateState computations
    }
  }
}

void gicp_plusplus::LocalizationNode::deskewPointcloud() {

  // [P2 FIX 2026-07-09] Hold the seed owner lock for the whole deskew phase:
  // basePose/base_pose_stamp_/prev_vel are read (and, via the INS prior,
  // written) throughout, and the reinit writers (applyInitialPose, param
  // pose, RTK full seed) run on other threads. Cost: contended only during a
  // reinit. Lock order: seed -> {geo, imu}; no holder of pose/geo ever takes
  // seed, so no cycle.
  std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);

  // REVIEW FIX: reset the per-frame sweep-span diagnostic up front so early
  // returns (deskew off, unsupported sensor, empty IMU buffer, ...) publish
  // -1 instead of the previous frame's stale span.
  this->last_scan_time_span_s_ = -1.0;

  // INS heading/pose prior — must run BEFORE any IMU integration or cloud
  // placement so the whole prior chain is consistent (see helper above).
  this->applyInsHeadingPriorToBasePose();

  // [REVIEW FIX 2026-07-08] Default validity time for this scan's T_prior /
  // basePose: the scan header stamp. Every fallback branch below (deskew off,
  // unsupported sensor, first scan, empty/short IMU history, integration
  // failure) produces a pose that is best described by the header time; the
  // main deskew path overrides this with the actual median point time.
  this->t_prior_stamp_ = this->scan_stamp.seconds();

  // [REVIEW FIX 2026-07-08 P2] Default: cloud NOT world-frame. Only branches
  // that actually transform/place points into the world set this true.
  // performLocalization() keys its seed/composition on this flag, not deskew_.
  this->scan_in_world_frame_ = false;

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
        this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
      }
    } else {
      this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
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
    // Per-point value is absolute PTP epoch ns (driver reconstruction of the
    // packet-header 48-bit seconds + per-ray 32-bit sub-second nanoseconds;
    // see Luminar Iris Data Output Specification v1.3.0 §2.1 and §2.2/§2.6.3).
    // We deskew on the relative offset (ts - anchor) anchored at the header
    // stamp, so the absolute epoch reference cancels. NOTE: this relies on the
    // driver supplying full epoch ns; a bare 32-bit ns field (sub-second, wraps
    // every 1 s) would make (ts - anchor) jump across a second boundary and
    // corrupt deskew for scans that straddle the rollover.
    //
    // ANCHOR: for a multi-LiDAR merged sweep, anchor on the PRIMARY scan's first
    // timestamp (captured in mergeAuxClouds() BEFORE aux append), NOT the global
    // merged minimum. An aux scan that began before the primary carries smaller
    // epoch timestamps; anchoring at the global min would map that aux point to
    // the header stamp (sweep_ref_time) and shift the entire merged sweep late.
    // Aux points earlier than the primary anchor therefore get correctly NEGATIVE
    // offsets -- which requires SIGNED subtraction below (uint64 underflow
    // otherwise). For the single-primary path we fall back to the global min,
    // which equals the primary min, so behavior is unchanged.
    uint64_t anchor_ts;
    if (this->luminar_primary_min_ts_valid_) {
      anchor_ts = this->luminar_primary_min_ts_ns_;
    } else {
      anchor_ts = std::numeric_limits<uint64_t>::max();
      for (const auto& pt : this->original_scan->points) {
        anchor_ts = std::min(anchor_ts, luminarPointTimestampNs(pt));
      }
    }
    const uint64_t min_ts_captured = anchor_ts;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Luminar scan: anchor_ts=%lu ns (%s), sweep_ref=%.3f s", min_ts_captured,
                         this->luminar_primary_min_ts_valid_ ? "primary" : "global", sweep_ref_time);
    point_time_cmp = [](const PointType& p1, const PointType& p2) {
      return luminarPointTimestampNs(p1) < luminarPointTimestampNs(p2);
    };
    extract_point_time_from_point = [&sweep_ref_time, min_ts_captured](const PointType& pt) {
      const uint64_t ts = luminarPointTimestampNs(pt);
      // Signed difference: aux points earlier than the primary anchor are valid
      // and must produce negative offsets (epoch ns fits in int64_t).
      return sweep_ref_time + static_cast<double>(static_cast<int64_t>(ts) - static_cast<int64_t>(min_ts_captured)) * 1e-9;
    };
    deskew_time_ready = true;
  }

  if (!deskew_time_ready) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unsupported localization/sensor_type for deskew; using scan without motion "
                         "compensation");
    // [REVIEW FIX 2026-07-08 P2] Sensor-frame cloud (scan_in_world_frame_
    // stays false) and a VALID prior for this scan — previously T_prior kept
    // its stale value from the last scan here.
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
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
    this->last_scan_time_span_s_ = -1.0;
    // [REVIEW FIX 2026-07-08 P2] Sensor-frame cloud (scan_in_world_frame_
    // stays false); refresh T_prior and prev_scan_stamp (both previously left
    // stale in this branch).
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    this->current_scan = this->original_scan;
    this->prev_scan_stamp = this->scan_stamp.seconds();
    return;
  }

  // P4#3: per-frame sweep time span of the (merged) cloud. A healthy 3-LiDAR
  // merge spans ~1 sweep period; a much larger span means a badly-offset aux
  // got rebased far from the primary and is being deskewed across a long arc.
  this->last_scan_time_span_s_ = timestamps.back() - timestamps.front();

  // A Luminar sweep that collapses to a single unique timestamp means every point
  // shares one time, so deskew degenerates to a rigid transform (no motion
  // compensation). This is the symptom of a wrong per-point time encoding (e.g.
  // global_shutter/collapsed times) -- warn so the operator can fix the source.
  if (this->sensor == dlio::SensorType::LUMINAR && this->deskew_ && timestamps.size() == 1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Luminar deskew collapsed to a single timestamp (%zu points share one time); "
                         "deskew reduced to a rigid transform. Check the per-point time encoding.",
                         deskewed_scan_->points.size());
  }

  int median_pt_index = timestamps.size() / 2;

  // Don't process scans on first iteration
  if (this->prev_scan_stamp == 0.0) {
    this->prev_scan_stamp = this->scan_stamp.seconds();
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->scan_in_world_frame_ = true;
    return;
  }

  // Check if we have sufficient IMU history
  // We need IMU data from BEFORE prev_scan_stamp to integrate
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (this->imu_buffer.empty()) {
      RCLCPP_WARN(this->get_logger(), "IMU buffer is empty, skipping deskewing");
      // [REVIEW FIX 2026-07-08 P2] Sensor-frame cloud (scan_in_world_frame_
      // stays false); refresh T_prior (previously left stale here).
      this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
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
      this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
      pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
      this->current_scan = deskewed_scan_;
      this->scan_in_world_frame_ = true;
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

  // Clamp timestamps to IMU buffer extent: when per-point timestamps are collapsed
  // (Luminar), sorted_timestamps.back() == scan_stamp which may be a few ms ahead of the
  // latest IMU sample due to callback ordering.  Clamping avoids rejecting the integration.
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
    // [REVIEW FIX 2026-07-08 P3] Snapshot buffer stats under mtx_imu — the
    // IMU callback push_fronts concurrently, so even this log-only read of
    // size()/back() was a data race.
    size_t imu_buf_size = 0;
    double imu_oldest_stamp = 0.0;
    {
      std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
      imu_buf_size = this->imu_buffer.size();
      if (!this->imu_buffer.empty()) imu_oldest_stamp = this->imu_buffer.back().stamp;
    }
    RCLCPP_WARN(this->get_logger(),
                "IMU integration failed! Got %lu frames for %lu timestamps. "
                "Time range: [%.3f, %.3f], IMU buffer size: %lu, first IMU: %.3f",
                frames.size(), timestamps.size(),
                this->prev_scan_stamp, timestamps.back(),
                imu_buf_size, imu_oldest_stamp);
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->scan_in_world_frame_ = true;
    this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Deskewing OK: %lu frames, scan time [%.3f, %.3f]",
                       frames.size(), timestamps.front(), timestamps.back());

  // Update prior to be the estimated pose at the median time of the scan
  this->T_prior = frames[median_pt_index];
  // [REVIEW FIX 2026-07-08] ...and record that time: basePose inherits it via
  // the accept/reject paths in performLocalization, and the INS heading prior
  // queries the INS buffer at exactly this stamp on the next scan.
  this->t_prior_stamp_ = timestamps[median_pt_index];

  // Deskew each point using its timestamp
  #pragma omp parallel for
  for (size_t i = 0; i < timestamps.size(); i++) {
    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;

    // Transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
      auto &pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }

  this->current_scan = deskewed_scan_;
  this->scan_in_world_frame_ = true;
  this->prev_scan_stamp = this->scan_stamp.seconds();
}

// Sensor-frame crop box. Applied to the raw lidar-frame cloud BEFORE deskew (see
// the call site in the scan handler). The box is axis-aligned ±crop_size_ around
// the lidar origin -- a near/far-field filter. It must NOT run after deskew,
// where points are in the world frame and the box would be centered on the map
// origin (clipping the whole scan far from origin).
void gicp_plusplus::LocalizationNode::cropBoxFilterSensorFrame(pcl::PointCloud<PointType>::Ptr& cloud) {
  if (!cloud || cloud->points.empty()) return;
  if (this->crop_size_ > 0.0 && this->crop_size_ < 1000.0) {  // Only apply if reasonable size
    const size_t original_size = cloud->points.size();
    pcl::CropBox<PointType> crop;
    crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
    crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));
    crop.setInputCloud(cloud);
    crop.filter(*cloud);
    RCLCPP_DEBUG(this->get_logger(), "Crop box (sensor frame): %lu -> %lu points", original_size, cloud->points.size());
  }
}

void gicp_plusplus::LocalizationNode::preprocessPointCloud(pcl::PointCloud<PointType>::Ptr& cloud) {

  size_t original_size = cloud->points.size();
  (void)original_size;

  // NOTE: the crop box is intentionally NOT applied here. After deskew the cloud
  // is in the world frame, so an origin-centered box would clip the scan far from
  // the map origin. Cropping happens in cropBoxFilterSensorFrame() before deskew.

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

void gicp_plusplus::LocalizationNode::performLocalization() {

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

  // When the cloud was actually placed in the world frame (main deskew path
  // and the transforming fallbacks), GICP's initial guess is Identity and the
  // final pose = T_corr * T_prior. Otherwise the points are still in the
  // lidar frame, so seed/solve in map<-lidar and convert the optimizer output
  // back to map<-base. [REVIEW FIX 2026-07-08 P2] Keyed on the per-scan
  // scan_in_world_frame_ flag set by deskewPointcloud(), NOT on deskew_:
  // several deskew fallback branches return a sensor-frame cloud while
  // deskew_ is true, and registering that cloud under the world-frame
  // assumption produced wrong-basin candidates at startup / IMU gaps.
  const Eigen::Matrix4f T_base_lidar = this->extrinsics.baselink2lidar_T;
  const Eigen::Matrix4f T_lidar_base = T_base_lidar.inverse();
  // PR#6: plain if-assignment instead of a ternary mixing two different
  // Eigen expression types (CwiseNullaryOp vs Product) — the ternary broke
  // package builds and had to be hot-patched in every replay worktree.
  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  if (!this->scan_in_world_frame_) {
    initial_guess = this->T_prior * T_base_lidar;
  }
  Eigen::Matrix4f guess_pose_map = this->T_prior;

  double guess_from_last_trans = 0.0;
  double guess_from_last_rot_deg = 0.0;
  if (this->last_gicp_valid_) {
    guess_from_last_trans = deltaTranslationNorm(this->last_gicp_pose_, guess_pose_map);
    guess_from_last_rot_deg = rotationDistanceDeg(this->last_gicp_pose_, guess_pose_map);
  }

  // ---- Ground-vehicle registration constraints (yaw-defect fix) ----
  // DoF mask: fix roll/pitch (4dof) or full attitude (3dof) to the initial
  // guess INSIDE the optimizer — the IMU prior attitude cannot be tens of
  // degrees wrong over one scan gap, so wrong-basin attitude updates are
  // removed at the source instead of gated after the fact. Every
  // full6dofEveryN-th scan runs unconstrained so roll/pitch re-anchor to the
  // map (bounds slow gyro-drift accumulation in the fixed axes).
  bool dof_full6_this_scan = (this->gicp_dof_mode_ == "6dof");
  if (!dof_full6_this_scan && this->gicp_full6dof_every_n_ > 0 &&
      (++this->dof_scan_counter_ % this->gicp_full6dof_every_n_) == 0) {
    dof_full6_this_scan = true;
  }
  const bool fix_rp = !dof_full6_this_scan &&
      (this->gicp_dof_mode_ == "4dof" || this->gicp_dof_mode_ == "3dof");
  const bool fix_yaw_dof = !dof_full6_this_scan && this->gicp_dof_mode_ == "3dof";
  this->gicp.setDoFMask(fix_rp, fix_rp, fix_yaw_dof);
  // Soft attitude prior toward the IMU-integrated initial guess (P1 deep
  // cause: the optimizer had NO IMU attitude term, so repeated map structure
  // could pull yaw into a plausible wrong basin unopposed).
  if (this->gicp_prior_yaw_info_ > 0.0 || this->gicp_prior_rollpitch_info_ > 0.0) {
    const Eigen::Matrix3d R_target =
        initial_guess.block<3, 3>(0, 0).cast<double>();
    this->gicp.setRotationPrior(
        R_target, Eigen::Vector3d(this->gicp_prior_rollpitch_info_,
                                  this->gicp_prior_rollpitch_info_,
                                  this->gicp_prior_yaw_info_));
  } else {
    this->gicp.clearRotationPrior();
  }

  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Starting GICP alignment...");
  auto start = std::chrono::high_resolution_clock::now();
  this->gicp.align(aligned_scratch, initial_guess);
  auto end = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: GICP alignment completed");

  double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

  double fitness_score = this->gicp.getFitnessScore();
  bool converged_precheck = this->gicp.hasConverged();
  double fitness_score_final = fitness_score;
  if (!converged_precheck) {
    // Yaw-defect fix (P2a): the cached score reflects the LAST LINEARIZATION
    // pose; on non-converged scans that lags the applied pose by one LM step,
    // and the "effectively converged" acceptance below would judge a bad
    // final candidate by a stale, flattering number. Recompute honestly at
    // the final pose (one kd-tree pass) and gate on the worse of the two.
    fitness_score_final = this->gicp.getFitnessScoreAtFinal();
    fitness_score = std::max(fitness_score, fitness_score_final);
  }
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
  const Eigen::Matrix4f candidate_pose = this->scan_in_world_frame_
      ? (optimizer_solution * this->T_prior)
      : (optimizer_solution * T_lidar_base);
  const bool candidate_pose_valid = matrixFinite(candidate_pose);

  // Marginal yaw stiffness (yaw-defect diagnostic): Schur complement of the
  // vehicle-re-centered hessian w.r.t. everything except yaw — the
  // information THIS SCAN actually carries about vehicle heading after the
  // other 5 DoF absorb what they can. Low values = the map geometry barely
  // constrains yaw here (wrong-basin risk); also the calibration reference
  // for gicp/prior/yawInfo (start at ~0.2x the run median).
  double yaw_marginal_stiffness = -1.0;
  if (candidate_pose_valid && final_hessian.allFinite() && matrixFinite(this->T_prior)) {
    const Eigen::Matrix<double, 6, 6> H_sym = 0.5 * (final_hessian + final_hessian.transpose());
    const Eigen::Vector3d c = this->T_prior.block<3, 1>(0, 3).cast<double>();
    Eigen::Matrix3d skew_c;
    skew_c << 0.0, -c.z(), c.y(), c.z(), 0.0, -c.x(), -c.y(), c.x(), 0.0;
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    A.block<3, 3>(3, 0) = skew_c;
    const Eigen::Matrix<double, 6, 6> H_c = A.transpose() * H_sym * A;
    const int o_idx[5] = {0, 1, 3, 4, 5};  // everything but yaw (index 2)
    Eigen::Matrix<double, 5, 5> Hoo;
    Eigen::Matrix<double, 5, 1> Hoy;
    for (int r = 0; r < 5; ++r) {
      Hoy(r) = H_c(o_idx[r], 2);
      for (int col = 0; col < 5; ++col) Hoo(r, col) = H_c(o_idx[r], o_idx[col]);
    }
    const Eigen::LDLT<Eigen::Matrix<double, 5, 5>> ldlt(Hoo);
    if (ldlt.info() == Eigen::Success) {
      const double schur = H_c(2, 2) - Hoy.dot(ldlt.solve(Hoy));
      if (std::isfinite(schur)) yaw_marginal_stiffness = std::max(0.0, schur);
    }
  }

  // --- P1: per-map fitness baseline (rolling median of accepted-frame fitness).
  // Absolute fitness thresholds calibrated on a same-run map (floor 0.03-0.06)
  // are meaningless on a cross-run map (floor ~0.27); gates below operate on
  // fitness_ratio = fitness / baseline. ratio stays -1 (gates inert) until
  // minSamples accepted frames have been observed.
  double fitness_baseline = -1.0;
  double fitness_ratio = -1.0;
  if (this->fitness_baseline_enable_ &&
      static_cast<int>(this->fitness_history_.size()) >= this->fitness_baseline_min_samples_) {
    std::vector<double> tmp(this->fitness_history_.begin(), this->fitness_history_.end());
    const size_t mid = tmp.size() / 2;
    std::nth_element(tmp.begin(), tmp.begin() + mid, tmp.end());
    fitness_baseline = tmp[mid];
  } else if (this->fitness_baseline_enable_ && this->fitness_baseline_seed_ > 0.0) {
    // Warm-up seed (review fix): without it the ratio gates and yaw veto are
    // inert for the first minSamples accepted frames — a replay that starts
    // mid-turn is unprotected exactly when it is most fragile. Seed with the
    // expected per-map floor (read it off the previous run's scorecard,
    // scripts/analyze_scan_debug_log.py); the rolling median takes over once
    // warmed up.
    fitness_baseline = this->fitness_baseline_seed_;
  }
  if (fitness_baseline > 1e-9 && std::isfinite(fitness_score)) {
    fitness_ratio = fitness_score / fitness_baseline;
  }

  // --- P1: degeneracy-aware partial update + yaw-consistency veto.
  // Eigen-projection engages when the (calibrated) condition proxy crosses
  // hessianCondMax; the yaw veto engages independently on low-confidence
  // matches (fitness_ratio above yawGate/fitnessRatio). Either way the scan is
  // not binary-rejected: the correction is projected and the IMU prior kept
  // along untrusted directions.
  const bool eigen_projection_wanted =
      this->degen_partial_update_enable_ && candidate_pose_valid &&
      this->gicp_hessian_cond_max_ > 0.0 && std::isfinite(hessian_condition) &&
      hessian_condition > this->gicp_hessian_cond_max_;
  // NOTE: yawGate is deliberately INDEPENDENT of degeneracy/partialUpdate —
  // disabling partial updates (legacy binary hessian gate) must not silently
  // disable the yaw-consistency veto.
  //
  // TWO-TIER YAW VETO (P1 yaw-safety fix, runs 19/20: raw GICP rotation
  // proposals up to 45-47 deg were accepted with plausible fitness, producing
  // 78-98 deg heading errors vs RTK):
  //   * SOFT tier (yawGate/maxCorrDeg, default 1.5 deg): armed only on
  //     low-confidence matches (fitness_ratio > yawGate/fitnessRatio) — the
  //     original wrong-basin ENTRY veto.
  //   * HARD tier (yawGate/hardMaxCorrDeg, default 8 deg): UNCONDITIONAL,
  //     independent of fitness ratio. T_prior carries the IMU-integrated yaw,
  //     whose error over one scan gap is <0.1 deg; a ground vehicle cannot
  //     make the IMU wrong by 8+ deg in 0.1-0.3 s, so any such GICP yaw
  //     "correction" is a wrong basin regardless of how plausible its fitness
  //     looks. The yaw component is dropped (IMU yaw kept); translation is
  //     still gated separately below.
  const bool yaw_soft_armed =
      candidate_pose_valid &&
      this->yaw_gate_enable_ && fitness_ratio > 0.0 &&
      fitness_ratio > this->yaw_gate_fitness_ratio_;
  const bool yaw_hard_armed =
      candidate_pose_valid && this->yaw_gate_enable_ &&
      this->yaw_gate_hard_max_corr_deg_ > 0.0;
  double yaw_veto_threshold_deg = 0.0;  // 0 = veto disabled in the projector
  if (yaw_soft_armed) {
    yaw_veto_threshold_deg = this->yaw_gate_max_corr_deg_;
  } else if (yaw_hard_armed) {
    yaw_veto_threshold_deg = this->yaw_gate_hard_max_corr_deg_;
  }
  const bool yaw_veto_wanted = yaw_soft_armed || yaw_hard_armed;
  DegeneracyProjection degen;
  degen.projected_pose = candidate_pose;
  if (eigen_projection_wanted || yaw_veto_wanted) {
    degen = projectDegenerateDelta(final_hessian, this->T_prior, candidate_pose,
                                   eigen_projection_wanted,
                                   this->degen_full6d_, this->degen_coupling_length_m_,
                                   this->degen_rel_floor_6d_,
                                   this->degen_rel_floor_rot_, this->degen_rel_floor_trans_,
                                   yaw_veto_threshold_deg);
  }
  // REVIEW FIX: require the projected pose to be finite before adopting it —
  // final_candidate reaches basePose/current_pose ahead of the downstream
  // gicp_valid finiteness guard, so a (pathological) NaN from the projection
  // would otherwise poison the next scan's prior.
  const Eigen::Matrix4f final_candidate =
      (degen.valid && degen.modified && matrixFinite(degen.projected_pose))
          ? degen.projected_pose : candidate_pose;

  // [REVIEW FIX 2026-07-08 P1] When the pose being APPLIED differs from the
  // optimizer solution (degeneracy projection and/or yaw veto modified it),
  // re-validate fitness and correspondence support AT final_candidate. The
  // raw optimizer fitness describes a pose nobody is applying: a wrong-basin
  // solution with plausible raw fitness could have its yaw vetoed and still
  // land its translation unvalidated. One extra kd-tree/linearize pass, and
  // only on modified (degenerate/vetoed) frames. If the modified pose cannot
  // be evaluated at all, fail safe: force the fitness gate to reject.
  int support_corr = num_correspondences;
  double support_ratio = correspondence_ratio;
  bool fitness_reevaluated = false;
  if (candidate_pose_valid && degen.valid && degen.modified &&
      matrixFinite(final_candidate)) {
    const Eigen::Matrix4f T_eval = this->scan_in_world_frame_
        ? Eigen::Matrix4f(final_candidate * this->T_prior.inverse())
        : Eigen::Matrix4f(final_candidate * T_base_lidar);
    double fit_applied = std::numeric_limits<double>::quiet_NaN();
    int corr_applied = 0;
    fitness_reevaluated = true;
    if (this->gicp.evaluateFitnessAt(T_eval, &fit_applied, &corr_applied)) {
      fitness_score = fit_applied;
      this->last_fitness_score_ = fitness_score;
      support_corr = corr_applied;
      support_ratio = this->current_scan->points.empty()
          ? 0.0
          : static_cast<double>(corr_applied) /
                static_cast<double>(this->current_scan->points.size());
      if (fitness_baseline > 1e-9 && std::isfinite(fitness_score)) {
        fitness_ratio = fitness_score / fitness_baseline;
      }
    } else {
      fitness_score = std::numeric_limits<double>::infinity();
      this->last_fitness_score_ = fitness_score;
      support_corr = 0;
      support_ratio = 0.0;
    }
  }

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
  // Speed/scan_dt-aware jump thresholds (P2#2). The jump is GICP-vs-IMU-prior, so
  // the legitimate disagreement scales with how far the prior could have drifted:
  // ~speed*scan_dt for translation and ~scan_dt for rotation (scan_dt here is the
  // time since the last accepted pose, so it grows during a failure streak). This
  // loosens the gate after a gap / at high speed so a valid reacquisition is not
  // rejected, while keeping it tight for slow, short-gap scans. Setting the scales
  // to 0 reproduces the fixed thresholds.
  float speed_est = 0.0f;
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    speed_est = this->state.v.lin.w.norm();
  }
  const double scan_dt_clamped = std::min(std::max(scan_dt, 0.0), 1.0);
  const double eff_jump_trans_m = this->debug_jump_trans_m_ +
      this->jump_trans_speed_scale_ * static_cast<double>(speed_est) * scan_dt_clamped;
  const double eff_jump_rot_deg = this->debug_jump_rot_deg_ +
      this->jump_rot_dt_scale_deg_ * scan_dt_clamped;
  const bool large_jump = candidate_pose_valid &&
                          (jump_trans > eff_jump_trans_m || jump_rot_deg > eff_jump_rot_deg);
  // The jump GATE evaluates the pose that would actually be applied. A partial
  // update can only shrink the GICP-vs-prior delta, so this is never looser
  // than the raw-candidate check; raw jump_trans/jump_rot_deg stay in the
  // debug topics/log as the unprojected disagreement signal.
  double final_jump_trans = jump_trans;
  double final_jump_rot_deg = jump_rot_deg;
  if (candidate_pose_valid && degen.valid && degen.modified) {
    final_jump_trans = deltaTranslationNorm(this->T_prior, final_candidate);
    final_jump_rot_deg = rotationDistanceDeg(this->T_prior, final_candidate);
  }
  const bool large_jump_final = candidate_pose_valid &&
      (final_jump_trans > eff_jump_trans_m || final_jump_rot_deg > eff_jump_rot_deg);

  // P1 yaw-safety fix #2: yaw split out of the 3D rotation jump gate. The
  // generic gate (30 deg + 60 deg/s * scan_dt) exists to admit re-acquisition
  // after gaps, but for YAW that envelope is physically absurd on a ground
  // vehicle — the IMU-integrated prior cannot be tens of degrees wrong over a
  // 0.1-0.3 s gap (runs 19/20: 45-47 deg proposals passed the ~36-45 deg
  // effective gate and produced 78-98 deg heading errors). Yaw innovation vs
  // T_prior gets its own tight budget with a hard absolute cap that scan_dt
  // scaling can never raise into relocalization territory:
  //   eff_yaw = min(yaw_max_deg + yaw_dt_scale_deg * scan_dt, yaw_total_max_deg)
  // Evaluated on the APPLIED pose (post yaw-veto/projection), so a vetoed
  // frame — whose yaw already equals the IMU prior — passes and keeps its
  // translation information. Controlled relocalization stays available via
  // GT snap recovery, which bypasses scan gates by design.
  const double yaw_innov_raw_deg =
      candidate_pose_valid ? yawInnovationDeg(this->T_prior, candidate_pose) : -1.0;
  const double yaw_innov_final_deg =
      candidate_pose_valid ? ((degen.valid && degen.modified)
                                  ? yawInnovationDeg(this->T_prior, final_candidate)
                                  : yaw_innov_raw_deg)
                           : -1.0;
  const double eff_yaw_max_deg =
      (this->jump_yaw_max_deg_ > 0.0)
          ? std::min(this->jump_yaw_max_deg_ + this->jump_yaw_dt_scale_deg_ * scan_dt_clamped,
                     this->jump_yaw_total_max_deg_ > 0.0 ? this->jump_yaw_total_max_deg_
                                                         : std::numeric_limits<double>::infinity())
          : std::numeric_limits<double>::infinity();
  const bool yaw_jump_final = candidate_pose_valid && std::isfinite(eff_yaw_max_deg) &&
                              yaw_innov_final_deg > eff_yaw_max_deg;

  // PR#6: BOUNDED non-converged fitness fallback. The "effectively converged"
  // rule (non-converged but fitness under the absolute threshold) exists for
  // the max-iterations-at-highway-speed case, where the correction is small.
  // Run5-on-Run3 showed it also admitting wrong-basin results with corrections
  // of 3.27 m/11.2 deg and 9.21 m/10.7 deg — and each accept reset the
  // consecutive-failure counter, locking GT recovery out until the pose had
  // drifted far from INS. Keep the fallback only for SMALL corrections
  // (vs the IMU prior, on the pose actually applied); larger non-converged
  // candidates are classified failed_to_converge. <=0 disables either bound.
  // PR-validated: nonconv accepts 1145->397, accepts with INS err >=50 m 99->3.
  const bool nonconv_fallback_ok =
      candidate_pose_valid && fitness_score <= this->gicp_fitness_reject_threshold_ &&
      (this->gicp_nonconv_ok_max_trans_m_ <= 0.0 ||
       final_jump_trans <= this->gicp_nonconv_ok_max_trans_m_) &&
      (this->gicp_nonconv_ok_max_rot_deg_ <= 0.0 ||
       final_jump_rot_deg <= this->gicp_nonconv_ok_max_rot_deg_);
  const bool effectively_converged = converged || nonconv_fallback_ok;

  // Independent stateful wrong-basin check. The shadow was anchored by the
  // already-existing odom-init / GT-snap path, then propagated only with Atlas
  // velocity + orientation. It is intentionally evaluated on the pose that
  // would actually be applied (post projection/yaw veto).
  double velocity_shadow_error_m = std::numeric_limits<double>::quiet_NaN();
  double velocity_shadow_age_s = std::numeric_limits<double>::quiet_NaN();
  double velocity_shadow_anchor_age_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t velocity_shadow_reset_count = 0;
  bool velocity_shadow_available = false;
  bool velocity_shadow_fail_closed_if_unavailable = false;
  Eigen::Vector3f velocity_shadow_p = Eigen::Vector3f::Zero();
  const double velocity_shadow_query_stamp =
      this->t_prior_stamp_ > 0.0 ? this->t_prior_stamp_ : this->scan_stamp.seconds();
  if (candidate_pose_valid && this->velocityShadowAt(
          velocity_shadow_query_stamp, velocity_shadow_p,
          velocity_shadow_age_s, velocity_shadow_anchor_age_s,
          velocity_shadow_reset_count,
          velocity_shadow_fail_closed_if_unavailable)) {
    velocity_shadow_available = true;
    const Eigen::Vector2d velocity_shadow_delta =
        (final_candidate.block<2, 1>(0, 3) -
         velocity_shadow_p.head<2>()).cast<double>();
    velocity_shadow_error_m = velocity_shadow_delta.norm();
  }
  if (this->velocity_shadow_enabled_ && !velocity_shadow_available &&
      std::isfinite(velocity_shadow_anchor_age_s) &&
      velocity_shadow_anchor_age_s > this->velocity_shadow_max_anchor_age_s_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Velocity shadow: anchor age %.1fs exceeds %.1fs; watchdog unavailable until next explicit GT snap/seed (%s)",
        velocity_shadow_anchor_age_s, this->velocity_shadow_max_anchor_age_s_,
        velocity_shadow_fail_closed_if_unavailable ? "fail-closed" : "fail-open");
  }
  const bool velocity_shadow_divergence_reject_wanted =
      this->velocity_shadow_enabled_ && velocity_shadow_available &&
      (!std::isfinite(velocity_shadow_error_m) ||
       velocity_shadow_error_m > this->velocity_shadow_max_horizontal_divergence_m_);
  const bool velocity_shadow_unavailable_reject_wanted =
      this->velocity_shadow_enabled_ && !velocity_shadow_available &&
      velocity_shadow_fail_closed_if_unavailable;
  const bool velocity_shadow_reject_wanted =
      velocity_shadow_divergence_reject_wanted ||
      velocity_shadow_unavailable_reject_wanted;

  // Ground-truth divergence cross-check (optional). Compares the scan's accepted-or-candidate
  // pose to a time-matched ground-truth odom sample. Only computes; does NOT influence
  // accept/reject decisions — purely a diagnostic.
  double gt_pos_err = -1.0;
  double gt_rot_err_deg = -1.0;
  double gt_dt = 0.0;
  if (this->gt_odom_enabled_ && this->gt_odom_received_.load() && candidate_pose_valid) {
    GtSample gt;
    // Cross-check is a CM-LEVEL DIAGNOSTIC -- only meaningful against
    // RTK-FIXED-quality Atlas samples. Snap-recovery and IMU dead-reckoning
    // do NOT participate in this gate; they accept Atlas dead-reckoning
    // quality as the next-best truth.
    if (this->getGtPoseAt(this->scan_stamp.seconds(), gt) && this->gtSampleIsRtkFixed(gt)) {
      // Evaluate the pose that would actually be APPLIED (post degeneracy
      // projection), so run-report gt_err statistics describe the output.
      const Eigen::Vector3f cand_p = final_candidate.block<3, 1>(0, 3);
      const Eigen::Quaternionf cand_q(Eigen::Matrix3f(final_candidate.block<3, 3>(0, 0)));
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
    publish_float(this->dbg_fitness_ratio_pub, fitness_ratio);
    publish_float(this->dbg_degen_rot_axes_pub, static_cast<double>(degen.degen_rot_axes));
    publish_float(this->dbg_degen_trans_axes_pub, static_cast<double>(degen.degen_trans_axes));
    publish_float(this->dbg_yaw_veto_pub, degen.yaw_vetoed ? 1.0 : 0.0);
    // P1 yaw-safety: raw GICP-vs-IMU yaw disagreement (pre-veto/projection).
    publish_float(this->dbg_yaw_innovation_pub, yaw_innov_raw_deg);
    publish_float(this->dbg_yaw_stiffness_pub, yaw_marginal_stiffness);
    // INS-vs-prior yaw difference at this scan's integration seed (NaN when
    // the INS prior did not run). Persistent nonzero = map-vs-ENU yaw
    // misalignment or INS heading fault — measure, don't just blend harder.
    publish_float(this->dbg_ins_yaw_diff_pub, this->last_ins_yaw_diff_deg_);
    publish_float(this->dbg_velocity_shadow_err_pub, velocity_shadow_error_m);
    publish_float(this->dbg_velocity_shadow_age_pub, velocity_shadow_age_s);
    publish_float(this->dbg_velocity_shadow_anchor_age_pub, velocity_shadow_anchor_age_s);
    std_msgs::msg::Bool velocity_shadow_available_msg;
    velocity_shadow_available_msg.data = velocity_shadow_available;
    this->dbg_velocity_shadow_available_pub->publish(velocity_shadow_available_msg);
    // P4#3: per-frame concat/source-set record (merged_aux_count = -1 when
    // concat disabled; aux dt = NaN when that aux did not merge this frame).
    publish_float(this->dbg_merged_aux_count_pub,
                  static_cast<double>(this->concat_last_merged_aux_));
    publish_float(this->dbg_scan_time_span_pub, this->last_scan_time_span_s_);
    for (size_t i = 0; i < this->dbg_aux_dt_pubs_.size(); ++i) {
      const double dt_i = (i < this->concat_last_aux_dt_.size())
          ? this->concat_last_aux_dt_[i] : std::numeric_limits<double>::quiet_NaN();
      const double pts_i = (i < this->concat_last_aux_points_.size())
          ? static_cast<double>(this->concat_last_aux_points_[i]) : 0.0;
      publish_float(this->dbg_aux_dt_pubs_[i], dt_i);
      publish_float(this->dbg_aux_points_pubs_[i], pts_i);
    }

    std_msgs::msg::Bool converged_msg;
    // PR#6: publish the same BOUNDED effective-convergence decision the
    // acceptance gate uses (was the unbounded fitness-only fallback).
    converged_msg.data = effectively_converged && candidate_pose_valid;
    this->dbg_converged_pub->publish(converged_msg);

    if (gt_pos_err >= 0.0) {
      publish_float(this->dbg_gt_pos_err_pub, gt_pos_err);
      publish_float(this->dbg_gt_rot_deg_pub, gt_rot_err_deg);
    }

    this->dbg_initial_guess_pose_pub->publish(
        poseStampedFromMatrix(guess_pose_map, this->scan_stamp, this->map_frame));
    if (candidate_pose_valid) {
      // Post-projection pose (what would be applied); raw disagreement is
      // still visible via jump_trans/jump_rot_deg topics.
      this->dbg_final_pose_pub->publish(
          poseStampedFromMatrix(final_candidate, this->scan_stamp, this->map_frame));
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
        << " fit_ratio=" << scalarSummary(fitness_ratio, 3)
        << " degen=[r" << degen.degen_rot_axes << ",t" << degen.degen_trans_axes
        << ",yaw_veto=" << (degen.yaw_vetoed ? 1 : 0)
        << ",partial=" << ((degen.valid && degen.modified) ? 1 : 0) << "]"
        << " final_error=" << scalarSummary(final_error, 6)
        << " correspondences=" << num_correspondences << "/" << this->current_scan->points.size()
        << " ratio=" << scalarSummary(correspondence_ratio, 3)
        << " support=[" << support_corr << "," << scalarSummary(support_ratio, 3)
        << ",reeval=" << (fitness_reevaluated ? 1 : 0) << "]"
        << " guess_to_solution=[" << scalarSummary(guess_to_solution_trans) << "m,"
        << scalarSummary(guess_to_solution_rot_deg) << "deg]"
        << " jump=[" << scalarSummary(jump_trans) << "m," << scalarSummary(jump_rot_deg) << "deg]"
        << " yaw_innov=[" << scalarSummary(yaw_innov_raw_deg, 2) << "deg,fin="
        << scalarSummary(yaw_innov_final_deg, 2) << "deg]"
        << " yaw_stiff=" << scalarSummary(yaw_marginal_stiffness, 1)
        << " ins_dyaw=" << scalarSummary(this->last_ins_yaw_diff_deg_, 2) << "deg"
        << " velocity_shadow=[available=" << (velocity_shadow_available ? 1 : 0)
        << ",err_xy=" << scalarSummary(velocity_shadow_error_m, 3)
        << "m,thr=" << scalarSummary(this->velocity_shadow_max_horizontal_divergence_m_, 3)
        << "m,age=" << scalarSummary(velocity_shadow_age_s, 3)
        << "s,anchor_age=" << scalarSummary(velocity_shadow_anchor_age_s, 3)
        << "s,resets=" << velocity_shadow_reset_count << "]"
        << " imu_buffer_span=" << scalarSummary(imu_buffer_span) << "s"
        << " scan_to_latest_imu_lag=" << scalarSummary(scan_to_latest_imu_lag) << "s"
        << " concat=[" << this->concat_last_merged_aux_ << "/" << this->aux_lidars_.size();
    for (size_t i = 0; i < this->concat_last_aux_dt_.size(); ++i) {
      oss << ",dt" << i << "=" << scalarSummary(this->concat_last_aux_dt_[i], 3)
          << "s,pts" << i << "=" << this->concat_last_aux_points_[i];
    }
    oss << ",span=" << scalarSummary(this->last_scan_time_span_s_, 3) << "s]"
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

  // effectively_converged is computed above (PR#6 bounded fallback) so the
  // debug `converged` topic publishes the same decision the gate uses.

  bool gicp_rejected_fitness = false;
  bool gicp_rejected_fitness_ratio = false;
  bool gicp_rejected_jump = false;
  bool gicp_rejected_yaw = false;
  bool gicp_rejected_hessian = false;
  bool gicp_rejected_support = false;
  bool gicp_rejected_velocity_shadow = false;
  bool gicp_rejected_velocity_shadow_unavailable = false;
  if (effectively_converged && candidate_pose_valid) {
    if (!final_hessian.allFinite()) {
      // [REVIEW FIX 2026-07-08 P3] Non-finite Hessian: hessianConditionProxy
      // returns +inf, but every Hessian gate below requires
      // std::isfinite(hessian_condition) — so these scans previously skipped
      // Hessian protection entirely. A candidate whose curvature cannot even
      // be evaluated offers nothing to trust or project: reject it and hold
      // the IMU/INS prior.
      gicp_rejected_hessian = true;
    } else if ((this->gicp_min_correspondences_ > 0 &&
                support_corr < this->gicp_min_correspondences_) ||
               (this->gicp_min_corr_ratio_ > 0.0 &&
                support_ratio < this->gicp_min_corr_ratio_)) {
      // [REVIEW FIX 2026-07-08 P3] Minimum-support gate: mean fitness over a
      // HANDFUL of inliers can look excellent while the scan barely overlaps
      // the map (require_all_aux=false front-only frames, crop/blockage).
      // Support is evaluated at the APPLIED pose when it was re-scored above.
      gicp_rejected_support = true;
    } else if (!std::isfinite(fitness_score) ||
               fitness_score > this->gicp_fitness_reject_threshold_) {
      // [REVIEW FIX 2026-07-08 P2] NaN fitness fails CLOSED: `NaN > x` is
      // false, so without this check a NaN score sailed through every
      // fitness comparison and the scan was accepted unvalidated.
      gicp_rejected_fitness = true;
    } else if (yaw_jump_final) {
      // P1 yaw-safety: hard yaw innovation gate vs the IMU prior, checked
      // BEFORE the degeneracy/jump paths and independent of fitness. Only
      // reachable when the (unconditional) yaw veto is disabled or failed to
      // engage — the vetoed pose carries IMU yaw and passes this gate.
      gicp_rejected_yaw = true;
    } else if (this->fitness_ratio_reject_ > 0.0 && fitness_ratio > 0.0 &&
               fitness_ratio > this->fitness_ratio_reject_) {
      // Wrong-basin gate (P1): fitness far above the map's own rolling-median
      // baseline is the bad-accept signature (run-12: bad accepts at gt_err>20m
      // had ratio median 1.34 / p90 2.55 while good accepts sat at 1.00). A
      // wrong-basin solution must not be partially applied either — fall back
      // to the IMU prior entirely.
      gicp_rejected_fitness_ratio = true;
    } else if (velocity_shadow_reject_wanted) {
      // Unlike the per-frame jump gate, this stays independent when a series
      // of individually-small accepted corrections drags both current_pose and
      // T_prior into a coherent but globally wrong basin.
      if (velocity_shadow_unavailable_reject_wanted) {
        gicp_rejected_velocity_shadow_unavailable = true;
      } else {
        gicp_rejected_velocity_shadow = true;
      }
    } else if (this->degen_partial_update_enable_) {
      // P1 partial-update path: degenerate geometry no longer rejects the scan
      // (the old binary gate produced 253-frame dead-reckoning streaks when the
      // absolute fitness warn threshold went stale on a cross-run map). The
      // correction has already been projected onto the well-constrained
      // eigen-subspace above; reject only when the analysis says NOTHING is
      // trustworthy (all axes degenerate / hessian not analyzable).
      if (eigen_projection_wanted && (!degen.valid || degen.fully_degenerate)) {
        gicp_rejected_hessian = true;
      } else if (this->gicp_reject_large_jumps_ && large_jump_final) {
        gicp_rejected_jump = true;
      }
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
      // LEGACY combined geometric degeneracy gate (degeneracy/partialUpdate:
      // false). High hessian condition alone is harmless when the IMU prior
      // was already good and GICP barely moved; reject only when degenerate
      // AND a slide signal fires. NOTE: the fitness/trans/rot warn thresholds
      // here are ABSOLUTE and calibrated per-map — on cross-run maps they go
      // stale (see docs/action_plan_turn_error_20260704.md), which is why the
      // partial-update path above is the default.
      gicp_rejected_hessian = true;
    } else if (this->gicp_reject_large_jumps_ && large_jump_final) {
      // large_jump_final == large_jump unless the (independent) yaw veto
      // modified the candidate; gate the pose that would actually be applied.
      gicp_rejected_jump = true;
    }
  }
  const bool gicp_accepted = effectively_converged && candidate_pose_valid &&
                             !gicp_rejected_fitness && !gicp_rejected_fitness_ratio &&
                             !gicp_rejected_hessian && !gicp_rejected_jump &&
                             !gicp_rejected_yaw && !gicp_rejected_support &&
                             !gicp_rejected_velocity_shadow &&
                             !gicp_rejected_velocity_shadow_unavailable;
  const bool gicp_partial = gicp_accepted && degen.valid && degen.modified;

  if (!candidate_pose_valid) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("invalid_solution").c_str());
  } else if (!effectively_converged) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("failed_to_converge").c_str());
  } else if (gicp_rejected_fitness) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (fitness=%.4f > threshold=%.4f): %s",
                fitness_score, this->gicp_fitness_reject_threshold_,
                build_scan_debug_log("rejected_fitness").c_str());
  } else if (gicp_rejected_support) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (support: corr=%d ratio=%.3f below min [%d, %.3f]): %s",
                support_corr, support_ratio,
                this->gicp_min_correspondences_, this->gicp_min_corr_ratio_,
                build_scan_debug_log("rejected_support").c_str());
  } else if (gicp_rejected_yaw) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (yaw innovation %.2f deg > eff %.2f deg vs IMU prior @ scan_dt=%.3fs — "
                "physically impossible heading correction for a ground vehicle): %s",
                yaw_innov_final_deg, eff_yaw_max_deg, scan_dt_clamped,
                build_scan_debug_log("rejected_yaw").c_str());
  } else if (gicp_rejected_fitness_ratio) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (fitness_ratio=%.3f > %.3f, baseline=%.4f — wrong-basin signature): %s",
                fitness_ratio, this->fitness_ratio_reject_, fitness_baseline,
                build_scan_debug_log("rejected_fitness_ratio").c_str());
  } else if (gicp_rejected_velocity_shadow_unavailable) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (velocity shadow unavailable after a known-pose anchor; fail-closed, age=%+.3fs anchor_age=%.3fs resets=%lu): %s",
                velocity_shadow_age_s, velocity_shadow_anchor_age_s,
                static_cast<unsigned long>(velocity_shadow_reset_count),
                build_scan_debug_log("rejected_velocity_shadow_unavailable").c_str());
  } else if (gicp_rejected_velocity_shadow) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (velocity-shadow divergence %.3fm > %.3fm, age=%+.3fs; Atlas position unused): %s",
                velocity_shadow_error_m, this->velocity_shadow_max_horizontal_divergence_m_,
                velocity_shadow_age_s,
                build_scan_debug_log("rejected_velocity_shadow").c_str());
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
                "GICP REJECTED (jump dT=%.3fm dR=%.2fdeg > eff thresholds [%.2fm, %.2fdeg] @ speed=%.1fm/s scan_dt=%.3fs): %s",
                final_jump_trans, final_jump_rot_deg, eff_jump_trans_m, eff_jump_rot_deg,
                static_cast<double>(speed_est), scan_dt_clamped,
                build_scan_debug_log("rejected_jump").c_str());
  } else if (large_jump && !gicp_partial) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("large_jump").c_str());
  } else if (this->debug_verbose_scan_log_) {
    // "ok_partial" = accepted after degeneracy projection / yaw veto shrank
    // the correction; grep-compatible with "status=ok" prefix matching is NOT
    // preserved on purpose so audits can split the two populations.
    RCLCPP_INFO(this->get_logger(), "%s",
                build_scan_debug_log(gicp_partial ? "ok_partial" : "ok").c_str());
  }

  bool gt_snap_applied_this_frame = false;
  if (gicp_accepted) {
    // P1: apply the (possibly degeneracy-projected) candidate, and feed the
    // per-map fitness baseline from accepted frames only, so wrong-basin /
    // rejected fitness never inflates the baseline the gates divide by.
    this->current_pose = final_candidate;
    if (this->fitness_baseline_enable_ && std::isfinite(fitness_score) && fitness_score >= 0.0) {
      this->fitness_history_.push_back(fitness_score);
      while (static_cast<int>(this->fitness_history_.size()) > this->fitness_baseline_window_) {
        this->fitness_history_.pop_front();
      }
    }

    // Update lidar pose for next iteration
    Eigen::Vector3f new_p = this->current_pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotSO3 = this->current_pose.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotSO3);
    q.normalize();

    {
      // [P2 FIX 2026-07-09] Seed writes under the owner lock (see seed_mtx_
      // in the header): a cross-thread reinit (RViz / RTK full seed) must
      // never interleave with these. Lock order here: pose -> seed.
      std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
      this->basePose.p = new_p;
      this->basePose.q = q;
      // [REVIEW FIX 2026-07-08] The accepted candidate is the pose at the median
      // point time of this scan (the cloud was deskewed to frames[median]).
      this->base_pose_stamp_ = this->t_prior_stamp_;
    }
    // P3: pair the measurement with the IMU prior it was registered against
    // (both at median scan time) so updateState can form the time-free delta.
    this->observer_prior_pose_ = this->T_prior;

    // Validate GICP result before using it
    bool gicp_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) && std::isfinite(new_p.z()) &&
                      std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());

    if (!gicp_valid) {
      RCLCPP_WARN(this->get_logger(), "GICP result contains invalid values, skipping geometric observer update");
    } else {
      // Initialize or update geometric observer
      if (!this->geo.first_opt_done) {
        // First time: initialize state to GICP result.
        // [REVIEW FIX 2026-07-08 P2/P3] Under geo.mtx — IMU callbacks are in a
        // REENTRANT group and propagateState may be mid-flight — and the
        // calibrated IMU biases are PRESERVED: the RTK / stationary
        // calibration paths write state.b.* before the first accepted scan,
        // and unconditionally zeroing them here silently discarded that
        // calibration on non-odom-init setups (use_odom_init=true was immune
        // only because odom init sets first_opt_done, skipping this branch).
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->state.p = new_p;
        this->state.q = q;
        this->state.v.lin.w = Eigen::Vector3f::Zero();
        this->state.v.lin.b = Eigen::Vector3f::Zero();
        this->state.v.ang.w = Eigen::Vector3f::Zero();
        this->state.v.ang.b = Eigen::Vector3f::Zero();
        if (!this->imu_calibrated_.load()) {
          this->state.b.accel = Eigen::Vector3f::Zero();
          this->state.b.gyro = Eigen::Vector3f::Zero();
        }

        // Initialize geo tracking
        this->geo.prev_p = this->state.p;
        this->geo.prev_q = this->state.q;
        this->geo.prev_vel = Eigen::Vector3f::Zero();

        // Mark as initialized
        this->geo.first_opt_done = true;
        ++this->geo.update_seq;  // discard in-flight propagateState computations

        RCLCPP_INFO(this->get_logger(), "Geometric observer initialized to pos=[%.2f,%.2f,%.2f]",
                    new_p.x(), new_p.y(), new_p.z());
      } else {
        // Update geometric observer with GICP measurement (skip on first scan)
        this->updateState();
      }
    }

    // Use geometric observer velocity for next IMU integration
    {
      // [P2 FIX 2026-07-09] seed -> geo (matches deskewPointcloud's order).
      std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
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
      // [REVIEW FIX 2026-07-08 P2] Record the ACCEPTED fitness here (and only
      // here) so the IMU-rate odometry covariance reflects the last accepted
      // scan's quality — not the latest attempt, which may be a rejection
      // with an inflated/+inf score.
      if (std::isfinite(fitness_score) && fitness_score >= 0.0) {
        this->last_accepted_fitness_score_ = fitness_score;
      }
    }

    // Reset consecutive-failure counter on any accepted scan so the GT-recovery
    // trigger only fires on sustained losing streaks.
    this->consecutive_failures_ = 0;
    // Mark this as the last known-good fix; the dead-reckoning covariance growth
    // (P3) measures elapsed time from here.
    this->last_accepted_scan_stamp_ = this->scan_stamp.seconds();

    // Log pose and correction
    Eigen::Vector3f t_corr = optimizer_solution.block<3, 1>(0, 3);
    RCLCPP_INFO(this->get_logger(),
                "Localization: ✓ %s%s | fitness=%.6f | time=%.2fms | "
                "correction=[%.3f, %.3f, %.3f] | pose=[%.2f, %.2f, %.2f]",
                converged ? "CONVERGED" : "ACCEPTED(fitness-ok)",
                gicp_partial ? " [PARTIAL: degenerate axes kept on IMU prior]" : "",
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
                       : gicp_rejected_support ? "insufficient correspondence support"
                       : gicp_rejected_fitness ? "fitness rejected"
                       : gicp_rejected_yaw ? "yaw-innovation rejected (impossible heading)"
                       : gicp_rejected_fitness_ratio ? "fitness-ratio rejected (wrong basin)"
                       : gicp_rejected_velocity_shadow_unavailable ? "velocity-shadow unavailable (fail-closed)"
                       : gicp_rejected_velocity_shadow ? "velocity-shadow divergence rejected"
                       : gicp_rejected_hessian ? "degenerate geometry"
                       : "jump rejected";
    if (matrixFinite(this->T_prior)) {
      this->current_pose = this->T_prior;
      const Eigen::Vector3f new_p = this->T_prior.block<3, 1>(0, 3);
      Eigen::Quaternionf q(this->T_prior.block<3, 3>(0, 0));
      q.normalize();
      // [P2 FIX 2026-07-09] Seed writes under the owner lock (order:
      // pose -> seed -> geo, consistent with the accept path and deskew).
      std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
      this->basePose.p = new_p;
      this->basePose.q = q;
      // [REVIEW FIX 2026-07-08] T_prior is also a median-point-time pose.
      this->base_pose_stamp_ = this->t_prior_stamp_;
      {
        // P2#1 (stale-velocity bug): seed the next scan's IMU integration from
        // the CURRENT IMU-propagated velocity, not geo.prev_vel. geo.prev_vel
        // is only refreshed by updateState() (accepted scans) or a GT snap, so
        // during an N-frame rejection streak it stayed frozen at the last
        // accepted scan's velocity while the vehicle's velocity vector rotated
        // through the turn — every per-scan prior then extrapolated straight
        // ("corner cutting", run-12 webm). state.v.lin.w is maintained at IMU
        // rate by propagateState() and is the correct dead-reckoning velocity.
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->prev_vel = this->state.v.lin.w;
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
    gt_snap_applied_this_frame = this->maybeSnapPoseToGT(reason);
  }

  if (this->debug_pub_enabled_ && this->dbg_snap_applied_pub) {
    std_msgs::msg::Bool snap_msg;
    snap_msg.data = gt_snap_applied_this_frame;
    this->dbg_snap_applied_pub->publish(snap_msg);
  }
}

void gicp_plusplus::LocalizationNode::publishPose() {

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

void gicp_plusplus::LocalizationNode::callbackGtOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
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
  // Carry Atlas-reported position covariance per-sample. The RTK quality
  // gate is no longer applied here -- every sample is pushed into the buffer
  // regardless of FIXED/FLOAT/dead-reckoning state. The gate now runs at
  // the CONSUMER side:
  //   * tryRtkCalibrationStep (init/calibration)  -> require FIXED
  //   * scan cross-check (gt_pos_err diagnostic)  -> require FIXED
  //   * maybeSnapPoseToGT (recovery from GICP failure) -> accept ANY sample
  // Rationale: Atlas's onboard INS already does coupled GNSS+IMU dead-
  // reckoning with calibrated sensors during RTK loss. When GICP fails to
  // match the LiDAR scan, the next-best truth is Atlas's pose at whatever
  // quality it currently has -- not our own software IMU dead-reckoning.
  s.cov_pos_xx = msg->pose.covariance[0];
  s.cov_pos_yy = msg->pose.covariance[7];
  s.cov_pos_zz = msg->pose.covariance[14];
  // [REVIEW FIX 2026-07-08] Carry the yaw variance (rad^2) the adapter
  // publishes in pose.covariance[35] (Atlas rpy covariance, deg^2 -> rad^2).
  // Consumed by the INS heading prior's yaw-quality gate. Generic publishers
  // that leave the field at 0 are treated as "unpopulated" downstream.
  s.cov_yaw = msg->pose.covariance[35];

  // Cache base_frame ← gt_body_frame TF on the first message (mirrors the IMU
  // extrinsic caching pattern in callbackImu). Required before the snap helper
  // and the diagnostic cross-check can compose poses; callback keeps appending
  // samples even while TF is missing.  Runs BEFORE the odom-init block below
  // so composeGtPoseInBase has the extrinsic ready to bring the first GT
  // sample into base_frame coordinates before applyInitialPose seeds the state.
  if (!this->gt_extrinsics_cached_.load()) {
    // [P2 FIX 2026-07-09] Serialize: this callback runs in a REENTRANT group,
    // so two first messages could execute this block concurrently
    // (std::string assignment race = UB, torn T_base_gtbody_ publication).
    // Re-check under the lock; gt_extrinsics_cached_ (atomic) is written LAST.
    std::lock_guard<std::mutex> init_lock(this->gt_init_mtx_);
    if (!this->gt_extrinsics_cached_.load()) {
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
    }  // re-check scope (gt_init_mtx_ held)
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
  if (this->use_odom_init_ && !this->use_odom_init_applied_.load()) {
    // [P2 FIX 2026-07-09] Serialize + re-check: without this, two concurrent
    // GT callbacks could both pass the flag test and run applyInitialPose
    // twice, interleaving their (sequential) lock scopes.
    std::lock_guard<std::mutex> init_lock(this->gt_init_mtx_);
    Eigen::Vector3f init_p;
    Eigen::Quaternionf init_q;
    // Re-check under the lock: a concurrent callback may have seeded while
    // we waited. The && short-circuit skips the compose entirely then.
    if (!this->use_odom_init_applied_.load() && this->composeGtPoseInBase(s, init_p, init_q)) {
      this->use_odom_init_applied_ = true;
      const rclcpp::Time stamp_ros(msg->header.stamp.sec, msg->header.stamp.nanosec);
      this->applyInitialPose(init_p, init_q, stamp_ros, "gt_odom");
      Eigen::Vector3f init_v_lin_body;
      Eigen::Vector3f init_v_ang_body;
      if (this->composeGtTwistInBase(s, init_v_lin_body, init_v_ang_body)) {
        (void)init_v_ang_body;
        this->resetVelocityShadow(init_p, init_q * init_v_lin_body,
                                  s.stamp, "gt_odom init");
      }
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

  bool first_gt = false;
  {
    std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
    if (!this->gt_odom_buffer_.empty() && s.stamp <= this->gt_odom_buffer_.back().stamp) {
      // Out-of-order or duplicate timestamp; drop to keep buffer monotone and
      // do not integrate it into the independent velocity shadow either.
      return;
    }
    this->gt_odom_buffer_.push_back(s);
    while (this->gt_odom_buffer_.size() > this->gt_odom_buffer_size_) {
      this->gt_odom_buffer_.pop_front();
    }
    // The GT callback group is reentrant. Keep monotonic buffer admission and
    // velocity integration in one critical section so t2 cannot integrate
    // ahead of an admitted-but-not-yet-integrated t1 callback.
    this->updateVelocityShadowFromGt(s);
    first_gt = !this->gt_odom_received_.exchange(true);
  }
  if (first_gt) {
    RCLCPP_INFO(this->get_logger(),
                "First ground-truth odom received at stamp=%.3f frame=%s child_frame=%s",
                s.stamp, msg->header.frame_id.c_str(),
                msg->child_frame_id.empty() ? "(empty)" : msg->child_frame_id.c_str());
  }
}

bool gicp_plusplus::LocalizationNode::gtSampleIsRtkFixed(const GtSample& s) const {
  // When the gate is disabled, treat every sample as FIXED -- the operator
  // has explicitly opted into "trust whatever the upstream publishes".
  if (!this->rtk_gate_enabled_) return true;
  return (s.cov_pos_xx <= this->rtk_gate_max_pose_var_xy_) &&
         (s.cov_pos_yy <= this->rtk_gate_max_pose_var_xy_) &&
         (s.cov_pos_zz <= this->rtk_gate_max_pose_var_z_);
}

bool gicp_plusplus::LocalizationNode::getGtPoseAt(double stamp, GtSample& out) {
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
  // Covariance is not linearly interpolated: carry the conservative (larger)
  // variance of the two bracketing samples so an interpolated pose can only
  // pass the RTK gate when BOTH neighbours were RTK-FIXED. Without this the
  // cov fields would keep their defaults and the gate could mis-classify the
  // sample. (Endpoint/single-sample branches above copy a real sample whole,
  // so their covariance is already valid.)
  out.cov_pos_xx = std::max(a->cov_pos_xx, b->cov_pos_xx);
  out.cov_pos_yy = std::max(a->cov_pos_yy, b->cov_pos_yy);
  out.cov_pos_zz = std::max(a->cov_pos_zz, b->cov_pos_zz);
  // Same conservative-max policy for yaw variance (mirrors GLIM gnss_global's
  // interpolation): an interpolated sample only reports healthy heading when
  // BOTH bracketing samples do. max() also does the right thing when one
  // neighbour is unpopulated (-1): the populated (real) variance wins.
  out.cov_yaw = std::max(a->cov_yaw, b->cov_yaw);
  return true;
}

bool gicp_plusplus::LocalizationNode::getGtFiniteDiffVelWorld(
    double stamp, Eigen::Vector3f& v_world_out) {
  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (this->gt_odom_buffer_.size() < 2) return false;
  // Buffer is monotone non-decreasing (out-of-order samples dropped at insert).
  auto it = std::lower_bound(
      this->gt_odom_buffer_.begin(), this->gt_odom_buffer_.end(), stamp,
      [](const GtSample& s, double t) { return s.stamp < t; });
  // Choose a bracketing (or nearest adjacent) pair around the query stamp.
  auto b = (it == this->gt_odom_buffer_.end()) ? std::prev(it) : it;
  auto a = (b == this->gt_odom_buffer_.begin()) ? b : std::prev(b);
  if (a == b) b = std::next(b);  // query before first sample: use first pair
  const double dt = b->stamp - a->stamp;
  if (dt <= 1e-6) return false;
  // Both endpoints must be reasonably close to the query, mirroring
  // getGtPoseAt's staleness contract.
  if (std::min(std::abs(stamp - a->stamp), std::abs(b->stamp - stamp)) >
      this->gt_odom_max_dt_) {
    return false;
  }
  v_world_out = (b->p - a->p) / static_cast<float>(dt);
  return v_world_out.allFinite();
}

bool gicp_plusplus::LocalizationNode::composeGtPoseInBase(
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

bool gicp_plusplus::LocalizationNode::composeGtTwistInBase(
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

void gicp_plusplus::LocalizationNode::resetVelocityShadow(
    const Eigen::Vector3f& p_world, const Eigen::Vector3f& v_world,
    double stamp, const char* reason) {
  if (!this->velocity_shadow_enabled_) return;
  // A known-pose anchor was requested. Even if its velocity payload is
  // malformed, the watchdog must not remain silently unarmed/fail-open.
  this->velocity_shadow_anchor_seen_.store(true);
  if (!p_world.allFinite() || !v_world.allFinite() || !std::isfinite(stamp)) {
    RCLCPP_WARN(this->get_logger(),
                "Velocity shadow: refusing non-finite reset (%s)", reason);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(this->velocity_shadow_mtx_);
    this->velocity_shadow_p_ = p_world;
    this->velocity_shadow_v_world_ = v_world;
    this->velocity_shadow_stamp_ = stamp;
    this->velocity_shadow_anchor_stamp_ = stamp;
    ++this->velocity_shadow_reset_count_;
    this->velocity_shadow_initialized_ = true;
    this->velocity_shadow_history_.clear();
    this->velocity_shadow_history_.push_back(
        VelocityShadowSample{stamp, p_world, v_world});
  }
  RCLCPP_INFO(this->get_logger(),
              "Velocity shadow: anchored from %s at t=%.3f p=[%.2f,%.2f,%.2f] v=[%.2f,%.2f,%.2f]m/s",
              reason, stamp, p_world.x(), p_world.y(), p_world.z(),
              v_world.x(), v_world.y(), v_world.z());
}

void gicp_plusplus::LocalizationNode::updateVelocityShadowFromGt(
    const GtSample& gt) {
  if (!this->velocity_shadow_enabled_ || !this->gt_extrinsics_cached_.load()) return;

  Eigen::Vector3f v_lin_base_body;
  Eigen::Vector3f v_ang_base_body;
  if (!this->composeGtTwistInBase(gt, v_lin_base_body, v_ang_base_body)) return;
  (void)v_ang_base_body;

  // Orientation is allowed: the existing INS heading prior already consumes
  // it. Position is deliberately not read here. Rotate body-FLU velocity into
  // map/local-ENU and integrate with a trapezoidal step.
  const Eigen::Matrix3f R_base_gtbody =
      this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Quaternionf q_gtbody_in_base(R_base_gtbody);
  const Eigen::Quaternionf q_map_base =
      (gt.q * q_gtbody_in_base.conjugate()).normalized();
  const Eigen::Vector3f v_world = q_map_base * v_lin_base_body;
  if (!v_world.allFinite() || !std::isfinite(gt.stamp)) return;

  bool gap_invalidated = false;
  double gap_s = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->velocity_shadow_mtx_);
    if (!this->velocity_shadow_initialized_) return;
    const double dt = gt.stamp - this->velocity_shadow_stamp_;
    if (dt <= 0.0) return;
    if (dt > this->velocity_shadow_max_integration_gap_s_) {
      // Mark the velocity track unavailable until the next explicit
      // known-pose anchor. Silently re-anchoring from ordinary Atlas position
      // would make the watchdog a disguised position/GT gate and invalidate
      // the scientific comparison. When failClosedAfterAnchor=true, the scan
      // gate rejects this state and the existing failure-streak recovery
      // performs an explicit, logged GT snap.
      this->velocity_shadow_initialized_ = false;
      this->velocity_shadow_history_.clear();
      gap_invalidated = true;
      gap_s = dt;
    } else {
      this->velocity_shadow_p_.head<2>() +=
          0.5f * (this->velocity_shadow_v_world_.head<2>() + v_world.head<2>()) *
          static_cast<float>(dt);
      this->velocity_shadow_v_world_ = v_world;
      this->velocity_shadow_stamp_ = gt.stamp;
      this->velocity_shadow_history_.push_back(
          VelocityShadowSample{gt.stamp, this->velocity_shadow_p_, v_world});
      const double keep_after =
          gt.stamp - this->velocity_shadow_history_duration_s_;
      // Keep one sample before the retention boundary so lower_bound queries
      // still have a left-hand interpolation endpoint.
      while (this->velocity_shadow_history_.size() > 2 &&
             this->velocity_shadow_history_[1].stamp < keep_after) {
        this->velocity_shadow_history_.pop_front();
      }
    }
  }
  if (gap_invalidated) {
    RCLCPP_WARN(this->get_logger(),
                "Velocity shadow: Atlas velocity gap %.3fs exceeds %.3fs; watchdog unavailable until next odom-init/GT-snap anchor",
                gap_s, this->velocity_shadow_max_integration_gap_s_);
  }
}

bool gicp_plusplus::LocalizationNode::velocityShadowAt(
    double stamp, Eigen::Vector3f& p_world, double& age_s,
    double& anchor_age_s, uint64_t& reset_count,
    bool& fail_closed_if_unavailable) const {
  std::lock_guard<std::mutex> lock(this->velocity_shadow_mtx_);
  reset_count = this->velocity_shadow_reset_count_;
  fail_closed_if_unavailable =
      this->velocity_shadow_enabled_ &&
      this->velocity_shadow_fail_closed_after_anchor_ &&
      (this->velocity_shadow_anchor_seen_.load() ||
       this->velocity_shadow_reset_count_ > 0 ||
       this->use_odom_init_applied_.load());
  if (std::isfinite(stamp) && std::isfinite(this->velocity_shadow_stamp_)) {
    age_s = stamp - this->velocity_shadow_stamp_;
  }
  if (std::isfinite(stamp) &&
      std::isfinite(this->velocity_shadow_anchor_stamp_)) {
    anchor_age_s = stamp - this->velocity_shadow_anchor_stamp_;
  }
  if (!this->velocity_shadow_enabled_ || !this->velocity_shadow_initialized_ ||
      !std::isfinite(stamp) || this->velocity_shadow_history_.empty()) {
    return false;
  }
  if (!std::isfinite(anchor_age_s) || anchor_age_s < 0.0 ||
      anchor_age_s > this->velocity_shadow_max_anchor_age_s_) {
    return false;
  }

  const auto it = std::lower_bound(
      this->velocity_shadow_history_.begin(),
      this->velocity_shadow_history_.end(), stamp,
      [](const VelocityShadowSample& sample, double query_stamp) {
        return sample.stamp < query_stamp;
      });

  if (it == this->velocity_shadow_history_.begin()) {
    // Never backward-extrapolate a delayed query from a future velocity
    // sample: during a turn that tangent approximation can hide metres of
    // divergence. Exact-front queries are valid; older queries fail closed.
    const double dt = stamp - it->stamp;
    if (!std::isfinite(dt) || dt < -1.0e-9) {
      return false;
    }
    p_world = it->p_world;
  } else if (it == this->velocity_shadow_history_.end()) {
    // Query is newer than the latest velocity sample. Short, bounded
    // constant-velocity holdover is preferable to silently using Atlas
    // position; once it expires the caller fails closed until a known-pose
    // anchor restores the watchdog.
    const auto& last = this->velocity_shadow_history_.back();
    const double dt = stamp - last.stamp;
    age_s = dt;
    if (!std::isfinite(dt) || dt < 0.0 ||
        dt > this->velocity_shadow_max_age_s_) {
      return false;
    }
    p_world = last.p_world +
        last.v_world * static_cast<float>(dt);
  } else {
    // The query lies inside retained integrated history. Cubic Hermite uses
    // both endpoint positions and velocities, avoiding the chord error of
    // plain position lerp across a sparse (up to maxIntegrationGapS) turn.
    const auto& right = *it;
    const auto& left = *std::prev(it);
    const double dt = right.stamp - left.stamp;
    if (!std::isfinite(dt) || dt <= 0.0) return false;
    const float u = static_cast<float>((stamp - left.stamp) / dt);
    const float u2 = u * u;
    const float u3 = u2 * u;
    const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
    const float h10 = u3 - 2.0f * u2 + u;
    const float h01 = -2.0f * u3 + 3.0f * u2;
    const float h11 = u3 - u2;
    p_world = (1.0f - u) * left.p_world + u * right.p_world;
    p_world.head<2>() =
        h00 * left.p_world.head<2>() +
        h10 * static_cast<float>(dt) * left.v_world.head<2>() +
        h01 * right.p_world.head<2>() +
        h11 * static_cast<float>(dt) * right.v_world.head<2>();
  }
  return p_world.head<2>().allFinite();
}

// RTK-driven IMU bias calibration. Pairs each IMU sample with a time-matched GT
// pose/twist and accumulates the bias residual. Linear acceleration in world
// frame is estimated by finite-differencing v_world between successive paired
// samples. On window fill, biases are averaged, the state is seeded from the
// latest GT, imu_calibrated_ is flipped, and the function returns true.
bool gicp_plusplus::LocalizationNode::tryRtkCalibrationStep(
    double stamp, const Eigen::Vector3f& measured_gyro,
    const Eigen::Vector3f& measured_accel) {
  GtSample gt;
  // RTK-driven IMU bias calibration needs CM-LEVEL truth -- only consume
  // RTK-FIXED samples. If only dead-reckoned Atlas poses are available the
  // init machine will time out (rtk_init/fallback_timeout) and drop into
  // stationary calibration.
  if (!this->getGtPoseAt(stamp, gt) || !this->gtSampleIsRtkFixed(gt)) {
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

  // [REVIEW FIX 2026-07-08 P2] Apply biases; seed POSE state only when
  // nothing else has initialized the filter yet. Previously this always
  // re-seeded state/geo.prev_* from the latest RTK sample but left
  // current_pose / basePose / base_pose_stamp_ / prev_vel untouched:
  //   * use_odom_init=true (already initialized): the observer state jumped
  //     to the RTK sample while the GICP scan chain kept integrating from the
  //     old basePose — two briefly divergent pose chains.
  //   * use_odom_init=false: state was seeded but `initialized` was never
  //     set, so callbackPointCloud waited forever.
  // Now: bias-only once initialized; otherwise a FULL seed that mirrors
  // applyInitialPose()/maybeSnapPoseToGT() (current_pose, basePose,
  // base_pose_stamp_, prev_vel, observer state), publishing initialized=true
  // LAST so a concurrent scan callback never sees a half-written seed.
  const bool already_seeded = this->initialized.load();
  const Eigen::Vector3f v_seed_world =
      this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_lin_body;
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.b.gyro = gyro_bias;
    this->state.b.accel = accel_bias;
    if (!already_seeded) {
      this->state.p = this->latest_rtk_seed_.p;
      this->state.q = this->latest_rtk_seed_.q;
      this->state.v.lin.b = this->latest_rtk_seed_.v_lin_body;
      this->state.v.lin.w = v_seed_world;
      this->state.v.ang.b = this->latest_rtk_seed_.v_ang_body;
      this->state.v.ang.w = this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_ang_body;
      this->geo.prev_p = this->latest_rtk_seed_.p;
      this->geo.prev_q = this->latest_rtk_seed_.q;
      this->geo.prev_vel = v_seed_world;
      // [REVIEW FIX 2026-07-08 P2] The state is a complete, valid estimate
      // (pose + velocities + biases): mark the observer initialized, exactly
      // as GT odom-init does. Otherwise the first accepted GICP scan entered
      // the first-time observer branch, which (previously) zeroed the freshly
      // calibrated biases and (still) discards the seeded velocities.
      this->geo.first_opt_done = true;
    }
    ++this->geo.update_seq;  // discard in-flight propagateState computations
  }
  if (!already_seeded) {
    Eigen::Matrix4f T_seed = Eigen::Matrix4f::Identity();
    T_seed.block<3, 3>(0, 0) = this->latest_rtk_seed_.q.toRotationMatrix();
    T_seed.block<3, 1>(0, 3) = this->latest_rtk_seed_.p;
    {
      std::lock_guard<std::mutex> lock(this->pose_mutex);
      this->current_pose = T_seed;
    }
    {
      std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);  // [P2 FIX 2026-07-09]
      this->basePose.p = this->latest_rtk_seed_.p;
      this->basePose.q = this->latest_rtk_seed_.q;
      this->base_pose_stamp_ = this->latest_rtk_seed_.stamp;
      this->prev_vel = v_seed_world;
    }
    this->initialized = true;  // publish only after ALL pose state is consistent
    this->resetVelocityShadow(this->latest_rtk_seed_.p, v_seed_world,
                              this->latest_rtk_seed_.stamp, "RTK full seed");
  }

  this->imu_calibrated_ = true;
  RCLCPP_INFO(this->get_logger(),
              "IMU calibrated (RTK-driven, %d samples, %.1fs, %s): "
              "gyro_bias=[%.4f,%.4f,%.4f] accel_bias=[%.3f,%.3f,%.3f] "
              "seed_pos=[%.2f,%.2f,%.2f] seed_v=[%.2f,%.2f,%.2f]m/s",
              this->rtk_calib_count_, elapsed,
              already_seeded ? "bias-only, pose chains already seeded"
                             : "full pose seed",
              gyro_bias.x(), gyro_bias.y(), gyro_bias.z(),
              accel_bias.x(), accel_bias.y(), accel_bias.z(),
              this->latest_rtk_seed_.p.x(), this->latest_rtk_seed_.p.y(), this->latest_rtk_seed_.p.z(),
              v_seed_world.x(), v_seed_world.y(), v_seed_world.z());
  return true;
}

bool gicp_plusplus::LocalizationNode::maybeSnapPoseToGT(const char* reason) {
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

  // Pose composition: bring gt sample from gt_body_frame into base_frame.
  // Shared with the diagnostic cross-check via composeGtPoseInBase.
  Eigen::Vector3f p_new;
  Eigen::Quaternionf q_new;
  if (!this->composeGtPoseInBase(gt, p_new, q_new)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "GT recovery: composeGtPoseInBase returned false "
                         "(extrinsic not cached). Deferring snap.");
    return false;
  }

  // Twist composition (P2#2): resolve the linear and angular sources
  // INDEPENDENTLY. Run 12/13 Atlas odom carried a valid linear twist but ZERO
  // angular twist (the adapter didn't populate twist.angular from the gyro),
  // and the old all-or-nothing "twist_is_zero" check passed the zeros through
  // — every one of the 1,860 snaps reset the angular-rate state to zero
  // mid-turn. Sources by priority:
  //   angular: GT twist -> live IMU gyro (bias-corrected under geo.mtx below;
  //            angular rate is rigid-body-invariant and the Atlas gyro IS the
  //            measured body rate in these axes) -> zero.
  //   linear:  GT twist -> finite difference of the bracketing GT poses
  //            (handles both an unpopulated twist and a true standstill
  //            uniformly) -> keep the current state velocity (never zero a
  //            moving vehicle's velocity: with the old behavior the next
  //            IMU prior integrates from v=0 and immediately re-fails).
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Matrix3f R_gtbody_base = R_base_gtbody.transpose();
  const Eigen::Vector3f t_gtbody_base = -R_gtbody_base * t_base_gtbody;

  const bool gt_ang_valid = gt.v_ang_body.norm() >= 0.01f;
  const bool gt_lin_valid = gt.v_lin_body.norm() >= 0.05f;

  Eigen::Vector3f omega_base_body = Eigen::Vector3f::Zero();
  bool omega_from_imu = false;
  if (gt_ang_valid) {
    omega_base_body = R_gtbody_base * gt.v_ang_body;
  } else {
    std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
    if (this->first_imu_received &&
        std::abs(this->imu_meas.stamp - this->scan_stamp.seconds()) < 0.2) {
      // P3: imu_meas is bias-corrected at buffering time — use as-is.
      omega_base_body = this->imu_meas.ang_vel;
      omega_from_imu = true;
    }
  }

  Eigen::Vector3f v_base_body = Eigen::Vector3f::Zero();
  bool lin_resolved = false;
  bool lin_from_fd = false;
  if (gt_lin_valid) {
    v_base_body = R_gtbody_base * (gt.v_lin_body + gt.v_ang_body.cross(t_gtbody_base));
    lin_resolved = true;
  } else {
    Eigen::Vector3f v_fd_world;
    if (this->getGtFiniteDiffVelWorld(this->scan_stamp.seconds(), v_fd_world)) {
      v_base_body = q_new.conjugate() * v_fd_world;
      lin_resolved = true;
      lin_from_fd = true;
    }
  }

  static bool warned_twist_sources = false;
  if (!warned_twist_sources && (!gt_ang_valid || !gt_lin_valid)) {
    warned_twist_sources = true;
    RCLCPP_INFO(this->get_logger(),
                "GT recovery: GT twist partially unpopulated (lin=%.3f m/s%s, ang=%.3f rad/s%s); "
                "backfilling angular from %s and linear from %s",
                gt.v_lin_body.norm(), gt_lin_valid ? "" : " INVALID",
                gt.v_ang_body.norm(), gt_ang_valid ? "" : " INVALID",
                gt_ang_valid ? "GT" : (omega_from_imu ? "IMU gyro" : "zero"),
                gt_lin_valid ? "GT" : (lin_from_fd ? "GT pose finite-difference" : "current state"));
  }

  // Apply state. Caller (performLocalization) already holds pose_mutex (line 1768),
  // so we MUST NOT re-acquire it here — std::mutex is non-recursive and that would
  // deadlock the entire scan callback. geo.mtx, however, is taken in narrow scopes
  // by performLocalization, never held across this call site, so locking it here
  // is correct.
  this->current_pose.setIdentity();
  this->current_pose.block<3, 3>(0, 0) = q_new.toRotationMatrix();
  this->current_pose.block<3, 1>(0, 3) = p_new;
  Eigen::Vector3f v_base_world;
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    // (P3: no bias subtraction here — imu_meas is bias-corrected at buffering;
    // omega_from_imu is noted only for the source log above.)
    (void)omega_from_imu;
    if (!lin_resolved) {
      // Last resort: preserve the IMU-propagated velocity through the snap.
      v_base_body = this->state.q.conjugate() * this->state.v.lin.w;
    }
    v_base_world = q_new * v_base_body;
    const Eigen::Vector3f omega_base_world = q_new * omega_base_body;
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
  {
    // [P2 FIX 2026-07-09] Seed writes under the owner lock (pose -> seed:
    // maybeSnapPoseToGT runs inside performLocalization's pose_mutex scope).
    std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
    this->basePose.p = p_new;
    this->basePose.q = q_new;
    // [REVIEW FIX 2026-07-08] The snap pose is the GT sample at the scan stamp.
    this->base_pose_stamp_ = this->scan_stamp.seconds();
    this->prev_vel = v_base_world;
  }

  // Shadow semantics are stricter than recovery velocity semantics: recovery
  // may fall back to a finite difference of Atlas *position*, but the shadow
  // must remain velocity-only between anchors. Re-compose the raw velflu twist
  // independently; a true zero velocity is valid.
  if (this->velocity_shadow_enabled_) {
    // The pose anchor itself is valid even if the velocity payload below is
    // malformed. Mark the watchdog armed now so a failed velocity reset takes
    // the fail-closed path rather than silently reverting to unarmed/fail-open.
    this->velocity_shadow_anchor_seen_.store(true);
  }
  Eigen::Vector3f shadow_v_lin_body;
  Eigen::Vector3f shadow_v_ang_body;
  if (this->composeGtTwistInBase(gt, shadow_v_lin_body, shadow_v_ang_body) &&
      shadow_v_lin_body.allFinite()) {
    (void)shadow_v_ang_body;
    this->resetVelocityShadow(p_new, q_new * shadow_v_lin_body,
                              this->scan_stamp.seconds(), "GT snap");
  } else if (this->velocity_shadow_enabled_) {
    std::lock_guard<std::mutex> shadow_lock(this->velocity_shadow_mtx_);
    this->velocity_shadow_initialized_ = false;
    this->velocity_shadow_history_.clear();
    RCLCPP_WARN(this->get_logger(),
                "Velocity shadow: GT snap had no finite Atlas velflu; watchdog left unavailable");
  }

  Eigen::Matrix4f T_snap = Eigen::Matrix4f::Identity();
  T_snap.block<3, 3>(0, 0) = q_new.toRotationMatrix();
  T_snap.block<3, 1>(0, 3) = p_new;
  this->last_gicp_pose_ = T_snap;
  this->last_gicp_stamp_ = this->scan_stamp;
  this->last_gicp_valid_ = true;

  RCLCPP_WARN(this->get_logger(),
              "Localization: ⟳ snapped pose to GT (%s after %d consecutive non-accepts) — "
              "pose=[%.2f,%.2f,%.2f] v=[%.2f,%.2f,%.2f]m/s ω=[%.2f,%.2f,%.2f]rad/s | gt_body=%s",
              reason, this->consecutive_failures_,
              p_new.x(), p_new.y(), p_new.z(),
              v_base_world.x(), v_base_world.y(), v_base_world.z(),
              omega_base_body.x(), omega_base_body.y(), omega_base_body.z(),
              this->gt_body_frame_.c_str());

  {
    geometry_msgs::msg::PoseStamped snap_msg;
    snap_msg.header.stamp = this->scan_stamp;
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

  this->consecutive_failures_ = 0;
  // A GT snap restores a known-good pose, so restart the dead-reckon clock (P3).
  this->last_accepted_scan_stamp_ = this->scan_stamp.seconds();
  return true;
}

void gicp_plusplus::LocalizationNode::callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu) {

  double stamp = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;

  Eigen::Vector3f ang_vel(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
  Eigen::Vector3f lin_accel(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

  // One-shot defensive check: warn if the incoming IMU header.frame_id does
  // not match the configured imu_frame. The single-source P1 design assumes
  // both are "gps_antenna_top"; any other combination usually indicates the
  // imu_topic launch arg was re-pointed at a different IMU (e.g. a NovAtel
  // or VectorNav source) without also updating localization/imu_frame. The
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
  // P3 (bonus): buffer BIAS-CORRECTED IMU so every consumer — propagateState,
  // integrateImu (T_prior) and the per-point deskew frames — integrates the
  // same corrected signal. Previously only propagateState subtracted state.b;
  // the prior/deskew path integrated the raw gyro/accel, so once RTK
  // calibration set a nonzero bias the two integration paths permanently
  // disagreed (worst during high-yaw-rate sweeps, where deskew rotation error
  // scales directly with the gyro bias). Mirrors upstream DLIO, which applies
  // the bias in the IMU callback before buffering.
  // NOTE: the calibration paths below intentionally keep consuming the RAW
  // ang_vel/lin_accel locals — bias estimation must see the uncorrected
  // signal. state.b is zero until calibration completes, so pre-calibration
  // buffered samples are unaffected.
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    imu_meas_temp.ang_vel = ang_vel - this->state.b.gyro;
    imu_meas_temp.lin_accel = lin_accel - this->state.b.accel;
  }

  // Calculate dt
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      const double newest_stamp = this->imu_buffer.front().stamp;
      // [REVIEW FIX 2026-07-08 P2] Enforce buffer monotonicity. The IMU
      // subscription is in a REENTRANT callback group: two callbacks can
      // finish out of timestamp order, which previously produced a NEGATIVE
      // dt and a non-monotonic buffer — imuMeasFromTimeRange() assumes strict
      // newest-to-oldest order, so a single inversion corrupts propagation,
      // deskew and the yaw priors. Drop the regressed/duplicate sample; at
      // 100+ Hz the information loss is negligible and every downstream
      // invariant holds.
      if (stamp <= newest_stamp) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "IMU sample out of order (%.6f <= newest %.6f) — dropped to keep "
                             "the buffer monotonic (reentrant callback race)",
                             stamp, newest_stamp);
        return;
      }
      imu_meas_temp.dt = stamp - newest_stamp;
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
  // ----- Initialization / IMU-bias calibration (seed-and-go) -----
  // Localization output must never stop. We DECOUPLE output from bias
  // calibration: as soon as a usable state seed exists (initialized with a
  // pose/orientation from GT odom-init, a param initial pose, or the first
  // GICP scan), we fall through to propagateState() below and publish
  // continuously, using the most stable information available at the time.
  // Bias calibration (RTK-driven or stationary) then runs in the BACKGROUND
  // and applies its refined bias as a smooth correction when it completes --
  // no output gap. We suppress output (early return) ONLY while there is
  // genuinely nothing to propagate from yet, i.e. before any seed exists; in
  // that unseeded window the original WAITING -> RTK/STATIONARY fallback logic
  // still applies so we reach a first fix as fast as possible.
  if (!this->imu_calibrated_) {
    // [REVIEW FIX 2026-07-08 P2] Serialize the init/bias-calibration state
    // machine. The IMU subscription is in a REENTRANT callback group under a
    // MultiThreadedExecutor: without this lock, parallel IMU callbacks could
    // interleave in the WAITING/RTK/STATIONARY phase transitions, corrupt the
    // RTK/stationary accumulator sums, or double-finalize the calibration.
    // Scope: startup only — once imu_calibrated_ is true the outer atomic
    // check bypasses the lock forever.
    std::lock_guard<std::mutex> calib_lock(this->calib_mtx_);
    if (this->imu_calibrated_) {
      // Another IMU thread finalized calibration while we waited on the
      // lock; fall through to normal propagation below.
    } else {

    InitPhase phase = this->init_phase_.load();

    // True once we have something stable to propagate from. Mirrors the
    // propagateState() gate below so that "fall through" always yields output.
    const bool can_propagate_now =
        this->initialized.load() &&
        (this->geo.first_opt_done.load() || this->imu_only_mode_);

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
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "RTK init: waiting for first GT odom (elapsed=%.1f/%.1fs)",
                             stamp - this->first_imu_stamp_, this->rtk_fallback_timeout_sec_);
        // Seed-and-go: only block output if we have no seed yet. With a param
        // seed we dead-reckon here until GT arrives instead of going dark.
        if (!can_propagate_now) return;
      }
    }

    if (phase == InitPhase::RTK_CALIBRATING) {
      // Accumulate an RTK-driven bias estimate in the background. It only
      // completes on RTK-FIXED pairings; with degraded-only Atlas odom it
      // never finishes -- but that is now BENIGN, because once a seed exists
      // we keep propagating below rather than stalling, and a later FIXED
      // sample still completes the calibration opportunistically. We only
      // force a stationary fallback while still UNSEEDED (output dark), so a
      // usable estimate is reached promptly in that case (the P1 stall fix).
      if (!can_propagate_now) {
        const double rtk_elapsed = stamp - this->rtk_calib_start_stamp_;
        const double rtk_phase_budget =
            this->rtk_calib_window_sec_ + this->rtk_fallback_timeout_sec_;
        if (rtk_elapsed > rtk_phase_budget) {
          RCLCPP_WARN(this->get_logger(),
                      "RTK init: %.1fs in RTK_CALIBRATING with no seed and no usable "
                      "RTK-FIXED GT pairing; falling back to stationary IMU calibration",
                      rtk_elapsed);
          this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
          this->imu_calib_start_stamp_ = stamp;  // restart window in the stationary path
          return;  // still unseeded — stationary calibration begins next IMU
        }
      }
      if (this->tryRtkCalibrationStep(stamp, ang_vel, lin_accel)) {
        // tryRtkCalibrationStep applied biases + seeded state and set imu_calibrated_.
        this->init_phase_ = InitPhase::DONE;
      } else if (!this->imu_calibrated_ && !can_propagate_now) {
        return;  // no seed yet — keep output suppressed until we have one
      }
      // Seeded: fall through and propagate; RTK bias refines opportunistically.
    } else if (phase == InitPhase::STATIONARY_CALIBRATING) {
      // Stationary path: assume omega=0 and accel direction = gravity. Used
      // when no GT seed is available; it provides the initial gravity/orientation.
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

        // [P2 FIX 2026-07-09] Gravity SIGN. This rig's documented convention
        // (see the specific-force note in tryRtkCalibrationStep) is that a
        // level stationary body reads accel ~= (0,0,+g), and the propagation
        // math (integrateImu / propagateState) computes world_accel = q*accel
        // then subtracts +g on world Z — which requires q_init to map the
        // measured accel direction onto world +Z. The previous (0,0,-1)
        // target was a pi flip: FromTwoVectors(+z, -z) started the observer
        // upside-down with arbitrary yaw; the bias math was self-consistent
        // with the flip (so nothing caught it), propagateState produced -2g
        // vertical acceleration, and the delta-form updateState never
        // corrects absolute attitude, so the IMU-rate output stayed flipped
        // until a GT snap. Both the target vector and expected_grav_body
        // flip TOGETHER so the bias estimate stays correct.
        Eigen::Vector3f grav_world(0.f, 0.f, +1.f);
        Eigen::Vector3f grav_body = accel_avg.normalized();
        Eigen::Quaternionf q_init = Eigen::Quaternionf::FromTwoVectors(grav_body, grav_world);

        Eigen::Vector3f expected_grav_body = q_init.conjugate()._transformVector(
            Eigen::Vector3f(0.f, 0.f, +static_cast<float>(this->gravity_)));

        // [REVIEW FIX 2026-07-08 P3] ALL observer-state writes under geo.mtx.
        // The IMU subscription runs in a REENTRANT callback group, so parallel
        // IMU callbacks can be inside propagateState()/bias reads while this
        // branch fires — the previous unlocked state.b.gyro / state.b.accel
        // stores were a real data race.
        {
          std::lock_guard<std::mutex> lock(this->geo.mtx);
          this->state.b.gyro = gyro_avg;
          this->state.b.accel = accel_avg - expected_grav_body;
          // Only adopt the accel-derived orientation when we have no better
          // one. [P2 FIX 2026-07-09] "Better one" must cover ALL seed
          // sources: the old guard checked only the param-pose and GT
          // odom-init flags, so an /initialpose (RViz) seed — which sets
          // `initialized` and possibly `first_opt_done` but neither flag —
          // was clobbered when the stationary window completed seconds
          // later, permanently (the delta-form observer never restores
          // absolute attitude, and first_opt_done being true blocks the
          // first-scan re-init). Overwriting a live estimate also violates
          // the stationary assumption if the vehicle has started moving.
          if (!this->use_param_initial_pose_ && !this->use_odom_init_applied_ &&
              !this->initialized.load() && !this->geo.first_opt_done.load()) {
            this->state.q = q_init;
            this->geo.prev_q = q_init;
          }
          ++this->geo.update_seq;  // discard in-flight propagateState computations
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
      } else if (!can_propagate_now) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "IMU calibrating (stationary)... %.1f/%.1fs (%d samples)",
                             elapsed, this->imu_calib_time_, this->imu_calib_count_);
        return;  // no seed yet — suppress output until gravity/orientation known
      }
      // Seeded: fall through and propagate while stationary stats accumulate.
    }
  }
  }

  // Propagate state with geometric observer (only after initialization)
  // Note: function-local statics here and in propagateState() (skip_count,
  // propagate_count, logged_first_publish, path_decimator, odom_publish_count,
  // last_report_time) are safe ONLY because the IMU callback group is
  // MutuallyExclusive (see its creation) — no two IMU callbacks run
  // concurrently. thread_local was wrong under exclusivity too: exclusive
  // callbacks can hop threads, splitting the counts.
  static int propagate_calls = 0;
  static int imu_total = 0;
  static bool logged_first_propagate = false;
  imu_total++;

  if (this->initialized && (this->geo.first_opt_done || this->imu_only_mode_)) {
    this->propagateState(imu_meas_temp);
    propagate_calls++;

    // Log first successful propagation
    if (!logged_first_propagate) {
      RCLCPP_INFO(this->get_logger(), "First IMU propagation successful! Starting high-frequency odometry.");
      logged_first_propagate = true;
    }
  }

  // Debug: Log IMU and propagation rates periodically
  static int imu_count = 0;
  if (++imu_count % 100 == 0) {  // Log every 100 IMU messages (~1 second)
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    RCLCPP_INFO(this->get_logger(), "IMU rate check: %d callbacks, %d propagations, initialized=%d, geo_init=%d",
                imu_total, propagate_calls, this->initialized.load(), this->geo.first_opt_done.load());
    imu_total = 0;
    propagate_calls = 0;
  }
}

bool gicp_plusplus::LocalizationNode::imuMeasFromTimeRange(
    double start_time, double end_time, std::vector<ImuMeas>& out) {

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

  // [REVIEW FIX 2026-07-08 P1] Copy the slice out (forward time order: from
  // the sample just before start_time through the sample at/after end_time)
  // while STILL holding mtx_imu. Iterators into the circular buffer must not
  // survive past the lock: a concurrent IMU push_front invalidates them.
  out.assign(boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it),
             boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it));

  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_plusplus::LocalizationNode::integrateImu(
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

  std::vector<ImuMeas> imu_slice;
  if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), imu_slice) == false) {
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

  if (imu_slice.size() < 2) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT slice<2: start=%.6f end=%.6f first.stamp=%.6f last.stamp=%.6f\n",
                   start_time, sorted_timestamps.back(),
                   imu_slice.empty() ? 0.0 : imu_slice.front().stamp,
                   imu_slice.empty() ? 0.0 : imu_slice.back().stamp);
      std::fflush(stderr);
    }
    return empty;
  }

  const ImuMeas& f1 = imu_slice[0];
  const ImuMeas& f2 = imu_slice[1];

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

  return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, imu_slice);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_plusplus::LocalizationNode::integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double>& sorted_timestamps,
    const std::vector<ImuMeas>& imu_slice) {

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(imu_slice.front().lin_accel);
  a[2] -= this->gravity_;

  // Iterate over IMU measurements and timestamps (imu_slice is a private
  // copy in forward time order — no shared-buffer iterators, see P1 fix)
  auto prev_imu_it = imu_slice.begin();
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != imu_slice.end(); imu_it++) {

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

void gicp_plusplus::LocalizationNode::propagateState(const ImuMeas& imu_local) {

  // [REVIEW FIX 2026-07-08 P1] The caller's OWN sample is propagated, not the
  // shared this->imu_meas latest — under the old Reentrant group, reading the
  // global latest meant an older callback could propagate a newer sample or
  // two callbacks could propagate the same one. With the now-exclusive IMU
  // group this is belt-and-braces, but it also makes the data flow exact:
  // each sample is propagated exactly once, in arrival order.

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

  // P3: biases are now subtracted ONCE, at buffering time in callbackImu, so
  // the buffered measurement is already corrected — do NOT subtract again here
  // (bias_gyro/bias_accel are still read above for the periodic status log).
  Eigen::Vector3f ang_vel_corrected = imu_local.ang_vel;
  Eigen::Vector3f lin_accel_corrected = imu_local.lin_accel;

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
  // When dead-reckoning (consecutive GICP failures) inflate by elapsed time and
  // distance travelled (speed*time), not by missed-scan count (P3).
  {
    const double kBaseSigmaXY  = 0.05;   // m   — floor for accepted scans
    const double kBaseSigmaZ   = 0.10;   // m   — z less constrained by LiDAR
    const double kBaseSigmaRot = 0.01;   // rad — roll/pitch/yaw floor
    const double kFitnessScale = 1.0;    // sigma_xy = max(base, scale * sqrt(fitness))
    // [P2 FIX 2026-07-09] NON-BLOCKING covariance snapshot. performLocalization
    // holds pose_mutex across the entire GICP solve (p50 18 ms, p99 68 ms);
    // taking it here — on EVERY IMU callback, in a MutuallyExclusive group —
    // stalled the whole IMU pipeline for up to gicp_ms each scan (output gaps
    // + delayed buffering => staler deskew clamp). try_lock instead: on
    // contention we publish the previous snapshot (covariance inputs change
    // once per scan; one-cycle staleness is immaterial). Statics are safe:
    // the IMU group is MutuallyExclusive.
    static bool cov_last_gicp_valid = false;
    static double cov_last_accepted_fitness = -1.0;
    static int cov_consecutive_failures = 0;
    static double cov_last_accepted_scan_stamp = -1.0;
    {
      std::unique_lock<std::mutex> pose_lock(this->pose_mutex, std::try_to_lock);
      if (pose_lock.owns_lock()) {
        cov_last_gicp_valid = this->last_gicp_valid_;
        cov_last_accepted_fitness = this->last_accepted_fitness_score_;
        cov_consecutive_failures = this->consecutive_failures_;
        cov_last_accepted_scan_stamp = this->last_accepted_scan_stamp_;
      }
    }

    double s_xy, s_z, s_rot;
    if (cov_last_gicp_valid && cov_last_accepted_fitness >= 0.0) {
      double f_sigma = kFitnessScale * std::sqrt(cov_last_accepted_fitness);
      s_xy  = std::max(kBaseSigmaXY,  f_sigma);
      s_z   = std::max(kBaseSigmaZ,   2.0 * f_sigma);
      s_rot = std::max(kBaseSigmaRot, 0.1 * f_sigma);
    } else {
      s_xy  = kBaseSigmaXY;
      s_z   = kBaseSigmaZ;
      s_rot = kBaseSigmaRot;
    }

    // Dead-reckoning inflation (P3). While GICP is failing the estimate rides on
    // IMU integration, whose error grows with DISTANCE travelled (speed*time)
    // plus a slow time term -- not with the raw number of missed scans. At racing
    // speed N missed scans cover ~4x more ground at 200 mph than at 100 mph, so a
    // count-based term under-reports uncertainty exactly when it matters. Keyed on
    // consecutive_failures_ (the real dead-reckon signal) and added on top of the
    // base/fitness sigma above. dr_cov_time_rate_ = dr_cov_dist_frac_ = 0 disables.
    if (cov_consecutive_failures > 0) {
      const double elapsed_dr = (cov_last_accepted_scan_stamp > 0.0)
          ? std::max(0.0, imu_local.stamp - cov_last_accepted_scan_stamp)
          : 0.0;
      const double speed = static_cast<double>(new_v_lin_w.norm());
      const double drift = this->dr_cov_time_rate_ * elapsed_dr +
                           this->dr_cov_dist_frac_ * speed * elapsed_dr;
      s_xy  += drift;
      s_z   += 2.0 * drift;
      s_rot += 0.05 * drift;
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

void gicp_plusplus::LocalizationNode::updateState() {

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

  // Bound the effective timestep used for the proportional corrections. The
  // observer applies dt*K, which is forward-Euler and stable only for dt*K < 2;
  // at Kv=11.25 that bound is hit at dt≈0.18 s, so a 0.3-0.5 s scan gap (dropped
  // Luminar frames / high-speed racing) would otherwise inject an unstable
  // correction from a single GICP residual. At nominal ~10 Hz dt_eff == dt.
  const double dt_eff = std::min(dt, this->geo_observer_dt_max_);

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

  // P3: delta-form correction target. basePose (pin/qin) is the GICP result
  // at the scan's MEDIAN POINT TIME; by now the state has been IMU-propagated
  // 0.1-0.3 s past it (half sweep + queueing + solve). Pulling the current
  // state toward the stale absolute pose drags it backwards along the
  // trajectory — zero-mean on straights, a systematic yaw/position lag in
  // turns. Instead, form the time-free world-frame correction
  //   T_corr = T_meas * inv(T_prior)   (both at median scan time)
  // and target T_corr (x) current_state: if IMU and GICP agree, T_corr = I and
  // the correction vanishes regardless of latency. Gains/structure unchanged.
  if (this->geo_delta_correction_ && matrixFinite(this->observer_prior_pose_)) {
    Eigen::Quaternionf q_prior(this->observer_prior_pose_.block<3, 3>(0, 0));
    q_prior.normalize();
    const Eigen::Vector3f p_prior = this->observer_prior_pose_.block<3, 1>(0, 3);
    const Eigen::Quaternionf q_corr = (qin * q_prior.conjugate()).normalized();
    const Eigen::Vector3f t_corr = pin - q_corr._transformVector(p_prior);
    // Sanity: the prior/measurement delta is bounded by the jump gate on
    // accepted scans; a huge delta here means observer_prior_pose_ is stale
    // or corrupted — fall back to the legacy absolute target for this update.
    const double corr_angle_deg =
        2.0 * std::acos(std::clamp(static_cast<double>(std::abs(q_corr.w())), 0.0, 1.0)) * 180.0 / M_PI;
    const float corr_trans =
        (q_corr._transformVector(this->state.p) + t_corr - this->state.p).norm();
    if (corr_angle_deg < 45.0 && std::isfinite(corr_trans) && corr_trans < 100.0f) {
      pin = q_corr._transformVector(this->state.p) + t_corr;
      qin = (q_corr * this->state.q).normalized();
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "updateState: delta correction implausible (rot=%.1fdeg trans=%.2fm); "
                           "using legacy absolute target this update",
                           corr_angle_deg, static_cast<double>(corr_trans));
    }
  }

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Construct error quaternion
  qe = qhat.conjugate() * qin;

  // P1 yaw-safety fix #3: bound the per-update ORIENTATION correction the way
  // position/velocity already can be bounded. qe is the body-frame rotation
  // error the observer will pull toward with gain dt_eff*Kq (~0.45/update);
  // without a clamp, one bad accepted scan with a large heading error injects
  // dt_eff*Kq*err yaw in a single update (runs 19/20: tens of degrees). Clamp
  // the error's yaw component (body z ~ heading on a near-level vehicle) and
  // optionally its total magnitude BEFORE the gain is applied. Legitimate
  // corrections converge unaffected: a clamped 5 deg yaw error still pulls
  // ~2.3 deg/update at 10 Hz — over 20 deg/s of authority.
  if (this->geo_max_yaw_correction_deg_ > 0.0 || this->geo_max_rot_correction_deg_ > 0.0) {
    constexpr float kDeg2RadF = static_cast<float>(M_PI / 180.0);
    Eigen::AngleAxisf aa_e(qe);
    Eigen::Vector3f rv = aa_e.angle() * aa_e.axis();
    bool clamped = false;
    if (this->geo_max_yaw_correction_deg_ > 0.0) {
      const float max_yaw = static_cast<float>(this->geo_max_yaw_correction_deg_) * kDeg2RadF;
      if (std::abs(rv.z()) > max_yaw) {
        rv.z() = std::copysign(max_yaw, rv.z());
        clamped = true;
      }
    }
    if (this->geo_max_rot_correction_deg_ > 0.0) {
      const float max_rot = static_cast<float>(this->geo_max_rot_correction_deg_) * kDeg2RadF;
      const float n = rv.norm();
      if (n > max_rot) {
        rv *= max_rot / n;
        clamped = true;
      }
    }
    if (clamped) {
      const float ang = rv.norm();
      qe = (ang > 1e-9f)
               ? Eigen::Quaternionf(Eigen::AngleAxisf(ang, rv / ang))
               : Eigen::Quaternionf::Identity();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "updateState: orientation error clamped (yaw<=%.1fdeg, rot<=%.1fdeg) "
                           "before applying observer gain — inspect yaw_innovation_deg",
                           this->geo_max_yaw_correction_deg_, this->geo_max_rot_correction_deg_);
    }
  }

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
  // Point One (Atlas) INS input so GICP residuals do not chase drift by rewriting the
  // trusted IMU bias estimate. Setting Kab/Kgb > 0 restores the upstream DLIO
  // adaptive observer behavior.
  if (this->geo_Kab_ > 0.0) {
    const double abias_max = this->geo_abias_max_;
    this->state.b.accel -= dt_eff * this->geo_Kab_ * err_body;
    this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);
  }

  if (this->geo_Kgb_ > 0.0) {
    const double gbias_max = this->geo_gbias_max_;
    this->state.b.gyro[0] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.x();
    this->state.b.gyro[1] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.y();
    this->state.b.gyro[2] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.z();
    this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);
  }

  // Proportional observer correction (matching upstream DLIO design), using the
  // bounded dt_eff and optional hard clamps on the per-update correction
  // magnitude so one GICP residual after a gap can't yank the state.
  // Position correction
  Eigen::Vector3f pos_corr = dt_eff * this->geo_Kp_ * err;
  if (this->geo_max_pos_correction_ > 0.0) {
    const float n = pos_corr.norm();
    if (n > this->geo_max_pos_correction_) {
      pos_corr *= static_cast<float>(this->geo_max_pos_correction_ / n);
    }
  }
  this->state.p += pos_corr;

  // Velocity correction
  Eigen::Vector3f vel_corr = dt_eff * this->geo_Kv_ * err;
  if (this->geo_max_vel_correction_ > 0.0) {
    const float n = vel_corr.norm();
    if (n > this->geo_max_vel_correction_) {
      vel_corr *= static_cast<float>(this->geo_max_vel_correction_ / n);
    }
  }
  this->state.v.lin.w += vel_corr;

  // Ground vehicle constraint: damp vertical velocity toward zero.
  // A ground vehicle's true Z-velocity is ~0; residual gravity miscompensation
  // causes vel_z to drift. Apply exponential decay each update.
  this->state.v.lin.w[2] *= (1.0f - dt_eff * this->geo_Kz_damping_);

  // Orientation correction
  this->state.q.w() += dt_eff * this->geo_Kq_ * qcorr.w();
  this->state.q.vec() += dt_eff * this->geo_Kq_ * qcorr.vec();
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
    // [REVIEW FIX 2026-07-08 P3] Keep the calibrated biases through the
    // emergency reset when they are still finite — they are calibration, not
    // drifting state. Zero them only if uncalibrated or themselves corrupt.
    if (!this->imu_calibrated_.load() ||
        !this->state.b.accel.allFinite() || !this->state.b.gyro.allFinite()) {
      this->state.b.accel = Eigen::Vector3f::Zero();
      this->state.b.gyro = Eigen::Vector3f::Zero();
    }
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
