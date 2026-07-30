#ifndef GICP_PLUSPLUS_RTK_GATE_HPP
#define GICP_PLUSPLUS_RTK_GATE_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace gicp_plusplus {

// RTK covariance-quality gate primitives (P1 FIX 2026-07-14).
//
// A variance component qualifies only when it is FINITE, NONNEGATIVE, and at
// most the configured limit. The former plain `<= threshold` comparison
// accepted the finite negative sentinel (-1 = "covariance not populated") as
// RTK-quality, letting unknown-quality /gps_p1/filtered_odom samples drive
// the INS heading prior, RTK bias calibration, and the GICP-vs-GT
// cross-check. This matches the adapter's stricter
// /gps_p1/filtered_odom_rtk_fixed gate (finite, nonnegative, thresholded).
// NaN fails closed via the isfinite test.
inline bool rtkCovarianceComponentOk(double var, double max_var,
                                     bool allow_zero_covariance = false) {
  return std::isfinite(var) &&
         (allow_zero_covariance ? var >= 0.0 : var > 0.0) &&
         var <= max_var;
}

inline bool rtkPositionCovarianceOk(double cov_xx, double cov_yy, double cov_zz,
                                    double max_var_xy, double max_var_z,
                                    bool allow_zero_covariance = false) {
  return rtkCovarianceComponentOk(cov_xx, max_var_xy, allow_zero_covariance) &&
         rtkCovarianceComponentOk(cov_yy, max_var_xy, allow_zero_covariance) &&
         rtkCovarianceComponentOk(cov_zz, max_var_z, allow_zero_covariance);
}

// Conservative combine for an INTERPOLATED sample's position variance
// (P1 FIX 2026-07-14b). A plain std::max() lets a single valid endpoint
// dominate: max(valid, -1) == valid, and max(valid, NaN) returns the valid
// first argument under C++ max semantics — so a pose interpolated against an
// unknown/invalid neighbour could still pass rtkPositionCovarianceOk(). If
// EITHER endpoint is non-finite or negative, the interpolated quality is
// unknown: return +inf (fails the gate closed); otherwise the maximum.
inline double rtkCombineInterpolatedVariance(double a, double b) {
  if (!std::isfinite(a) || a < 0.0 || !std::isfinite(b) || b < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(a, b);
}

}  // namespace gicp_plusplus

#endif  // GICP_PLUSPLUS_RTK_GATE_HPP
