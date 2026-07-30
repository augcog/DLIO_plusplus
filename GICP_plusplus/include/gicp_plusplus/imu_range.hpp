#ifndef GICP_PLUSPLUS_IMU_RANGE_HPP
#define GICP_PLUSPLUS_IMU_RANGE_HPP

#include <cmath>
#include <limits>
#include <vector>

namespace gicp_plusplus {

// Select an oldest->newest IMU slice from a container stored newest->oldest.
// The result contains the nearest real sample at/before start_time (or one up
// to older_tolerance_s after it), every interior sample, and the first sample
// at/after end_time. A strictly monotone buffer and at least two output samples
// are required.
template <typename Container>
bool selectBracketedImuRange(
    const Container& newest_to_oldest, double start_time, double end_time,
    double older_tolerance_s,
    std::vector<typename Container::value_type>& out) {
  out.clear();
  if (newest_to_oldest.empty() ||
      !std::isfinite(start_time) || !std::isfinite(end_time) ||
      !std::isfinite(older_tolerance_s) || older_tolerance_s < 0.0 ||
      start_time > end_time) {
    return false;
  }

  const double newest = newest_to_oldest.front().stamp;
  const double oldest = newest_to_oldest.back().stamp;
  if (!std::isfinite(newest) || !std::isfinite(oldest) ||
      newest < end_time || oldest - start_time > older_tolerance_s) {
    return false;
  }

  auto start_it = newest_to_oldest.rend();
  double previous_stamp = -std::numeric_limits<double>::infinity();
  for (auto it = newest_to_oldest.rbegin();
       it != newest_to_oldest.rend(); ++it) {
    if (!std::isfinite(it->stamp) || it->stamp <= previous_stamp) {
      return false;
    }
    previous_stamp = it->stamp;
    if (it->stamp <= start_time) {
      start_it = it;
    } else {
      break;
    }
  }
  if (start_it == newest_to_oldest.rend()) {
    // Startup phase: the oldest retained IMU may land just after the requested
    // time. The precheck above limits this extrapolation to older_tolerance_s.
    start_it = newest_to_oldest.rbegin();
  }

  previous_stamp = -std::numeric_limits<double>::infinity();
  for (auto it = start_it; it != newest_to_oldest.rend(); ++it) {
    if (!std::isfinite(it->stamp) || it->stamp <= previous_stamp) {
      out.clear();
      return false;
    }
    previous_stamp = it->stamp;
    out.push_back(*it);
    if (it->stamp >= end_time && out.size() >= 2) {
      return true;
    }
  }
  out.clear();
  return false;
}

}  // namespace gicp_plusplus

#endif  // GICP_PLUSPLUS_IMU_RANGE_HPP
