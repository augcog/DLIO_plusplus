#pragma once

#include <cstddef>
#include <vector>

namespace glim {
namespace gnss_detail {

template <typename FactorPointerAt>
bool factor_batch_committed(
  size_t batch_offset,
  const std::vector<const void*>& expected_factors,
  const std::vector<size_t>& new_factor_indices,
  size_t graph_size,
  FactorPointerAt factor_pointer_at) {
  if (
    expected_factors.empty() ||
    batch_offset > new_factor_indices.size() ||
    expected_factors.size() > new_factor_indices.size() - batch_offset) {
    return false;
  }

  for (size_t i = 0; i < expected_factors.size(); ++i) {
    const size_t graph_index = new_factor_indices[batch_offset + i];
    if (
      graph_index >= graph_size ||
      factor_pointer_at(graph_index) != expected_factors[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace gnss_detail
}  // namespace glim
