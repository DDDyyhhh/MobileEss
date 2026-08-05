#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace fast_lio
{

inline bool mapping_update_usable(bool measurement_valid, int effective_points)
{
  return measurement_valid && effective_points > 0;
}

inline bool valid_extrinsic_vector(
  const std::vector<double> &values, std::size_t expected_size)
{
  if (values.size() != expected_size) {
    return false;
  }

  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace fast_lio
