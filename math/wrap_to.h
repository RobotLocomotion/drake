#pragma once

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {
namespace math {

/// For variables that are meant to be periodic, (e.g. over a 2π interval),
/// wraps `value` into the interval `[low, high)`.  For example:
///   wrap_to(.1, 0, 1) = .1
///   wrap_to(1, 0, 1) = 0
///   wrap_to(-.1, 0, 1) = .9
///   wrap_to(2.1, 0, 1) = .1
///   wrap_to(-1.1, 0, 1) = .9
///   wrap_to(6, 4, 8) = 6
///   wrap_to(2, 4, 8) = 6
template <class T1, class T2>
T1 wrap_to(const T1& value, const T2& low, const T2& high) {
  DRAKE_ASSERT(low <= high);
  T2 range = high - low;
  using std::floor;
  return value - range * floor((value - low) / range);
}

}  // namespace math
}  // namespace drake
