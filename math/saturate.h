#pragma once

// TODO(jwnimmer-tri): Figure out how to remove this include.
#include "drake/common/autodiff.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace math {

/// Saturates the input `value` between upper and lower bounds. If `value` is
/// within `[low, high]` then return it; else return the boundary.
template <class T1, class T2, class T3>
T1 saturate(const T1& value, const T2& low, const T3& high) {
  DRAKE_ASSERT(low <= high);
  return cond(
      value < low, low,
      value > high, high,
      value);
}

}  // namespace math
}  // namespace drake
