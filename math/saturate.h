#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"

namespace drake {
namespace math {

template <class T1, class T2, class T3>
DRAKE_DEPRECATED("2023-01-01", "Use C++17's std::clamp instead")
T1 saturate(const T1& value, const T2& low, const T3& high) {
  DRAKE_ASSERT(low <= high);
  return cond(
      value < low, low,
      value > high, high,
      value);
}

}  // namespace math
}  // namespace drake
