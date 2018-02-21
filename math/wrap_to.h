#pragma once

#include <cmath>

#include "drake/common/double_overloads.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace math {

/// For variables that are meant to be periodic, (e.g. over a 2π interval),
/// wraps `value` into the interval `[low, high)`.  Precisely, `wrap_to`
/// returns:
///   value + k*(high-low)
/// for the unique integer value `k` that lands the output in the desired
/// interval. @p low and @p high must be finite, and low < high.
///
template <class T1, class T2>
T1 wrap_to(const T1& value, const T2& low, const T2& high) {
  DRAKE_ASSERT(low < high);
  const T2 range = high - low;
  return value - range * floor((value - low) / range);
  // TODO(russt): jwnimmer preferred the following implementation (which may be
  // numerically better), but fmod is not supported yet by autodiff nor
  // symbolic:
  // using std::fmod;
  // const T1 rem = fmod(value - low, high - low);
  // return if_then_else(rem >= T1(0), low + rem, high + rem);
}


}  // namespace math
}  // namespace drake
