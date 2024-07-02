#pragma once

#include <map>
#include <string>

#include <Eigen/Core>

#include "drake/common/string_map.h"

namespace drake {
namespace internal {

// This file contains a trivial SIMD-enabled helper function, intended as a test
// fixture that allows us to unit test drake/common/hwy_dynamic.h.

/* Compute y = x0 ⊙ x1 (i.e., elementwise multiplication). */
void ArrayMul(const Eigen::ArrayXd& x0, const Eigen::ArrayXd& x1,
              Eigen::ArrayXd* y);

/* Returns some instrumentation counters on ArrayMul() calls. (This is mutable
so that the test can clear it; we don't expect tests to write to it.) */
string_map<int>& GetMutableArrayMulCounters();

}  // namespace internal
}  // namespace drake
