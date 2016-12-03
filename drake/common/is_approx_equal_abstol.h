#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_compat.h"

namespace drake {

/// Returns true if and only if the two matrices are equal to within a certain
/// absolute elementwise @p tolerance.  Special values (infinities, NaN, etc.)
/// do not compare as equal elements.
template <typename DerivedA, typename DerivedB>
bool is_approx_equal_abstol(const Eigen::MatrixBase<DerivedA>& m1,
                            const Eigen::MatrixBase<DerivedB>& m2,
                            double tolerance) {
  return (
      (m1.rows() == m2.rows()) &&
      (m1.cols() == m2.cols()) &&
      ((m1 - m2).template lpNorm<Eigen::Infinity>() <= tolerance));
}

}  // namespace drake
