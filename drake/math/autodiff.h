/// @file
/// Utilities for arithmetic on AutoDiffScalar.

#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "drake/math/autodiff_internal.h"

/// Overloads round to mimic std::round from <cmath>.
/// Must appear in global namespace so that ADL can select between this
/// implementation and the STL one.
template <typename DerType>
double round(const Eigen::AutoDiffScalar<DerType>& x) {
  return round(x.value());
}

/// Overloads floor to mimic std::floor from <cmath>.
/// Must appear in global namespace so that ADL can select between this
/// implementation and the STL one.
template <typename DerType>
double floor(const Eigen::AutoDiffScalar<DerType>& x) {
  return floor(x.value());
}

namespace drake {
namespace math {

template <typename Derived>
typename autodiff_internal::AutoDiffToValueMatrix<Derived>::type
autoDiffToValueMatrix(const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  typename autodiff_internal::AutoDiffToValueMatrix<Derived>::type ret(
      auto_diff_matrix.rows(), auto_diff_matrix.cols());
  for (int i = 0; i < auto_diff_matrix.rows(); i++) {
    for (int j = 0; j < auto_diff_matrix.cols(); ++j) {
      ret(i, j) = auto_diff_matrix(i, j).value();
    }
  }
  return ret;
}

}  // namespace math
}  // namespace drake
