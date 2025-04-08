#pragma once

#include "drake/common/eigen_types.h"

namespace drake {

/// Returns @p scalar as a double.  Never throws.
inline double ExtractDoubleOrThrow(double scalar) {
  return scalar;
}

/// Returns @p scalar as a double.  Never throws.
inline double ExtractDoubleOrThrow(float scalar) {
  return static_cast<double>(scalar);
}

/// Returns @p matrix as an Eigen::Matrix<double, ...> with the same size
/// allocation as @p matrix.  Calls ExtractDoubleOrThrow on each element of the
/// matrix, and therefore throws if any one of the extractions fail.
template <typename Derived>
typename std::enable_if_t<std::is_same_v<typename Derived::Scalar, double>,
                          MatrixLikewise<double, Derived>>
ExtractDoubleOrThrow(const Eigen::MatrixBase<Derived>& matrix) {
  return matrix
      .unaryExpr([](const typename Derived::Scalar& value) {
        return ExtractDoubleOrThrow(value);
      })
      .eval();
}

}  // namespace drake
