/// @file
/// Internal implementation details for drake/math/autodiff_gradient.h.

#pragma once

#include <Eigen/Dense>

#include "drake/math/gradient.h"

namespace drake {
namespace math {
namespace gradient_internal {

template <typename Derived>
struct AutoDiffToGradientMatrix {
  typedef typename Gradient<
      Eigen::Matrix<typename Derived::Scalar::Scalar,
                    Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
      Eigen::Dynamic>::type type;
};

}  // namespace gradient_internal
}  // namespace math
}  // namespace drake
