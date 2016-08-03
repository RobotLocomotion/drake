/// @file
/// Internal implementation details for drake/math/autodiff.h.

#pragma once

#include <Eigen/Dense>

namespace drake {
namespace math {
namespace autodiff_internal {

template <typename Derived>
struct AutoDiffToValueMatrix {
  typedef typename Eigen::Matrix<typename Derived::Scalar::Scalar,
                                 Derived::RowsAtCompileTime,
                                 Derived::ColsAtCompileTime>
      type;
};

}  // namespace autodiff_internal
}  // namespace math
}  // namespace drake
