#pragma once

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/math/gradient.h"

namespace drake {
namespace math {
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> VectorToSkewSymmetric(
    const Eigen::MatrixBase<Derived>& p) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>,
                                           drake::kSpaceDimension);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ret;
  ret << 0.0, -p(2), p(1), p(2), 0.0, -p(0), -p(1), p(0), 0.0;
  return ret;
}

}  // namespace math
}  // namespace drake
